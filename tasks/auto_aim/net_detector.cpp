#include "net_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/intel_gpu/properties.hpp>

#ifdef DUST_ENABLE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
static_assert(NV_TENSORRT_MAJOR == 10, "Dust requires TensorRT 10.x");
#endif

#include "tools/logger.hpp"

namespace auto_aim
{

#ifdef DUST_ENABLE_TENSORRT
namespace
{
void cuda_check(cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

class TensorRTLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    try {
      if (severity <= Severity::kERROR) tools::logger()->error("[TensorRT] {}", message);
      else if (severity == Severity::kWARNING) tools::logger()->warn("[TensorRT] {}", message);
    } catch (...) {
    }
  }
};

std::size_t tensor_elements(const nvinfer1::Dims & shape)
{
  if (shape.nbDims <= 0) throw std::runtime_error("TensorRT tensor has no dimensions");
  std::size_t count = 1;
  for (int i = 0; i < shape.nbDims; ++i) {
    if (shape.d[i] <= 0 || shape.d[i] > std::numeric_limits<int>::max() ||
        count > std::numeric_limits<std::size_t>::max() / shape.d[i]) {
      throw std::runtime_error("TensorRT requires positive, static tensor dimensions");
    }
    count *= shape.d[i];
  }
  return count;
}
}  // namespace
#endif

struct NetDetector::Impl
{
  struct Slot
  {
    ov::InferRequest infer_request;
    cv::Mat input;
#ifdef DUST_ENABLE_TENSORRT
    int cuda_device = 0;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream = nullptr;
    void * device_input = nullptr;
    void * device_output = nullptr;
    void * host_input = nullptr;
    void * host_output = nullptr;
    cv::Mat chw;

    ~Slot()
    {
      if (!stream) return;
      cudaSetDevice(cuda_device);
      cudaStreamSynchronize(stream);
      context.reset();
      if (device_input) cudaFree(device_input);
      if (device_output) cudaFree(device_output);
      if (host_input) cudaFreeHost(host_input);
      if (host_output) cudaFreeHost(host_output);
      cudaStreamDestroy(stream);
    }
#endif
  };

  explicit Impl(Config config)
  : config_(std::move(config))
  {
    if (config_.input_width <= 0 || config_.input_height <= 0) {
      throw std::invalid_argument("NetDetector input dimensions must be positive");
    }
    if (config_.infer_request_buffer_num <= 0) {
      throw std::invalid_argument("infer_request_buffer_num must be positive");
    }

    if (config_.backend == "openvino") {
      init_openvino();
    } else if (config_.backend == "tensorrt") {
#ifdef DUST_ENABLE_TENSORRT
      init_tensorrt();
#else
      throw std::runtime_error("TensorRT backend was not built; configure with -DENABLE_TENSORRT=ON");
#endif
    } else {
      throw std::invalid_argument("inference_backend must be openvino or tensorrt");
    }

    slots_.reserve(config_.infer_request_buffer_num);
    for (int i = 0; i < config_.infer_request_buffer_num; ++i) {
      auto slot = std::make_unique<Slot>();
      slot->input = cv::Mat(config_.input_height, config_.input_width, CV_8UC3, cv::Scalar(0, 0, 0));
      if (config_.backend == "openvino") {
        slot->infer_request = compiled_model_.create_infer_request();
      }
#ifdef DUST_ENABLE_TENSORRT
      else {
        slot->cuda_device = config_.cuda_device;
        cuda_check(cudaStreamCreateWithFlags(&slot->stream, cudaStreamNonBlocking), "cudaStreamCreate");
        slot->context.reset(engine_->createExecutionContext());
        if (!slot->context) throw std::runtime_error("TensorRT createExecutionContext failed");
        cuda_check(cudaMalloc(&slot->device_input, input_bytes_), "cudaMalloc input");
        cuda_check(cudaMalloc(&slot->device_output, output_bytes_), "cudaMalloc output");
        cuda_check(cudaMallocHost(&slot->host_input, input_bytes_), "cudaMallocHost input");
        cuda_check(cudaMallocHost(&slot->host_output, output_bytes_), "cudaMallocHost output");
        if (!slot->context->setTensorAddress(input_name_.c_str(), slot->device_input) ||
            !slot->context->setTensorAddress(output_name_.c_str(), slot->device_output)) {
          throw std::runtime_error("TensorRT setTensorAddress failed");
        }
      }
#endif
      slots_.push_back(std::move(slot));
      available_slots_.push_back(static_cast<std::size_t>(i));
    }

    tools::logger()->info(
      "[NetDetector] backend: {}, model: {}, device: {}, input: {}x{}, request pool: {}",
      config_.backend, config_.model_path,
      config_.backend == "tensorrt" ? std::to_string(config_.cuda_device) : config_.device,
      config_.input_width, config_.input_height, config_.infer_request_buffer_num);
  }

  void init_openvino()
  {
    auto model = core_.read_model(config_.model_path);
    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();
    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape(ov::Shape {1, static_cast<std::size_t>(config_.input_height),
        static_cast<std::size_t>(config_.input_width), 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
    input.model().set_layout("NCHW");
    input.preprocess().convert_element_type(ov::element::f32);
    input.preprocess().convert_color(ov::preprocess::ColorFormat::RGB);
    input.preprocess().scale(255.0);
    for (std::size_t i = 0; i < model->outputs().size(); ++i) {
      ppp.output(i).tensor().set_element_type(ov::element::f32);
    }

    ov::AnyMap compile_properties;
    if (config_.performance_mode == "LATENCY") {
      compile_properties[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;
    } else if (config_.performance_mode == "THROUGHPUT") {
      compile_properties[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::THROUGHPUT;
    } else {
      throw std::invalid_argument(
        "performance_mode must be LATENCY or THROUGHPUT, got: " + config_.performance_mode);
    }
    if (config_.device == "GPU" || config_.device == "gpu") {
      // Keep inference ahead of desktop/compositor work on the shared iGPU.
      compile_properties[ov::intel_gpu::hint::queue_priority.name()] = ov::hint::Priority::HIGH;
      compile_properties[ov::intel_gpu::hint::queue_throttle.name()] = ov::hint::Priority::LOW;
    }
    compiled_model_ = core_.compile_model(ppp.build(), config_.device, compile_properties);
  }

#ifdef DUST_ENABLE_TENSORRT
  void init_tensorrt()
  {
    cuda_check(cudaSetDevice(config_.cuda_device), "cudaSetDevice");
    std::ifstream file(config_.model_path, std::ios::binary | std::ios::ate);
    if (!file || file.tellg() <= 0) {
      throw std::runtime_error("Cannot read TensorRT engine: " + config_.model_path);
    }
    const auto size = static_cast<std::size_t>(file.tellg());
    std::vector<char> serialized(size);
    file.seekg(0);
    if (!file.read(serialized.data(), size)) throw std::runtime_error("Cannot read entire TensorRT engine");
    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) throw std::runtime_error("TensorRT createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(serialized.data(), size));
    if (!engine_) throw std::runtime_error("TensorRT engine deserialization failed");
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error("TensorRT detector requires exactly one input and one output");
    }

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE ||
          engine_->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR) {
        throw std::runtime_error("TensorRT detector requires linear device tensors");
      }
      auto & tensor_name = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT ?
        input_name_ : output_name_;
      if (!tensor_name.empty()) throw std::runtime_error("TensorRT engine has duplicate input/output modes");
      tensor_name = name;
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT engine is missing input/output");
    }
    const auto input_shape = engine_->getTensorShape(input_name_.c_str());
    const auto output_shape = engine_->getTensorShape(output_name_.c_str());
    const auto input_elements = tensor_elements(input_shape);
    const auto output_elements = tensor_elements(output_shape);
    if (input_shape.nbDims != 4 || input_shape.d[0] != 1 || input_shape.d[1] != 3 ||
        input_shape.d[2] != config_.input_height || input_shape.d[3] != config_.input_width) {
      throw std::runtime_error("TensorRT input shape must be [1,3,input_height,input_width]");
    }
    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    if (input_type_ != nvinfer1::DataType::kHALF && input_type_ != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("TensorRT input must be FP16 or FP32");
    }
    if (engine_->getTensorDataType(output_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
        (output_shape.nbDims != 2 && output_shape.nbDims != 3) ||
        (output_shape.nbDims == 3 && output_shape.d[0] != 1)) {
      throw std::runtime_error("TensorRT output must be FP32 [rows,cols] or [1,rows,cols]");
    }
    const auto input_element_bytes = input_type_ == nvinfer1::DataType::kHALF ? 2 : sizeof(float);
    if (input_elements > std::numeric_limits<std::size_t>::max() / input_element_bytes ||
        output_elements > std::numeric_limits<std::size_t>::max() / sizeof(float)) {
      throw std::runtime_error("TensorRT tensor byte size overflows");
    }
    input_bytes_ = input_elements * input_element_bytes;
    output_bytes_ = output_elements * sizeof(float);
    output_rows_ = static_cast<int>(output_shape.d[output_shape.nbDims - 2]);
    output_cols_ = static_cast<int>(output_shape.d[output_shape.nbDims - 1]);
    tools::logger()->info(
      "[TensorRT] input '{}' {} [1,3,{},{}]: {} bytes; output '{}' FP32 [{},{}]: {} bytes",
      input_name_, input_type_ == nvinfer1::DataType::kHALF ? "FP16" : "FP32",
      config_.input_height, config_.input_width, input_bytes_, output_name_, output_rows_,
      output_cols_, output_bytes_);
  }
#endif

  void submit(Slot & slot)
  {
    if (config_.backend == "openvino") {
      ov::Tensor input_tensor(
        ov::element::u8,
        {1, static_cast<std::size_t>(config_.input_height),
          static_cast<std::size_t>(config_.input_width), 3},
        slot.input.data);
      slot.infer_request.set_input_tensor(input_tensor);
      slot.infer_request.start_async();
      return;
    }
#ifdef DUST_ENABLE_TENSORRT
    cuda_check(cudaSetDevice(config_.cuda_device), "cudaSetDevice");
    cv::dnn::blobFromImage(slot.input, slot.chw, 1.0 / 255.0, {}, {}, true, false, CV_32F);
    if (input_type_ == nvinfer1::DataType::kHALF) {
      const int shape[] = {1, 3, config_.input_height, config_.input_width};
      cv::Mat half_input(4, shape, CV_16F, slot.host_input);
      slot.chw.convertTo(half_input, CV_16F);
    } else {
      std::memcpy(slot.host_input, slot.chw.data, input_bytes_);
    }
    cuda_check(cudaMemcpyAsync(slot.device_input, slot.host_input, input_bytes_,
      cudaMemcpyHostToDevice, slot.stream), "cudaMemcpyAsync input");
    if (!slot.context->enqueueV3(slot.stream)) throw std::runtime_error("TensorRT enqueueV3 failed");
    cuda_check(cudaMemcpyAsync(slot.host_output, slot.device_output, output_bytes_,
      cudaMemcpyDeviceToHost, slot.stream), "cudaMemcpyAsync output");
#endif
  }

  void wait_slot(std::size_t index)
  {
    auto & slot = *slots_[index];
    if (config_.backend == "openvino") slot.infer_request.wait();
#ifdef DUST_ENABLE_TENSORRT
    else {
      cuda_check(cudaSetDevice(config_.cuda_device), "cudaSetDevice");
      cuda_check(cudaStreamSynchronize(slot.stream), "cudaStreamSynchronize");
    }
#endif
  }

  cv::Mat output(std::size_t index)
  {
    auto & slot = *slots_[index];
#ifdef DUST_ENABLE_TENSORRT
    if (config_.backend == "tensorrt") {
      return cv::Mat(output_rows_, output_cols_, CV_32F, slot.host_output).clone();
    }
#endif
    const auto tensor = slot.infer_request.get_output_tensor(0);
    const auto & shape = tensor.get_shape();
    if ((shape.size() != 2 && shape.size() != 3) || (shape.size() == 3 && shape[0] != 1)) {
      throw std::runtime_error("NetDetector supports only 2D or single-batch 3D output tensors");
    }
    return cv::Mat(static_cast<int>(shape[shape.size() - 2]),
      static_cast<int>(shape.back()), CV_32F, tensor.data<float>()).clone();
  }

  cv::Rect resolve_roi(const cv::Mat & image, std::optional<cv::Rect> roi_override) const
  {
    if (roi_override) {
      const auto bounds = cv::Rect(0, 0, image.cols, image.rows);
      if ((*roi_override & bounds) != *roi_override || roi_override->width <= 0 ||
          roi_override->height <= 0) {
        throw std::runtime_error("Dynamic ROI is outside the input image");
      }
      return *roi_override;
    }
    return {0, 0, image.cols, image.rows};
  }

  void release(std::size_t slot_index) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      available_slots_.push_back(slot_index);
    }
    slot_available_.notify_one();
  }

  Config config_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
#ifdef DUST_ENABLE_TENSORRT
  TensorRTLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::string input_name_, output_name_;
  nvinfer1::DataType input_type_ = nvinfer1::DataType::kHALF;
  std::size_t input_bytes_ = 0, output_bytes_ = 0;
  int output_rows_ = 0, output_cols_ = 0;
#endif
  std::vector<std::unique_ptr<Slot>> slots_;
  std::deque<std::size_t> available_slots_;
  mutable std::mutex mutex_;
  std::condition_variable slot_available_;
};

NetDetector::Ticket::Ticket(std::shared_ptr<Impl> impl, std::size_t slot_index, cv::Mat source,
  double scale, cv::Rect roi, bool has_roi)
: impl_(std::move(impl)), slot_index_(slot_index), source_(std::move(source)), scale_(scale), roi_(roi),
  has_roi_(has_roi)
{
}

NetDetector::Ticket::~Ticket()
{
  release();
}

void NetDetector::Ticket::release() noexcept
{
  if (released_) return;
  try {
    if (!completed_) impl_->wait_slot(slot_index_);
  } catch (...) {
    // Destructors cannot surface inference failures; return the slot after draining it.
  }
  impl_->release(slot_index_);
  released_ = true;
}

NetDetector::NetDetector(Config config)
: impl_(std::make_shared<Impl>(std::move(config)))
{
}

NetDetector::TicketPtr NetDetector::try_start_async(
  const cv::Mat & image, bool clone_source, std::optional<cv::Rect> roi_override)
{
  return start_impl(image, clone_source, false, roi_override);
}

NetDetector::TicketPtr NetDetector::start(const cv::Mat & image, std::optional<cv::Rect> roi_override)
{
  return start_impl(image, false, true, roi_override);
}

NetDetector::TicketPtr NetDetector::start_impl(
  const cv::Mat & image, bool clone_source, bool wait_for_slot, std::optional<cv::Rect> roi_override)
{
  if (image.empty()) return nullptr;
  if (image.type() != CV_8UC3) throw std::invalid_argument("NetDetector expects a CV_8UC3 BGR image");

  // Validate input eagerly so invalid ROIs are reported even when the request pool is busy.
  const cv::Rect roi = impl_->resolve_roi(image, roi_override);

  std::size_t slot_index;
  {
    std::unique_lock<std::mutex> lock(impl_->mutex_);
    if (wait_for_slot) {
      impl_->slot_available_.wait(lock, [this] { return !impl_->available_slots_.empty(); });
    } else if (impl_->available_slots_.empty()) {
      return nullptr;
    }
    slot_index = impl_->available_slots_.front();
    impl_->available_slots_.pop_front();
  }

  bool ticket_owns_slot = false;
  try {
    cv::Mat source = clone_source ? image.clone() : image;
    const double scale = std::min(
      static_cast<double>(impl_->config_.input_width) / roi.width,
      static_cast<double>(impl_->config_.input_height) / roi.height);
    const int resized_width = std::max(1, static_cast<int>(roi.width * scale));
    const int resized_height = std::max(1, static_cast<int>(roi.height * scale));

    const int pad_x = 0;
    const int pad_y = 0;
    auto & slot = *impl_->slots_[slot_index];
    slot.input.setTo(cv::Scalar(0, 0, 0));
    cv::resize(source(roi), slot.input(cv::Rect(pad_x, pad_y, resized_width, resized_height)),
      {resized_width, resized_height});
    impl_->submit(slot);

    auto * ticket = new Ticket(
      impl_, slot_index, std::move(source), scale, roi, roi_override.has_value());
    // shared_ptr deletes the ticket if its control-block allocation fails.
    ticket_owns_slot = true;
    return std::shared_ptr<Ticket>(ticket);
  } catch (...) {
    // Submission may fail after work was queued; buffers must not be reused yet.
    if (!ticket_owns_slot) {
      try {
        impl_->wait_slot(slot_index);
      } catch (...) {
      }
      impl_->release(slot_index);
    }
    throw;
  }
}

NetDetector::Result NetDetector::wait(const TicketPtr & ticket) const
{
  if (!ticket || ticket->impl_.get() != impl_.get() || ticket->released_) {
    throw std::invalid_argument("Invalid NetDetector ticket");
  }

  const auto wait_started = std::chrono::steady_clock::now();
  impl_->wait_slot(ticket->slot_index_);
  const double wait_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - wait_started)
                           .count();
  ticket->completed_ = true;

  return {ticket->source_, impl_->output(ticket->slot_index_), ticket->scale_, ticket->roi_,
    ticket->has_roi_, wait_ms};
}

std::size_t NetDetector::request_capacity() const
{
  return impl_->slots_.size();
}

}  // namespace auto_aim
