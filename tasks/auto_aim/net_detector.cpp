#include "net_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "tools/logger.hpp"

namespace auto_aim
{

struct NetDetector::Impl
{
  struct Slot
  {
    ov::InferRequest infer_request;
    cv::Mat input;
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

    compiled_model_ = core_.compile_model(
      ppp.build(), config_.device,
      ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

    slots_.reserve(config_.infer_request_buffer_num);
    for (int i = 0; i < config_.infer_request_buffer_num; ++i) {
      Slot slot;
      slot.infer_request = compiled_model_.create_infer_request();
      slot.input = cv::Mat(config_.input_height, config_.input_width, CV_8UC3, cv::Scalar(0, 0, 0));
      slots_.push_back(std::move(slot));
      available_slots_.push_back(static_cast<std::size_t>(i));
    }

    tools::logger()->info(
      "[NetDetector] model: {}, device: {}, input: {}x{}, request pool: {}", config_.model_path,
      config_.device, config_.input_width, config_.input_height, config_.infer_request_buffer_num);
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
    if (!config_.use_roi) return {0, 0, image.cols, image.rows};

    const int width = config_.roi.width == -1 ? image.cols - config_.roi.x : config_.roi.width;
    const int height = config_.roi.height == -1 ? image.rows - config_.roi.y : config_.roi.height;
    const cv::Rect roi(config_.roi.x, config_.roi.y, width, height);
    if (roi.x < 0 || roi.y < 0 || roi.width <= 0 || roi.height <= 0 ||
        roi.x + roi.width > image.cols || roi.y + roi.height > image.rows) {
      throw std::runtime_error("Configured ROI is outside the input image");
    }
    return roi;
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
  std::vector<Slot> slots_;
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
    if (!completed_) impl_->slots_[slot_index_].infer_request.wait();
  } catch (...) {
    // Destructors cannot surface OpenVINO failures; the slot is still returned to the pool.
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

  try {
    cv::Mat source = clone_source ? image.clone() : image;
    const double scale = std::min(
      static_cast<double>(impl_->config_.input_width) / roi.width,
      static_cast<double>(impl_->config_.input_height) / roi.height);
    const int resized_width = std::max(1, static_cast<int>(roi.width * scale));
    const int resized_height = std::max(1, static_cast<int>(roi.height * scale));

    const int pad_x = 0;
    const int pad_y = 0;
    auto & slot = impl_->slots_[slot_index];
    slot.input.setTo(cv::Scalar(0, 0, 0));
    cv::resize(source(roi), slot.input(cv::Rect(pad_x, pad_y, resized_width, resized_height)),
      {resized_width, resized_height});
    ov::Tensor input_tensor(
      ov::element::u8,
      {1, static_cast<std::size_t>(impl_->config_.input_height),
        static_cast<std::size_t>(impl_->config_.input_width), 3},
      slot.input.data);
    slot.infer_request.set_input_tensor(input_tensor);
    slot.infer_request.start_async();

    return std::shared_ptr<Ticket>(new Ticket(
      impl_, slot_index, std::move(source), scale, roi,
      impl_->config_.use_roi || roi_override.has_value()));
  } catch (...) {
    impl_->release(slot_index);
    throw;
  }
}

NetDetector::Result NetDetector::wait(const TicketPtr & ticket) const
{
  if (!ticket || ticket->impl_.get() != impl_.get() || ticket->released_) {
    throw std::invalid_argument("Invalid NetDetector ticket");
  }

  auto & request = impl_->slots_[ticket->slot_index_].infer_request;
  request.wait();
  ticket->completed_ = true;

  const auto output_tensor = request.get_output_tensor(0);
  const auto & output_shape = output_tensor.get_shape();
  if (output_shape.size() == 2) {
    return {ticket->source_,
      cv::Mat(static_cast<int>(output_shape[0]), static_cast<int>(output_shape[1]), CV_32F,
        output_tensor.data<float>()),
      ticket->scale_, ticket->roi_, ticket->has_roi_};
  }
  if (output_shape.size() == 3) {
    return {ticket->source_,
      cv::Mat(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
        output_tensor.data<float>()),
      ticket->scale_, ticket->roi_, ticket->has_roi_};
  }
  throw std::runtime_error("NetDetector supports only 2D or 3D output tensors");
}

std::size_t NetDetector::request_capacity() const
{
  return impl_->slots_.size();
}

}  // namespace auto_aim
