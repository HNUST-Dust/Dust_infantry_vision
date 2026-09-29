#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <vector>

#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

int main(int argc, char * argv[])
{
  assert(argc >= 3);

  cv::VideoCapture video(argv[2]);
  cv::Mat image;
  assert(video.read(image) && !image.empty());

  auto_aim::YOLO yolo(argv[1], false);
  std::vector<auto_aim::NetDetector::TicketPtr> tickets;
  tickets.reserve(yolo.request_capacity());

  for (std::size_t i = 0; i < yolo.request_capacity(); ++i) {
    auto ticket = yolo.try_start_async(image);
    assert(ticket);
    tickets.push_back(std::move(ticket));
  }
  assert(!yolo.try_start_async(image));

  for (auto & ticket : tickets) {
    yolo.postprocess(ticket);
    ticket.reset();
  }

  assert(yolo.try_start_async(image));

  auto_aim::multithread::MultiThreadDetector detector(argv[1], false);
  const cv::Rect full(0, 0, image.cols, image.rows);
  const std::size_t count = detector.request_capacity() * 20 + 10;
  const auto capture_start = std::chrono::steady_clock::now();
  auto timestamp_for = [&](uint64_t index) {
    return capture_start + std::chrono::milliseconds(index);
  };
  auto pose_for = [](uint64_t index) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(0.01 * index, Eigen::Vector3d::UnitY()));
  };
  std::vector<auto_aim::multithread::Detection> delivered_detections;
  const auto error = detector.submit(
    image, timestamp_for(0), cv::Rect(-1, 0, image.cols, image.rows), std::nullopt, pose_for(0));
  assert(error.status == auto_aim::multithread::SubmitStatus::accepted);
  const auto error_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (detector.drop_counts().skipped_error == 0 &&
         std::chrono::steady_clock::now() < error_deadline) {
    if (auto detection = detector.wait_pop_for(std::chrono::milliseconds(10)))
      delivered_detections.push_back(std::move(*detection));
  }
  assert(detector.drop_counts().skipped_error >= 1);
  while (auto detection = detector.wait_pop_for(std::chrono::milliseconds(10)))
    delivered_detections.push_back(std::move(*detection));

  for (std::size_t i = 0; i < count; ++i) {
    const auto submitted = detector.submit(
      image, timestamp_for(i + 1), full, std::nullopt, pose_for(i + 1));
    assert(submitted.status == auto_aim::multithread::SubmitStatus::accepted);
    assert(submitted.sequence == 0);  // A sequence is assigned only after dispatch accepts a frame.
  }
  detector.close();
  const auto closed = detector.submit(image, std::chrono::steady_clock::now(), full);
  assert(closed.status == auto_aim::multithread::SubmitStatus::closed);
  assert(closed.sequence == 0);

  uint64_t expected_sequence = 1;
  bool saw_error_placeholder = false;
  while (auto detection = detector.wait_pop()) {
    delivered_detections.push_back(std::move(*detection));
  }
  detector.join();
  assert(!delivered_detections.empty());
  for (const auto & detection : delivered_detections) {
    assert(detection.sequence == expected_sequence++);
    // Results retain the selected frame's capture metadata; overwritten frames have no sequence.
    assert(detection.timestamp >= capture_start);
    assert(detection.timestamp <= timestamp_for(count + 1));
    assert(detection.q);
    assert(detection.capture_ms >= 0.0);
    assert(detection.submit_ms >= 0.0);
    assert(detection.inference_ms >= 0.0);
    assert(detection.delivery_ms >= 0.0);
    if (detection.timestamp == timestamp_for(0)) {
      assert(!detection.inferred);
      saw_error_placeholder = true;
    }
  }
  assert(expected_sequence > 1);
  assert(saw_error_placeholder);

  return 0;
}
