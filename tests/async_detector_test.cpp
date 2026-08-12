#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
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
  std::vector<auto_aim::multithread::SubmitResult> submissions;
  const std::size_t count = detector.request_capacity() + 2;
  for (std::size_t i = 0; i < count; ++i) {
    submissions.push_back(detector.submit(image, std::chrono::steady_clock::now(), full));
    assert(submissions.back().sequence == i + 1);
  }
  const auto error = detector.submit(
    image, std::chrono::steady_clock::now(), cv::Rect(-1, 0, image.cols, image.rows));
  assert(error.status == auto_aim::multithread::SubmitStatus::skipped_error);
  assert(error.sequence == count + 1);

  const auto empty = detector.submit({}, std::chrono::steady_clock::now(), full);
  assert(empty.status == auto_aim::multithread::SubmitStatus::skipped_empty);
  assert(empty.sequence == count + 2);

  detector.close();
  const auto closed = detector.submit(image, std::chrono::steady_clock::now(), full);
  assert(closed.status == auto_aim::multithread::SubmitStatus::closed);
  assert(closed.sequence == 0);

  uint64_t expected_sequence = 1;
  std::size_t delivered = 0;
  while (auto detection = detector.wait_pop()) {
    assert(detection->sequence == expected_sequence++);
    ++delivered;
  }
  detector.join();
  assert(delivered == count + 2);

  return 0;
}
