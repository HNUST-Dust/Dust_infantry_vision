#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <opencv2/opencv.hpp>
#include <vector>

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
  return 0;
}
