#include "io/gimbal/frame_parser.hpp"
#include <cassert>
#include <vector>

struct Frame
{
  uint8_t bytes[44];
};

int main()
{
  Frame frame{};
  frame.bytes[0] = 'S';
  frame.bytes[1] = 'P';
  for (size_t i = 2; i < 42; ++i)
    frame.bytes[i] = static_cast<uint8_t>(i);
  auto crc = tools::get_crc16(frame.bytes, 42);
  frame.bytes[42] = crc & 255;
  frame.bytes[43] = crc >> 8;
  for (size_t offset = 0; offset < 44; ++offset) {
    std::vector<uint8_t> stream(frame.bytes + offset, frame.bytes + 44);
    stream.insert(stream.end(), frame.bytes, frame.bytes + 44);
    stream.insert(stream.end(), frame.bytes, frame.bytes + 44);
    io::GimbalFrameParser<Frame> parser;
    Frame out{};
    size_t count = 0;
    for (auto byte : stream)
      if (parser.push(byte, out)) {
        assert(std::memcmp(out.bytes, frame.bytes, 44) == 0);
        ++count;
      }
    assert(count == (offset == 0 ? 3 : 2));
  }
  // False header, corrupt frame, lost byte and extra byte must not hide later packets.
  for (int fault = 0; fault < 4; ++fault) {
    std::vector<uint8_t> stream(frame.bytes, frame.bytes + 44);
    if (fault == 0)
      stream.insert(stream.begin(), {'S', 'P', 0});
    if (fault == 1)
      stream[10] ^= 1;
    if (fault == 2)
      stream.erase(stream.begin() + 10);
    if (fault == 3)
      stream.insert(stream.begin() + 10, 99);
    stream.insert(stream.end(), frame.bytes, frame.bytes + 44);
    io::GimbalFrameParser<Frame> parser;
    Frame out{};
    size_t count = 0;
    for (auto byte : stream)
      if (parser.push(byte, out))
        ++count;
    assert(count >= 1);
    assert(std::memcmp(out.bytes, frame.bytes, 44) == 0);
  }
}
