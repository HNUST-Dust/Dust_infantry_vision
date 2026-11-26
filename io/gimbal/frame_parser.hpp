#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "tools/protocol/crc.hpp"

namespace io
{
// Keep partial packets across reads and slide one byte after a corrupt candidate.
template <typename Frame> class GimbalFrameParser
{
public:
  bool push(uint8_t byte, Frame & frame, bool skip_crc = false)
  {
    bytes_[size_++] = byte;
    while (size_ && (bytes_[0] != 'S' || (size_ >= 2 && bytes_[1] != 'P'))) {
      discard_first();
    }
    if (size_ != bytes_.size())
      return false;
    if (!skip_crc && !tools::check_crc16(bytes_.data(), bytes_.size())) {
      discard_first();
      return false;
    }
    std::memcpy(&frame, bytes_.data(), bytes_.size());
    size_ = 0;
    return true;
  }

private:
  void discard_first()
  {
    --size_;
    std::memmove(bytes_.data(), bytes_.data() + 1, size_);
  }
  std::array<uint8_t, sizeof(Frame)> bytes_{};
  size_t size_ = 0;
};
}  // namespace io
