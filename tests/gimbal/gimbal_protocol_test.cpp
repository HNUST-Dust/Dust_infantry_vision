#include "io/gimbal/gimbal.hpp"
#include "tools/protocol/crc.hpp"

#include <array>
#include <cassert>
#include <cstring>

int main()
{
  // Fixed little-endian MCU fixture, independent of the host struct layout.
  std::array<uint8_t, 44> bytes{'S', 'P', 1, 0,   0,   128, 63, 0,   0,  0,  0,   0,  0, 0, 0,
                                0,   0,   0, 0,   0,   0,   0,  63,  0,  0,  128, 62, 0, 0, 0,
                                191, 0,   0, 128, 190, 0,   0,  184, 65, 44, 1,   2,  0, 0};
  const auto crc = tools::get_crc16(bytes.data(), 42);
  bytes[42] = static_cast<uint8_t>(crc);
  bytes[43] = static_cast<uint8_t>(crc >> 8);
  assert(tools::check_crc16(bytes.data(), bytes.size()));
  io::GimbalToVision packet{};
  std::memcpy(&packet, bytes.data(), bytes.size());
  assert(packet.mode == 1 && packet.q[0] == 1.0F);
  assert(packet.yaw == 0.5F && packet.yaw_vel == 0.25F);
  assert(packet.pitch == -0.5F && packet.pitch_vel == -0.25F);
  assert(packet.bullet_speed == 23.0F && packet.bullet_count == 300);
  assert(packet.enemy_color == 2 && packet.crc16 == crc);
  bytes[41] = 1;
  assert(!tools::check_crc16(bytes.data(), bytes.size()));
}
