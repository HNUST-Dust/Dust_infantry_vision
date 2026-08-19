#ifndef IO__USB_INTERRUPT_TRANSPORT_HPP
#define IO__USB_INTERRUPT_TRANSPORT_HPP

#include <libusb-1.0/libusb.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace io
{
class UsbInterruptTransport
{
public:
  UsbInterruptTransport(
    uint16_t vid, uint16_t pid, int interface_number, uint8_t ep_in, uint8_t ep_out,
    unsigned int timeout_ms = 20);
  ~UsbInterruptTransport();

  UsbInterruptTransport(const UsbInterruptTransport &) = delete;
  UsbInterruptTransport & operator=(const UsbInterruptTransport &) = delete;

  void open();
  void close();
  int read(uint8_t * data, std::size_t size, int & transferred);
  int write(const uint8_t * data, std::size_t size, int & transferred);
  int write_async(const uint8_t * data, std::size_t size);

  static std::string error_string(int error_code);

private:
  uint16_t vid_;
  uint16_t pid_;
  int interface_number_;
  uint8_t ep_in_;
  uint8_t ep_out_;
  unsigned int timeout_ms_;

  libusb_context * context_ = nullptr;
  libusb_device_handle * handle_ = nullptr;
  bool claimed_ = false;
  std::mutex read_mutex_;
  std::mutex write_mutex_;

  libusb_transfer * async_out_transfer_ = nullptr;
  std::array<uint8_t, 64> async_out_buffer_{};
  std::array<uint8_t, 64> pending_out_buffer_{};
  std::size_t pending_out_size_ = 0;
  bool async_out_inflight_ = false;
  bool pending_out_valid_ = false;
  std::atomic<bool> event_quit_ = false;
  std::thread event_thread_;
  mutable std::mutex async_out_mutex_;
  std::condition_variable async_out_cv_;
  static void LIBUSB_CALL async_out_callback(libusb_transfer * transfer);
  void handle_async_out_completion(libusb_transfer * transfer);
  int submit_async_out_locked(const uint8_t * data, std::size_t size);
  void event_loop();
};
}  // namespace io

#endif  // IO__USB_INTERRUPT_TRANSPORT_HPP
