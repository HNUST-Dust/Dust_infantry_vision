#include "usb_interrupt_transport.hpp"

#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "tools/logger.hpp"

namespace io
{
UsbInterruptTransport::UsbInterruptTransport(
  uint16_t vid, uint16_t pid, int interface_number, uint8_t ep_in, uint8_t ep_out,
  unsigned int timeout_ms)
: vid_(vid),
  pid_(pid),
  interface_number_(interface_number),
  ep_in_(ep_in),
  ep_out_(ep_out),
  timeout_ms_(timeout_ms)
{
}

UsbInterruptTransport::~UsbInterruptTransport()
{
  close();
}

void UsbInterruptTransport::open()
{
  std::scoped_lock lock(read_mutex_, write_mutex_);
  if (handle_ != nullptr) return;

  int rc = libusb_init(&context_);
  if (rc != LIBUSB_SUCCESS) {
    context_ = nullptr;
    throw std::runtime_error("libusb_init: " + error_string(rc));
  }

  handle_ = libusb_open_device_with_vid_pid(context_, vid_, pid_);
  if (handle_ == nullptr) {
    libusb_exit(context_);
    context_ = nullptr;
    throw std::runtime_error("USB device not found or inaccessible");
  }

  libusb_set_auto_detach_kernel_driver(handle_, 1);
  int current_configuration = 0;
  rc = libusb_get_configuration(handle_, &current_configuration);
  if (rc == LIBUSB_SUCCESS && current_configuration != 1) {
    rc = libusb_set_configuration(handle_, 1);
  }
  if (rc != LIBUSB_SUCCESS) {
    libusb_close(handle_);
    libusb_exit(context_);
    handle_ = nullptr;
    context_ = nullptr;
    throw std::runtime_error("libusb configuration: " + error_string(rc));
  }

  rc = libusb_claim_interface(handle_, interface_number_);
  if (rc != LIBUSB_SUCCESS) {
    libusb_close(handle_);
    libusb_exit(context_);
    handle_ = nullptr;
    context_ = nullptr;
    throw std::runtime_error("libusb_claim_interface: " + error_string(rc));
  }
  claimed_ = true;

  async_out_transfer_ = libusb_alloc_transfer(0);
  if (async_out_transfer_ == nullptr) {
    libusb_release_interface(handle_, interface_number_);
    libusb_close(handle_);
    libusb_exit(context_);
    claimed_ = false;
    handle_ = nullptr;
    context_ = nullptr;
    throw std::runtime_error("libusb_alloc_transfer failed");
  }
  {
    std::lock_guard<std::mutex> async_lock(async_out_mutex_);
    async_out_inflight_ = false;
    pending_out_valid_ = false;
    pending_out_size_ = 0;
  }
  event_quit_ = false;
  event_thread_ = std::thread(&UsbInterruptTransport::event_loop, this);
}

void UsbInterruptTransport::close()
{
  std::scoped_lock lock(read_mutex_, write_mutex_);
  if (async_out_transfer_ != nullptr) {
    {
      std::unique_lock<std::mutex> async_lock(async_out_mutex_);
      async_out_cv_.wait_for(async_lock, std::chrono::milliseconds(timeout_ms_ + 5), [this] {
        return !async_out_inflight_ && !pending_out_valid_;
      });
      event_quit_ = true;
      pending_out_valid_ = false;
      if (async_out_inflight_) libusb_cancel_transfer(async_out_transfer_);
    }
    if (event_thread_.joinable()) event_thread_.join();
    libusb_free_transfer(async_out_transfer_);
    async_out_transfer_ = nullptr;
  }
  if (handle_ != nullptr) {
    if (claimed_) libusb_release_interface(handle_, interface_number_);
    libusb_close(handle_);
  }
  if (context_ != nullptr) libusb_exit(context_);
  claimed_ = false;
  handle_ = nullptr;
  context_ = nullptr;
}

int UsbInterruptTransport::read(uint8_t * data, std::size_t size, int & transferred)
{
  if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return LIBUSB_ERROR_INVALID_PARAM;
  }
  std::lock_guard<std::mutex> lock(read_mutex_);
  transferred = 0;
  if (handle_ == nullptr) return LIBUSB_ERROR_NO_DEVICE;
  return libusb_interrupt_transfer(
    handle_, ep_in_, data, static_cast<int>(size), &transferred, timeout_ms_);
}

int UsbInterruptTransport::write(const uint8_t * data, std::size_t size, int & transferred)
{
  if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return LIBUSB_ERROR_INVALID_PARAM;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  transferred = 0;
  if (handle_ == nullptr) return LIBUSB_ERROR_NO_DEVICE;
  return libusb_interrupt_transfer(
    handle_, ep_out_, const_cast<unsigned char *>(data), static_cast<int>(size), &transferred,
    timeout_ms_);
}

int UsbInterruptTransport::write_async(const uint8_t * data, std::size_t size)
{
  if (data == nullptr || size == 0 || size > async_out_buffer_.size()) {
    return LIBUSB_ERROR_INVALID_PARAM;
  }
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  std::lock_guard<std::mutex> async_lock(async_out_mutex_);
  if (handle_ == nullptr || async_out_transfer_ == nullptr || event_quit_) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  if (async_out_inflight_) {
    std::memcpy(pending_out_buffer_.data(), data, size);
    pending_out_size_ = size;
    pending_out_valid_ = true;
    return LIBUSB_SUCCESS;
  }

  return submit_async_out_locked(data, size);
}

void LIBUSB_CALL UsbInterruptTransport::async_out_callback(libusb_transfer * transfer)
{
  static_cast<UsbInterruptTransport *>(transfer->user_data)
    ->handle_async_out_completion(transfer);
}

void UsbInterruptTransport::handle_async_out_completion(libusb_transfer * transfer)
{
  std::lock_guard<std::mutex> lock(async_out_mutex_);
  async_out_inflight_ = false;

  const bool completed = transfer->status == LIBUSB_TRANSFER_COMPLETED &&
                         transfer->actual_length == transfer->length;
  if (!completed && transfer->status != LIBUSB_TRANSFER_CANCELLED) {
    tools::logger()->warn(
      "[USB Interrupt] Async OUT failed: status={}, transferred={}/{}", transfer->status,
      transfer->actual_length, transfer->length);
  }

  if (!event_quit_ && pending_out_valid_) {
    const auto size = pending_out_size_;
    pending_out_valid_ = false;
    submit_async_out_locked(pending_out_buffer_.data(), size);
  }
  async_out_cv_.notify_all();
}

int UsbInterruptTransport::submit_async_out_locked(const uint8_t * data, std::size_t size)
{
  std::memcpy(async_out_buffer_.data(), data, size);
  libusb_fill_interrupt_transfer(
    async_out_transfer_, handle_, ep_out_, async_out_buffer_.data(), static_cast<int>(size),
    async_out_callback, this, timeout_ms_);
  const int rc = libusb_submit_transfer(async_out_transfer_);
  if (rc == LIBUSB_SUCCESS) {
    async_out_inflight_ = true;
  } else {
    tools::logger()->warn(
      "[USB Interrupt] Async OUT submit failed: {}", error_string(rc));
  }
  return rc;
}

void UsbInterruptTransport::event_loop()
{
  while (true) {
    {
      std::lock_guard<std::mutex> lock(async_out_mutex_);
      if (event_quit_ && !async_out_inflight_) break;
    }
    timeval timeout{0, 5000};
    const int rc = libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
    if (rc == LIBUSB_SUCCESS || rc == LIBUSB_ERROR_INTERRUPTED) continue;

    std::lock_guard<std::mutex> lock(async_out_mutex_);
    tools::logger()->warn("[USB Interrupt] Event handling failed: {}", error_string(rc));
    if (rc == LIBUSB_ERROR_NO_DEVICE) {
      async_out_inflight_ = false;
      pending_out_valid_ = false;
      async_out_cv_.notify_all();
      break;
    }
  }
}

std::string UsbInterruptTransport::error_string(int error_code)
{
  return std::string(libusb_error_name(error_code)) + ": " + libusb_strerror(error_code);
}
}  // namespace io
