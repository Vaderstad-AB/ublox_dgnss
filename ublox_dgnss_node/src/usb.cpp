// Copyright 2021 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <chrono>
#include <thread>
#include <cstring>
#include <string>
#include <memory>
#include "ublox_dgnss_node/callback.hpp"
#include "ublox_dgnss_node/usb.hpp"

using namespace std::placeholders;

namespace usb
{
namespace
{
const char * transfer_status_txt(enum libusb_transfer_status status)
{
  switch (status) {
    case LIBUSB_TRANSFER_COMPLETED: return "completed";
    case LIBUSB_TRANSFER_ERROR: return "transfer failed";
    case LIBUSB_TRANSFER_TIMED_OUT: return "transfer timed out";
    case LIBUSB_TRANSFER_CANCELLED: return "transfer cancelled";
    case LIBUSB_TRANSFER_STALL: return "transfer stalled";
    case LIBUSB_TRANSFER_NO_DEVICE: return "device disconnected";
    case LIBUSB_TRANSFER_OVERFLOW: return "transfer overflow - device sent more than requested";
    default: return "unknown transfer status";
  }
}
}  // namespace

Connection::Connection(int vendor_id, int product_id, std::string serial_str, int log_level)
{
  vendor_id_ = vendor_id;
  product_id_ = product_id;
  serial_str_ = serial_str;
  class_id_ = LIBUSB_HOTPLUG_MATCH_ANY;
  log_level_ = log_level;
  ctx_ = NULL;
  devh_ = NULL;
  dev_ = NULL;
  hp_[0] = 0;
  hp_[1] = 0;
  ep_data_out_addr_ = 0;
  ep_data_in_addr_ = 0;
  ep_comms_in_addr_ = 0;
  timeout_ms_ = 45;
  timeout_tv_ = {1, 0};       // default the timeout to 1 seconds
  keep_running_ = true;
  attached_ = false;
  last_data_tp_ = std::chrono::steady_clock::now();
}

void Connection::set_log_callback(log_cb_fn cb_fn)
{
  log_cb_fn_ = cb_fn;
}

void Connection::set_log_level(int level)
{
  log_level_ = level;
  if (ctx_ == NULL) {
    return;
  }
    #if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, level);
    #else
  libusb_set_debug(ctx_, level);
    #endif
}

void Connection::record_error(LogSeverity severity, const std::string & msg)
{
  if (severity <= LOG_WARN) {
    const std::lock_guard<std::mutex> lock(health_mutex_);
    last_error_ = msg;
  }
  if (log_cb_fn_) {
    log_cb_fn_(severity, msg);
  } else if (severity <= LOG_WARN) {
    std::cerr << "[usb] " << msg << std::endl;
  }
}

void Connection::note_data_received(size_t len)
{
  bytes_in_ += len;
  const std::lock_guard<std::mutex> lock(health_mutex_);
  last_data_tp_ = std::chrono::steady_clock::now();
}

void Connection::clear_endpoint_halt(unsigned char endpoint)
{
  if (devh_ == nullptr) {
    return;
  }
  int rc = libusb_clear_halt(devh_, endpoint);
  if (rc < 0) {
    record_error(
      LOG_WARN, "clear halt failed on endpoint 0x" +
      std::to_string(static_cast<int>(endpoint)) + ": " + libusb_error_name(rc));
  } else {
    record_error(LOG_WARN, "cleared halt on stalled endpoint");
  }
}

health_t Connection::health()
{
  health_t h;
  h.attached = attached_.load();
  h.device_ready = device_ready();
  h.bytes_in = bytes_in_.load();
  h.bytes_out = bytes_out_.load();
  h.transfers_in_ok = transfers_in_ok_.load();
  h.transfers_out_ok = transfers_out_ok_.load();
  h.transfer_errors = transfer_errors_.load();
  h.stalls = stalls_.load();
  h.timeouts = timeouts_.load();
  h.disconnects = disconnects_.load();
  h.submit_failures = submit_failures_.load();
  h.reopen_attempts = reopen_attempts_.load();
  h.reopen_successes = reopen_successes_.load();
  h.device_resets = device_resets_.load();
  h.transfer_in_starved = transfer_in_starved_.load();
  h.queued_transfer_in = queued_transfer_in_num();
  {
    const std::lock_guard<std::mutex> lock(health_mutex_);
    h.last_error = last_error_;
    h.seconds_since_last_data =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_data_tp_).count();
  }
  return h;
}

void Connection::init()
{
  int rc = libusb_init(&ctx_);
  if (rc < 0) {
    throw std::string("Error initialising libusb: ") + libusb_error_name(rc);
  }

  set_log_level(log_level_);

    #if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000107)
  // forward libusb's own diagnostics so low level USB faults become visible
  usb_log_callback_t<void(libusb_context * ctx, enum libusb_log_level level,
    const char * str)>::func =
    [this](libusb_context * cb_ctx, enum libusb_log_level level, const char * str) {
      (void)cb_ctx;
      if (!log_cb_fn_ || str == nullptr) {
        return;
      }
      std::string msg(str);
      while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) {
        msg.pop_back();
      }
      LogSeverity severity;
      switch (level) {
        case LIBUSB_LOG_LEVEL_ERROR: severity = LOG_ERROR; break;
        case LIBUSB_LOG_LEVEL_WARNING: severity = LOG_WARN; break;
        case LIBUSB_LOG_LEVEL_INFO: severity = LOG_INFO; break;
        default: severity = LOG_DEBUG; break;
      }
      log_cb_fn_(severity, "libusb: " + msg);
    };
  libusb_set_log_cb(
    ctx_,
    static_cast<libusb_log_cb>(usb_log_callback_t<void(libusb_context * ctx,
    enum libusb_log_level level, const char * str)>::callback),
    LIBUSB_LOG_CB_CONTEXT);
    #endif

  if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    throw std::string("Hotplug capabilities are not supported on this platform!");
  }

  // setup C style to C++ style callback
  hotplug_attach_callback_t<int(libusb_context * ctx, libusb_device * device,
    libusb_hotplug_event event, void * user_data)>::func = std::bind<int>(
    &Connection::hotplug_attach_callback, this, _1, _2, _3, _4);
  libusb_hotplug_callback_fn hotplug_attach_callback_fn =
    static_cast<libusb_hotplug_callback_fn>(hotplug_attach_callback_t<int(libusb_context * ctx,
    libusb_device * device, libusb_hotplug_event event, void * user_data)>::callback);

  rc = libusb_hotplug_register_callback(
    ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
    product_id_, class_id_, hotplug_attach_callback_fn, NULL, &hp_[0]);
  if (LIBUSB_SUCCESS != rc) {
    throw std::string("Error registering hotplug attach callback");
  }

  // setup C style to C++ style callback
  hotplug_detach_callback_t<int(libusb_context * ctx, libusb_device * device,
    libusb_hotplug_event event, void * user_data)>::func = std::bind<int>(
    &Connection::hotplug_detach_callback, this, _1, _2, _3, _4);
  libusb_hotplug_callback_fn hotplug_detach_callback_fn =
    static_cast<libusb_hotplug_callback_fn>(hotplug_detach_callback_t<int(libusb_context * ctx,
    libusb_device * device, libusb_hotplug_event event, void * user_data)>::callback);

  rc = libusb_hotplug_register_callback(
    ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
    product_id_, class_id_, hotplug_detach_callback_fn, NULL, &hp_[1]);
  if (LIBUSB_SUCCESS != rc) {
    throw std::string("Error registering hotplug detach callback");
  }
}

// Helper to check if device is really ready
bool Connection::device_ready()
{
  return devh_ &&
         num_interfaces_ == 2 &&
         ep_data_out_addr_ != 0 && ep_data_out_addr_ != 0xaaaa &&
         ep_data_in_addr_ != 0 && ep_data_in_addr_ != 0xaaaa &&
         ep_comms_in_addr_ != 0 && ep_comms_in_addr_ != 0xaaaa &&
         device_speed() != LIBUSB_SPEED_UNKNOWN;
}

// Function to open a USB device with a specific Vendor ID, Product ID, and serial number string
libusb_device_handle * Connection::open_device_with_serial_string(
  libusb_context * ctx,
  int vendor_id, int product_id,
  std::string serial_str,
  char * serial_num_string,
  size_t serial_num_string_size)
{
  libusb_device_handle * devHandle = nullptr;
  libusb_device ** deviceList = nullptr;

  if (serial_num_string != nullptr && serial_num_string_size > 0) {
    serial_num_string[0] = '\0';
  }

  ssize_t rc = libusb_get_device_list(ctx, &deviceList);
  if (rc < 0) {
    throw std::string("Error getting device list: ") +
          libusb_error_name(static_cast<int>(rc));
  }
  ssize_t deviceCount = rc;

  // Iterate through the list to find the desired device
  for (ssize_t i = 0; i < deviceCount; i++) {
    libusb_device * device = deviceList[i];
    libusb_device_descriptor desc;

    if (libusb_get_device_descriptor(device, &desc) < 0) {
      continue;
    }

    if (desc.idVendor != vendor_id || desc.idProduct != product_id) {
      continue;
    }

    // a device we cannot open (permissions, already claimed) must not abort the whole scan
    int open_rc = libusb_open(device, &devHandle);
    if (open_rc < 0) {
      record_error(
        LOG_WARN, std::string("Error opening candidate device: ") + libusb_error_name(open_rc));
      devHandle = nullptr;
      continue;
    }

    int len = libusb_get_string_descriptor_ascii(
      devHandle, desc.iSerialNumber,
      reinterpret_cast<unsigned char *>(serial_num_string),
      static_cast<int>(serial_num_string_size));
    if (len < 0 && serial_num_string_size > 0) {
      serial_num_string[0] = '\0';
    }

    // an empty requested serial string means take the first device found
    if (serial_str.empty() || serial_str == serial_num_string) {
      break;
    }

    // Close the device if it didn't match
    libusb_close(devHandle);
    devHandle = nullptr;
  }
  // Free the device list
  libusb_free_device_list(deviceList, 1);

  return devHandle;
}


bool Connection::open_device()
{
  char serial_num_string[256];
  devh_ = open_device_with_serial_string(
    ctx_, vendor_id_, product_id_, serial_str_,
    serial_num_string, sizeof(serial_num_string));
  if (!devh_) {
    if (serial_str_.empty()) {
      throw std::string("Error finding USB device");
    } else {
      throw std::string("Error finding USB device with specified serial string, looking for \"") +
            serial_str_ + "\" however \"" + serial_num_string + "\" was found.";
    }
    return false;
  }

  // retrieve

  int rc = libusb_set_auto_detach_kernel_driver(devh_, true);
  if (rc < 0) {
    throw std::string("Error set auto detach kernel driver: ") + libusb_error_name(rc);
  }

  /* As we are dealing with a CDC-ACM device, it's highly probable that
   * Linux already attached the cdc-acm driver to this device.
   * We need to detach the drivers from all the USB interfaces. The CDC-ACM
   * Class defines two interfaces: the Control interface and the
   * Data interface.
   */
  for (int if_num = 0; if_num < 2; if_num++) {
    if (libusb_kernel_driver_active(devh_, if_num)) {
      int detach_rc = libusb_detach_kernel_driver(devh_, if_num);
      record_error(
        LOG_INFO, "detached kernel driver for interface " + std::to_string(if_num) + ": " +
        libusb_error_name(detach_rc));
    }
    rc = libusb_claim_interface(devh_, if_num);
    if (rc < 0) {
      record_error(
        LOG_WARN, "claim interface " + std::to_string(if_num) + " failed: " +
        libusb_error_name(rc));
      throw std::string("Error claiming interface: ") + libusb_error_name(rc);
    }
  }

  dev_ = libusb_get_device(devh_);

  /* get the device descriptor - newer libusb versions always succeed */
  struct libusb_device_descriptor dev_desc;
  rc = libusb_get_device_descriptor(dev_, &dev_desc);
  if (rc < 0) {
    throw std::string("Error getting device descriptor: ") + *libusb_error_name(rc);
  }
  auto num_configurations = dev_desc.bNumConfigurations;       // this should be 1
  if (num_configurations != 1) {
    throw std::string("Error bNumConfigurations is not 1 - dont know which configuration to use");
  }

  /* get the active USB configuration descriptor */
  struct libusb_config_descriptor * conf_desc;
  rc = libusb_get_active_config_descriptor(dev_, &conf_desc);
  if (rc < 0) {
    throw std::string("Error getting active configuration descriptor: ") + libusb_error_name(rc);
  }

  num_interfaces_ = conf_desc->bNumInterfaces;
  if (num_interfaces_ != 2) {
    throw std::string("Error config bNumInterfaces != 2");
  }

  for (uint8_t i = 0; i < num_interfaces_; i++) {
    auto interface = &conf_desc->interface[i];
    for (uint8_t j = 0; j < interface->num_altsetting; j++) {
      auto interface_desc = &interface->altsetting[j];

      switch (interface_desc->bInterfaceClass) {
        case LIBUSB_CLASS_COMM:
          // should only have one endpoint
          ep_comms_in_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          break;
        case LIBUSB_CLASS_DATA:
          ep_data_out_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          ep_data_in_addr_ = interface_desc->endpoint[1].bEndpointAddress;
          break;
        default:
          throw std::string("Error unknown bInterfaceClass");
      }
    }
  }
  libusb_free_config_descriptor(conf_desc);

  /* Start configuring the device:
   * - set line state
   */
  rc = libusb_control_transfer(
    devh_, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS,
    0, NULL, 0, 0);
  if (rc < 0 && rc != LIBUSB_ERROR_BUSY) {
    throw libusb_error_name(rc);
  }

  // Check device readiness after all setup
  if (!device_ready()) {
    close_devh();
    throw std::string("Device opened but not ready (bad descriptors or endpoints)");
  }

  err_count_ = 0;
  transfer_in_starved_ = false;
  note_data_received(0);

  return true;
}

char * Connection::device_speed_txt()
{
  char * speed_txt;
  switch (device_speed()) {
    case LIBUSB_SPEED_LOW:
      speed_txt = const_cast<char *>("SPEED_LOW (1.5 MBit/s)");
      break;
    case LIBUSB_SPEED_HIGH:
      speed_txt = const_cast<char *>("SPEED_HIGH (480 MBit/s)");
      break;
    case LIBUSB_SPEED_FULL:
      speed_txt = const_cast<char *>("SPEED_FULL (12 MBit/s)");
      break;
    case LIBUSB_SPEED_SUPER:
      speed_txt = const_cast<char *>("SPEED_SUPER (5000 MBit/s)");
      break;
    case LIBUSB_SPEED_SUPER_PLUS:
      speed_txt = const_cast<char *>("SPEED_SUPER_PLUS (10000 MBit/s)");
      break;
    default:
      speed_txt = const_cast<char *>("SPEED_UNKNOWN");
      break;
  }
  return speed_txt;
}

int Connection::hotplug_attach_callback(
  libusb_context * ctx, libusb_device * dev,
  libusb_hotplug_event event, void * user_data)
{
  (void)ctx;
  (void)dev;
  (void)event;
  (void)user_data;

  record_error(LOG_INFO, "hotplug attach event");

  // If device is marked attached but not actually ready, force re-open
  if (attached_ && !device_ready()) {
    record_error(LOG_WARN, "device marked attached but not ready, forcing re-open");
    close_devh();
    attached_ = false;
  }

  // if device already attached, don't attempt to open further devices
  if (attached_) {
    record_error(LOG_INFO, "device already attached, skipping open");
    return 0;
  }

  // Retry a few times - cdc_acm may still be claiming the device on hotplug
  for (int attempt = 0; attempt < 5; attempt++) {
    if (attempt > 0) {
      record_error(LOG_WARN, "open retry attempt " + std::to_string(attempt) + "/4");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    try {
      if (open_device()) {
        attached_ = true;
        record_error(LOG_INFO, "device successfully opened and attached");
        if (hp_attach_cb_fn_) {
          (hp_attach_cb_fn_)();
        }
        return 0;
      }
    } catch (const std::string & e) {
      record_error(LOG_WARN, "open_device() error: " + e);
    } catch (const std::exception & e) {
      record_error(LOG_WARN, std::string("open_device() exception: ") + e.what());
    } catch (...) {
      record_error(LOG_WARN, "open_device() unknown exception");
    }
  }
  record_error(LOG_ERROR, "giving up opening device after 5 attempts");
  return 0;
}

int Connection::hotplug_detach_callback(
  libusb_context * ctx, libusb_device * dev,
  libusb_hotplug_event event, void * user_data)
{
  (void)ctx;
  (void)dev;
  (void)event;
  (void)user_data;
  if (attached_) {
    disconnects_++;
    record_error(LOG_ERROR, "usb device detached");
    close_devh();
    attached_ = false;
    if (hp_detach_cb_fn_) {
      (hp_detach_cb_fn_)();
    }
  }
  return 0;
}


int Connection::read_chars(u_char * data, size_t size)
{
  if (devh_ == nullptr || !attached_) {
    throw UsbException("read_chars: device not attached");
  }

  /* To receive characters from the device initiate a bulk_transfer to the
   * Endpoint with address ep_in_addr.
   */
  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_in_addr_ | LIBUSB_ENDPOINT_IN, data, size, &actual_length,
    timeout_ms_);
  if (rc == LIBUSB_ERROR_TIMEOUT) {
    timeouts_++;
    throw TimeoutException();
  } else if (rc < 0) {
    transfer_errors_++;
    std::string exception_msg("Error while waiting for char: ");
    exception_msg.append(libusb_error_name(rc));
    record_error(LOG_WARN, exception_msg);
    throw UsbException(exception_msg);
  }

  if (actual_length > 0) {
    note_data_received(static_cast<size_t>(actual_length));
  }

  return actual_length;
}

void Connection::write_char(u_char c)
{
  if (devh_ == nullptr || !attached_) {
    throw UsbException("write_char: device not attached");
  }

  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT, &c, 1,
    &actual_length, 0);
  if (rc < 0) {
    std::string exception_msg("Error while sending char: ");
    exception_msg.append(libusb_error_name(rc));
    throw UsbException(exception_msg);
  }
}

void Connection::write_buffer(u_char * buf, size_t size)
{
  if (devh_ == nullptr || !attached_) {
    throw UsbException("write_buffer: device not attached");
  }

  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT, buf, size,
    &actual_length, 0);
  if (rc < 0) {
    transfer_errors_++;
    std::string exception_msg("Error while sending buf: ");
    exception_msg.append(libusb_error_name(rc));
    record_error(LOG_WARN, exception_msg);
    throw UsbException(exception_msg);
  }

  bytes_out_ += static_cast<uint64_t>(actual_length);

  if (actual_length != static_cast<int>(size)) {
    std::string exception_msg("Error didn't send full buf - size: ");
    exception_msg.append(std::to_string(size));
    exception_msg.append(" actual_length: ");
    exception_msg.append(std::to_string(actual_length));
    record_error(LOG_WARN, exception_msg);
    throw UsbException(exception_msg);
  }
}

void Connection::callback_out(struct libusb_transfer * transfer)
{
  // everything needed from the transfer must be read before it is freed
  bool * completed = reinterpret_cast<bool *>(transfer->user_data);
  const enum libusb_transfer_status status = transfer->status;
  const unsigned char endpoint = transfer->endpoint;
  const int actual_length = transfer->actual_length;

  if (status == LIBUSB_TRANSFER_COMPLETED) {
    transfers_out_ok_++;
    bytes_out_ += static_cast<uint64_t>(actual_length);
    if (out_cb_fn_) {
      (out_cb_fn_)(transfer);
    }
  } else {
    transfer_errors_++;
    switch (status) {
      case LIBUSB_TRANSFER_STALL:
        stalls_++;
        clear_endpoint_halt(endpoint);
        break;
      case LIBUSB_TRANSFER_TIMED_OUT:
        timeouts_++;
        break;
      case LIBUSB_TRANSFER_NO_DEVICE:
        disconnects_++;
        attached_ = false;
        break;
      default:
        break;
    }
    std::string msg = std::string("out transfer: ") + transfer_status_txt(status);
    record_error(LOG_WARN, msg);
    if (exception_cb_fn_) {
      (exception_cb_fn_)(UsbException(msg), completed);
    }
  }

  libusb_free_transfer(transfer);
  if (completed != nullptr) {
    *completed = true;
  }

  // the IN pipeline must never be left without an outstanding transfer
  ensure_transfer_in_queued();
}

void Connection::callback_in(struct libusb_transfer * transfer)
{
  // everything needed from the transfer must be read before it is freed
  bool * completed = reinterpret_cast<bool *>(transfer->user_data);
  const enum libusb_transfer_status status = transfer->status;
  const unsigned char endpoint = transfer->endpoint;
  const int actual_length = transfer->actual_length;

  if (status == LIBUSB_TRANSFER_COMPLETED) {
    transfers_in_ok_++;
    if (actual_length > 0) {
      note_data_received(static_cast<size_t>(actual_length));
    }
    if (in_cb_fn_) {
      (in_cb_fn_)(transfer);
    }
    err_count_ = 0;
  } else {
    transfer_errors_++;
    switch (status) {
      case LIBUSB_TRANSFER_STALL:
        stalls_++;
        clear_endpoint_halt(endpoint);
        break;
      case LIBUSB_TRANSFER_TIMED_OUT:
        timeouts_++;
        break;
      case LIBUSB_TRANSFER_NO_DEVICE:
        disconnects_++;
        attached_ = false;
        break;
      default:
        break;
    }
    std::string msg = std::string("in transfer: ") + transfer_status_txt(status);
    // rate limit so a permanently broken link cannot flood the log
    if (++err_count_ < 10) {
      record_error(LOG_WARN, msg);
      if (exception_cb_fn_) {
        (exception_cb_fn_)(UsbException(msg), completed);
      }
    } else if (err_count_ == 10) {
      record_error(LOG_ERROR, "in transfer failing repeatedly, suppressing further messages");
    }
  }

  libusb_free_transfer(transfer);
  if (completed != nullptr) {
    *completed = true;
  }

  // the IN pipeline must never be left without an outstanding transfer
  ensure_transfer_in_queued();
}

void Connection::write_buffer_async(u_char * buf, size_t size, void * user_data)
{
  (void)user_data;
  if (out_cb_fn_ == nullptr) {
    throw UsbException("No out callback function set");
  }
  if (exception_cb_fn_ == nullptr) {
    throw UsbException("No exception callback function set");
  }
  if (devh_ == nullptr || !attached_) {
    throw UsbException("write_buffer_async: device not attached");
  }

  auto transfer_out = make_transer_out(buf, size);
  submit_transfer(transfer_out, "async submit transfer out: ");
}

std::shared_ptr<transfer_t> Connection::make_transer_out(u_char * buf, size_t size)
{
  auto transfer_out = libusb_alloc_transfer(0);

  auto transfer = std::make_shared<transfer_t>();
  transfer->transfer = transfer_out;
  transfer->type = USB_OUT;
  transfer->buffer->resize(size);
  std::memcpy(transfer->buffer->data(), buf, size);
  transfer->completed = false;

  void * user_data = &transfer->completed;

  // setup C style to C++ style callback
  callback_out_t<void(struct libusb_transfer * transfer_out)>::func = std::bind(
    &Connection::callback_out, this, std::placeholders::_1);
  libusb_transfer_cb_fn callback_out_fn =
    static_cast<libusb_transfer_cb_fn>(callback_out_t<void(struct libusb_transfer * transfer_out)>::
    callback);

  transfer_out->flags = LIBUSB_TRANSFER_SHORT_NOT_OK;
  libusb_fill_bulk_transfer(
    transfer_out, devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT,
    // buf, size,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_out_fn, user_data, 0
  );

  return transfer;
}

std::shared_ptr<transfer_t> Connection::make_transfer_in()
{
  auto transfer_in = libusb_alloc_transfer(0);

  auto transfer = std::make_shared<transfer_t>();
  transfer->transfer = transfer_in;
  transfer->type = USB_IN;
  transfer->buffer->resize(IN_BUFFER_SIZE);
  transfer->completed = false;

  void * user_data = &transfer->completed;

  // setup C style to C++ style callback
  callback_in_t<void(struct libusb_transfer * transfer_in)>::func = std::bind(
    &Connection::callback_in, this, std::placeholders::_1);
  libusb_transfer_cb_fn callback_in_fn =
    static_cast<libusb_transfer_cb_fn>(callback_in_t<void(struct libusb_transfer * transfer_in)>::
    callback);

  // setup asynchronous transfer in to host from usb
  libusb_fill_bulk_transfer(
    transfer_in, devh_, ep_data_in_addr_ | LIBUSB_ENDPOINT_IN,
    // in_buffer_, IN_BUFFER_SIZE,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_in_fn, user_data, 0);             // no timeout

  return transfer;
}

void Connection::submit_transfer(
  std::shared_ptr<transfer_t> transfer, const std::string msg,
  bool wait_for_completed)
{
  (void)wait_for_completed;

  if (transfer == nullptr || transfer->transfer == nullptr) {
    throw UsbException("transfer->transfer is null");
  }

  // when we decline to submit, the libusb transfer must still be released
  if (!keep_running_ || !attached_) {
    libusb_free_transfer(transfer->transfer);
    transfer->transfer = nullptr;
    transfer->completed = true;
    return;
  }

  int rc = libusb_submit_transfer(transfer->transfer);
  if (rc < 0) {
    submit_failures_++;
    libusb_free_transfer(transfer->transfer);
    transfer->transfer = nullptr;
    transfer->completed = true;
    if (transfer->type == USB_IN) {
      transfer_in_starved_ = true;
    }
    std::string exception_msg = msg;
    exception_msg.append(libusb_error_name(rc));
    record_error(LOG_ERROR, exception_msg);
    throw UsbException(exception_msg);
  }

  const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);
  // only adding those that were succesfully submitted to the queue
  transfer_queue_.push_back(transfer);

  // remove completed from the queue
  cleanup_transfer_queue_unlocked();
}

void Connection::cleanup_transfer_queue_unlocked()
{
  // erase invalidates deque iterators, so the returned iterator must be used
  auto it = transfer_queue_.begin();
  while (it != transfer_queue_.end()) {
    if ((*it)->completed) {
      it = transfer_queue_.erase(it);
    } else {
      ++it;
    }
  }
}

size_t Connection::queued_transfer_in_num()
{
  const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);

  size_t num = 0;
  for (auto it = transfer_queue_.begin(); it != transfer_queue_.end(); ++it) {
    auto t = it->get();
    if (!t->completed && t->type == USB_IN) {
      ++num;
    }
  }
  return num;
}

bool Connection::ensure_transfer_in_queued()
{
  if (!keep_running_ || !attached_ || devh_ == nullptr) {
    return false;
  }

  if (queued_transfer_in_num() > 0) {
    transfer_in_starved_ = false;
    return true;
  }

  try {
    auto transfer_in = make_transfer_in();
    submit_transfer(transfer_in, "re-arm transfer in: ");
  } catch (const UsbException & e) {
    transfer_in_starved_ = true;
    record_error(LOG_ERROR, std::string("could not re-arm IN transfer: ") + e.what());
    return false;
  } catch (...) {
    transfer_in_starved_ = true;
    record_error(LOG_ERROR, "could not re-arm IN transfer: unknown error");
    return false;
  }

  bool armed = queued_transfer_in_num() > 0;
  transfer_in_starved_ = !armed;
  return armed;
}

bool Connection::reopen_device()
{
  reopen_attempts_++;
  record_error(LOG_WARN, "re-opening usb device");

  close_devh();
  attached_ = false;
  {
    const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);
    transfer_queue_.clear();
  }

  try {
    if (open_device()) {
      attached_ = true;
      reopen_successes_++;
      record_error(LOG_WARN, "usb device re-opened");
      return true;
    }
  } catch (const std::string & e) {
    record_error(LOG_ERROR, "re-open failed: " + e);
  } catch (const std::exception & e) {
    record_error(LOG_ERROR, std::string("re-open failed: ") + e.what());
  } catch (...) {
    record_error(LOG_ERROR, "re-open failed: unknown error");
  }
  return false;
}

bool Connection::reset_device()
{
  if (devh_ != nullptr) {
    device_resets_++;
    record_error(LOG_WARN, "issuing usb port reset");
    int rc = libusb_reset_device(devh_);
    if (rc < 0) {
      record_error(LOG_ERROR, std::string("usb reset failed: ") + libusb_error_name(rc));
    }
  }
  // the handle cannot be trusted after a reset, always re-open
  return reopen_device();
}

void Connection::init_async()
{
  try {
    if (!devh_ || !attached_ ||
        ep_data_in_addr_ == 0 || ep_data_in_addr_ == 0xaaaa ||
        ep_data_out_addr_ == 0 || ep_data_out_addr_ == 0xaaaa) {
      throw UsbException("USB device not ready in init_async");
    }
    if (in_cb_fn_ == nullptr) {
      throw UsbException("No in callback function set");
    }
    if (out_cb_fn_ == nullptr) {
      throw UsbException("No out callback function set");
    }
    if (exception_cb_fn_ == nullptr) {
      throw UsbException("No exception callback function set");
    }
  } catch (const UsbException & e) {
    record_error(LOG_ERROR, std::string("init_async: ") + e.what());
    throw;  // rethrow to propagate error if needed
  }

  note_data_received(0);
  err_count_ = 0;

  // submit initial transfer in request
  // - at first get a few zero length records
  auto transfer_in = make_transfer_in();
  submit_transfer(transfer_in, "init_async transfer: ", false);
}

void Connection::handle_usb_events()
{
  if (!keep_running_) {return;}

  int rc = libusb_handle_events_timeout(ctx_, &timeout_tv_);
  switch (rc) {
    case LIBUSB_ERROR_INTERRUPTED:
      // expected while shutting down, not a fault
      keep_running_ = false;
      return;
    case LIBUSB_ERROR_NO_DEVICE: {
        if (attached_) {
          disconnects_++;
        }
        attached_ = false;
      }
      break;
    default:
      break;
  }
  if (rc < 0) {
    record_error(LOG_ERROR, std::string("handle_usb_events: ") + libusb_error_name(rc));
    throw UsbException(libusb_error_name(rc));
  }
}

void Connection::close_devh()
{
  if (devh_) {
    for (int if_num = 0; if_num < 2; if_num++) {
      int rc = libusb_release_interface(devh_, if_num);
      if (rc >= 0) {
        libusb_attach_kernel_driver(devh_, if_num);
      }
    }
    libusb_close(devh_);             // hangs if the device has been detached already
    devh_ = nullptr;
    dev_ = nullptr;
    attached_ = false;
  }
}

void Connection::shutdown()
{
  keep_running_ = false;

    #if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000107)
  // the log callback holds a reference to this object, detach it before teardown
  if (ctx_ != NULL) {
    libusb_set_log_cb(ctx_, NULL, LIBUSB_LOG_CB_CONTEXT);
  }
    #endif
  log_cb_fn_ = nullptr;

  // de register hotplug callbacks
  if (hp_[0]) {
    libusb_hotplug_deregister_callback(ctx_, hp_[0]);
    hp_[0] = 0;
  }
  if (hp_[1]) {
    libusb_hotplug_deregister_callback(ctx_, hp_[1]);
    hp_[1] = 0;
  }

  close_devh();
}

Connection::~Connection()
{
  shutdown();

  libusb_exit(ctx_);
}
}  // namespace usb
