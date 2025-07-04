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

#include <unistd.h>
#include <deque>
#include <mutex>
#include <string>
#include <chrono>
#include <ctime>
#include <memory>
#include <vector>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ublox_dgnss_node/visibility_control.h"
#include "ublox_dgnss_node/usb.hpp"
#include "ublox_dgnss_node/ubx/ubx_cfg.hpp"
#include "ublox_dgnss_node/ubx/ubx_mon.hpp"
#include "ublox_dgnss_node/ubx/ubx_inf.hpp"
#include "ublox_dgnss_node/ubx/ubx_ack.hpp"
#include "ublox_dgnss_node/ubx/ubx_nav.hpp"
#include "ublox_dgnss_node/ubx/ubx_rxm.hpp"
#include "ublox_dgnss_node/ubx/ubx_esf.hpp"
#include "ublox_dgnss_node/ubx/ubx_sec.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_clock.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_dop.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_eoe.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_ecef.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_odo.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_orb.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_sat.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_sig.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pos_ecef.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_svin.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_time_utc.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_vel_ecef.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_vel_ned.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_cor.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_rtcm.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_measx.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_rawx.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_spartn.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_spartn_key.hpp"
#include "ublox_ubx_msgs/msg/ubx_esf_meas.hpp"
#include "ublox_ubx_msgs/msg/ubx_esf_status.hpp"
#include "ublox_ubx_msgs/msg/ubx_mon_comms.hpp"
#include "ublox_ubx_msgs/msg/ubx_sec_sig.hpp"
#include "ublox_ubx_msgs/msg/ubx_sec_sig_log.hpp"
#include "ublox_ubx_msgs/msg/ubx_sec_uniqid.hpp"
#include "ublox_ubx_interfaces/srv/hot_start.hpp"
#include "ublox_ubx_interfaces/srv/warm_start.hpp"
#include "ublox_ubx_interfaces/srv/cold_start.hpp"
#include "ublox_ubx_interfaces/srv/reset_odo.hpp"

#include "rtcm_msgs/msg/message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ublox_dgnss
{

enum FrameType
{
  frame_in,       // in from the gps device
  frame_out       // out to the gps device
};
struct ubx_queue_frame_t
{
  rclcpp::Time ts;
  std::shared_ptr<ubx::Frame> ubx_frame;
  FrameType frame_type;
};
struct rtcm_queue_frame_t
{
  rclcpp::Time ts;
  std::vector<uint8_t> buf;
  FrameType frame_type;
};

enum ParamStatus
{
  PARAM_INITIAL,       // default value parameter used
                       // ie not set by user (and sent to GPS) or retrieved from gps
  PARAM_USER,       // value set by user either overidden at startup or param set
  PARAM_LOADED,       // loaded from gps device - not all items have a value set
                      // or default value on the gps
  PARAM_VALSET,       // value sent to gps device - might get rejected there
  PARAM_VALGET,       // attempt to retrieve value from gps device
  PARAM_ACKNAK       // todo in a future version - poll for value or valset might not work
};
struct param_state_t
{
  rclcpp::ParameterValue value;
  ParamStatus status;
};

class UbloxDGNSSNode : public rclcpp::Node
{
public:
  UBLOX_DGNSS_NODE_PUBLIC
  explicit UbloxDGNSSNode(const rclcpp::NodeOptions & options)
  : Node("ublox_dgnss", rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    callback_group_rtcm_timer_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_ubx_timer_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_usb_events_timer_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // this flag is used to control if certain parameters can be updated
    is_initialising_ = true;

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          get_logger(), "Interrupted while waiting for parameter client service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_WARN(get_logger(), "parameter client service not available, waiting again...");
    }

    // check for device serial string parameter
    check_for_device_serial_param(parameters_client);
    // check for frame_id parameter
    check_for_frame_id_param(parameters_client);

    // check that the CFG parameters are valid that have been supplied as args/yaml
    std::vector<std::string> prefixes;
    auto list_param_result = list_parameters(prefixes, 1);
    for (auto name : list_param_result.names) {
      // check for specified serial number string, silently skip over it - already handled above
      if (strcmp(name.c_str(), DEV_STRING_PARAM_NAME.c_str()) == 0) {
        continue;
      }
      // check for specified frame_id string, silently skip over it - already handled above
      if (strcmp(name.c_str(), FRAME_ID_PARAM_NAME.c_str()) == 0) {
        continue;
      }
      // ignore other parameters that don't start with "CFG"
      if (strncmp(name.c_str(), "CFG", 3) != 0) {
        continue;
      }
      RCLCPP_INFO(get_logger(), "parameter supplied: %s", name.c_str());
      bool valid = false;
      for (const auto & kv : ubx::cfg::ubxKeyCfgItemMap) {
        auto ubx_cfg_item = kv.second;
        if (strcmp(ubx_cfg_item.ubx_config_item, name.c_str()) == 0) {
          valid = true;
          break;
        }
      }
      if (!valid) {
        RCLCPP_WARN(
          get_logger(), "parameter supplied: %s is not recognised. Ignoring!",
          name.c_str());
      }
    }

    auto qos = rclcpp::SensorDataQoS();
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    ubx_nav_clock_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavClock>(
      "ubx_nav_clock", qos, pub_options);
    ubx_nav_cov_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavCov>(
      "ubx_nav_cov", qos, pub_options);
    ubx_nav_dop_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavDOP>(
      "ubx_nav_dop", qos, pub_options);
    ubx_nav_eoe_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavEOE>(
      "ubx_nav_eoe", qos, pub_options);
    ubx_nav_hp_pos_ecef_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavHPPosECEF>(
      "ubx_nav_hp_pos_ecef", qos, pub_options);
    ubx_nav_hp_pos_llh_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", qos, pub_options);
    ubx_nav_odo_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavOdo>(
      "ubx_nav_odo", qos, pub_options);
    ubx_nav_orb_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavOrb>(
      "ubx_nav_orb", qos, pub_options);
    ubx_nav_sat_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavSat>(
      "ubx_nav_sat", qos, pub_options);
    ubx_nav_sig_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavSig>(
      "ubx_nav_sig", qos, pub_options);
    ubx_nav_pos_ecef_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavPosECEF>(
      "ubx_nav_pos_ecef", qos, pub_options);
    ubx_nav_pos_llh_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavPosLLH>(
      "ubx_nav_pos_llh", qos, pub_options);
    ubx_nav_pvt_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavPVT>(
      "ubx_nav_pvt", qos, pub_options);
    ubx_nav_rel_pos_ned_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>(
      "ubx_nav_rel_pos_ned", qos, pub_options);
    ubx_nav_status_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavStatus>(
      "ubx_nav_status", qos, pub_options);
    ubx_nav_svin_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavSvin>(
      "ubx_nav_svin", qos, pub_options);
    ubx_nav_time_utc_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavTimeUTC>(
      "ubx_nav_time_utc", qos, pub_options);
    ubx_nav_vel_ecef_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavVelECEF>(
      "ubx_nav_vel_ecef", qos, pub_options);
    ubx_nav_vel_ned_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXNavVelNED>(
      "ubx_nav_vel_ned", qos, pub_options);
    ubx_rxm_cor_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmCor>(
      "ubx_rxm_cor", qos, pub_options);
    ubx_rxm_rtcm_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmRTCM>(
      "ubx_rxm_rtcm", qos, pub_options);
    ubx_rxm_measx_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmMeasx>(
      "ubx_rxm_measx", qos, pub_options);
    ubx_rxm_rawx_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmRawx>(
      "ubx_rxm_rawx", qos, pub_options);
    ubx_rxm_spartn_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmSpartn>(
      "ubx_rxm_spartn", qos, pub_options);
    ubx_rxm_spartnkey_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXRxmSpartnKey>(
      "ubx_rxm_spartnkey", qos, pub_options);
    ubx_esf_status_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXEsfStatus>(
      "ubx_esf_status", qos, pub_options);
    ubx_esf_meas_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXEsfMeas>(
      "ubx_esf_meas", qos, pub_options);
    ubx_mon_comms_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXMonComms>(
      "ubx_mon_comms", qos, pub_options);
    ubx_sec_sig_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXSecSig>(
      "ubx_sec_sig", qos, pub_options);
    ubx_sec_sig_log_pub_ = this->create_publisher<ublox_ubx_msgs::msg::UBXSecSigLog>(
      "ubx_sec_sig_log", qos, pub_options);
    rtcm_pub_ = this->create_publisher<rtcm_msgs::msg::Message>(
      "rtcm", 10);

    // ros2 parameter call backs
    parameters_callback_handle_ =
      this->add_on_set_parameters_callback(
      std::bind(
        &UbloxDGNSSNode::on_set_parameters_callback,
        this, _1));

    usb::connection_in_cb_fn connection_in_callback = std::bind(
      &UbloxDGNSSNode::ublox_in_callback,
      this, _1);
    usb::connection_out_cb_fn connection_out_callback = std::bind(
      &UbloxDGNSSNode::ublox_out_callback, this, _1);
    usb::connection_exception_cb_fn connection_exception_callback = std::bind(
      &UbloxDGNSSNode::ublox_exception_callback, this, _1, _2);
    usb::hotplug_attach_cb_fn usb_hotplug_attach_callback = std::bind(
      &UbloxDGNSSNode::hotplug_attach_callback, this);
    usb::hotplug_detach_cb_fn usb_hotplug_detach_callback = std::bind(
      &UbloxDGNSSNode::hotplug_detach_callback, this);

    std::string node_name(this->get_name());
    hot_start_service_ = this->create_service<ublox_ubx_interfaces::srv::HotStart>(
      node_name + "/hot_start", std::bind(&UbloxDGNSSNode::hot_start_callback, this, _1, _2));
    cold_start_service_ = this->create_service<ublox_ubx_interfaces::srv::ColdStart>(
      node_name + "/cold_start", std::bind(&UbloxDGNSSNode::cold_start_callback, this, _1, _2));
    warm_start_service_ = this->create_service<ublox_ubx_interfaces::srv::WarmStart>(
      node_name + "/warm_start", std::bind(&UbloxDGNSSNode::warm_start_callback, this, _1, _2));
    reset_odo_service_ = this->create_service<ublox_ubx_interfaces::srv::ResetODO>(
      node_name + "/reset_odo", std::bind(&UbloxDGNSSNode::reset_odo_callback, this, _1, _2));

    try {
      usbc_ = std::make_shared<usb::Connection>(F9_VENDOR_ID, F9_PRODUCT_ID, serial_str_);
      usbc_->set_in_callback(connection_in_callback);
      usbc_->set_out_callback(connection_out_callback);
      usbc_->set_exception_callback(connection_exception_callback);
      usbc_->set_hotplug_attach_callback(usb_hotplug_attach_callback);
      usbc_->set_hotplug_detach_callback(usb_hotplug_detach_callback);

      usbc_->init();

      if (!usbc_->devh_valid()) {
        RCLCPP_ERROR(get_logger(), "USBDevice handle not valid. USB device not connected?");
        rclcpp::shutdown();
      }
    } catch (std::string const & msg) {
      RCLCPP_ERROR(this->get_logger(), "usb init error: %s", msg.c_str());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (usb::UsbException & e) {
      RCLCPP_ERROR(this->get_logger(), "usb init UsbException: %s", e.what());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "usb init events exception: %s", e.what());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (const char * msg) {
      RCLCPP_ERROR(this->get_logger(), "usb init events - %s", msg);
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (...)
    {
      RCLCPP_INFO(this->get_logger(), "usb init events - unknown error");
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    }


    log_usbc();

    ubx_queue_.clear();
    ubx_timer_ = create_wall_timer(
      10ms, std::bind(&UbloxDGNSSNode::ubx_timer_callback, this),
      callback_group_ubx_timer_);
    rtcm_queue_.clear();
    rtcm_timer_ = create_wall_timer(
      10ms, std::bind(&UbloxDGNSSNode::rtcm_timer_callback, this),
      callback_group_rtcm_timer_);

    ubx_cfg_ = std::make_shared<ubx::cfg::UbxCfg>(usbc_);
    ubx_cfg_->cfg_val_set_cfgdata_clear();
    ubx_cfg_->cfg_val_set_layer_ram(true);

    ubx_mon_ = std::make_shared<ubx::mon::UbxMon>(usbc_);
    ubx_inf_ = std::make_shared<ubx::inf::UbxInf>(usbc_);
    ubx_nav_ = std::make_shared<ubx::nav::UbxNav>(usbc_);
    ubx_rxm_ = std::make_shared<ubx::rxm::UbxRxm>(usbc_);
    ubx_esf_ = std::make_shared<ubx::esf::UbxEsf>(usbc_);
    ubx_sec_ = std::make_shared<ubx::sec::UbxSec>(usbc_);

    async_initialised_ = false;

    auto handle_usb_events_callback = [this]() -> void
      {
        if (usbc_ == nullptr) {
          return;
        }

        RCLCPP_DEBUG(get_logger(), "start handle_usb_events");
        try {
          usbc_->handle_usb_events();
        } catch (usb::UsbException & e) {
          RCLCPP_ERROR(this->get_logger(), "handle usb events UsbException: %s", e.what());
        } catch (std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "handle usb events exception: %s", e.what());
        } catch (const char * msg) {
          RCLCPP_ERROR(this->get_logger(), "handle usb events - %s", msg);
        } catch (const std::string & msg) {
          RCLCPP_ERROR(this->get_logger(), "handle usb events - %s", msg.c_str());
        }
        ;

        RCLCPP_DEBUG(get_logger(), "finish handle_usb_events");
      };

    handle_usb_events_timer_ = create_wall_timer(
      10ms, handle_usb_events_callback,
      callback_group_usb_events_timer_);

    if (!async_initialised_) {
      RCLCPP_INFO(get_logger(), "ublox_dgnss_init_async start");
      ublox_dgnss_init_async();
      RCLCPP_INFO(get_logger(), "ublox_dgnss_init_async finished");
      async_initialised_ = true;
    }


    rclcpp::SubscriptionOptions sub_options;
    sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    ubx_esf_meas_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXEsfMeas>(
      "/ubx_esf_meas_to_device", 10,
      std::bind(&UbloxDGNSSNode::ubx_esf_meas_callback, this, _1),
      sub_options);
    rtcm_sub_ = this->create_subscription<rtcm_msgs::msg::Message>(
      "/ntrip_client/rtcm", 10,
      std::bind(&UbloxDGNSSNode::rtcm_callback, this, _1),
      sub_options);

    is_initialising_ = false;
  }


  UBLOX_DGNSS_NODE_LOCAL
  ~UbloxDGNSSNode()
  {
    usbc_->shutdown();
    usbc_.reset();
    RCLCPP_INFO(this->get_logger(), "finished");
  }

private:
  bool is_initialising_;

  rclcpp::CallbackGroup::SharedPtr callback_group_usb_events_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_ubx_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_rtcm_timer_;

  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<ubx::cfg::UbxCfg> ubx_cfg_;
  std::shared_ptr<ubx::mon::UbxMon> ubx_mon_;
  std::shared_ptr<ubx::inf::UbxInf> ubx_inf_;
  std::shared_ptr<ubx::nav::UbxNav> ubx_nav_;
  std::shared_ptr<ubx::rxm::UbxRxm> ubx_rxm_;
  std::shared_ptr<ubx::esf::UbxEsf> ubx_esf_;
  std::shared_ptr<ubx::sec::UbxSec> ubx_sec_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
// specific to libusb to to process events asynchronously
  rclcpp::TimerBase::SharedPtr handle_usb_events_timer_;

// don't want to block fetching of messages from the ublox device,
// so put them in a queue, with a timestamp to be processed later
  std::deque<ubx_queue_frame_t> ubx_queue_;
  std::mutex ubx_queue_mutex_;
  std::deque<rtcm_queue_frame_t> rtcm_queue_;
  std::mutex rtcm_queue_mutex_;

  rclcpp::TimerBase::SharedPtr ubx_timer_;
  rclcpp::TimerBase::SharedPtr rtcm_timer_;

  bool async_initialised_;

  std::map<std::string, param_state_t> cfg_param_cache_map_;

  std::string frame_id_;
  const std::string FRAME_ID_PARAM_NAME = "FRAME_ID";

  std::string serial_str_;
  const std::string DEV_STRING_PARAM_NAME = "DEVICE_SERIAL_STRING";

  std::string unique_id_;

  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavClock>::SharedPtr ubx_nav_clock_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavCov>::SharedPtr ubx_nav_cov_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavDOP>::SharedPtr ubx_nav_dop_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavEOE>::SharedPtr ubx_nav_eoe_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavHPPosECEF>::SharedPtr ubx_nav_hp_pos_ecef_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavOdo>::SharedPtr ubx_nav_odo_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavOrb>::SharedPtr ubx_nav_orb_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavSat>::SharedPtr ubx_nav_sat_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavSig>::SharedPtr ubx_nav_sig_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavPosECEF>::SharedPtr ubx_nav_pos_ecef_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavPosLLH>::SharedPtr ubx_nav_pos_llh_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr ubx_nav_pvt_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr ubx_nav_rel_pos_ned_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavStatus>::SharedPtr ubx_nav_status_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavSvin>::SharedPtr ubx_nav_svin_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavTimeUTC>::SharedPtr ubx_nav_time_utc_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavVelECEF>::SharedPtr ubx_nav_vel_ecef_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavVelNED>::SharedPtr ubx_nav_vel_ned_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmCor>::SharedPtr ubx_rxm_cor_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmRTCM>::SharedPtr ubx_rxm_rtcm_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmMeasx>::SharedPtr ubx_rxm_measx_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmRawx>::SharedPtr ubx_rxm_rawx_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmSpartn>::SharedPtr ubx_rxm_spartn_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXRxmSpartnKey>::SharedPtr ubx_rxm_spartnkey_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXEsfStatus>::SharedPtr ubx_esf_status_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXEsfMeas>::SharedPtr ubx_esf_meas_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXMonComms>::SharedPtr ubx_mon_comms_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXSecSig>::SharedPtr ubx_sec_sig_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXSecSigLog>::SharedPtr ubx_sec_sig_log_pub_;
  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXEsfMeas>::SharedPtr ubx_esf_meas_sub_;
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;

  rclcpp::Service<ublox_ubx_interfaces::srv::HotStart>::SharedPtr hot_start_service_;
  rclcpp::Service<ublox_ubx_interfaces::srv::WarmStart>::SharedPtr warm_start_service_;
  rclcpp::Service<ublox_ubx_interfaces::srv::ColdStart>::SharedPtr cold_start_service_;
  rclcpp::Service<ublox_ubx_interfaces::srv::ResetODO>::SharedPtr reset_odo_service_;

  UBLOX_DGNSS_NODE_LOCAL
  void check_for_device_serial_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    // default to empty string
    serial_str_ = "";
    // Check if the parameter exists
    if (!param_client->has_parameter(DEV_STRING_PARAM_NAME)) {
      RCLCPP_INFO(
        this->get_logger(), "Parameter %s not found, will use first ublox device.",
        DEV_STRING_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    serial_str_ = param_client->get_parameter<std::string>(DEV_STRING_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      DEV_STRING_PARAM_NAME.c_str(), serial_str_.c_str());
  }

  UBLOX_DGNSS_NODE_LOCAL
  void check_for_frame_id_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    // default to ubx
    frame_id_ = "ubx";
    // Check if the parameter exists
    if (!param_client->has_parameter(FRAME_ID_PARAM_NAME)) {
      RCLCPP_INFO(
        this->get_logger(), "Parameter %s not found, defaulting to 'ubx' frame_id",
        FRAME_ID_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    frame_id_ = param_client->get_parameter<std::string>(FRAME_ID_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      FRAME_ID_PARAM_NAME.c_str(), frame_id_.c_str());
  }


  UBLOX_DGNSS_NODE_LOCAL
  void log_usbc()
  {
    RCLCPP_INFO(
      this->get_logger(), "usb vendor_id: 0x%04x product_id: 0x%04x " \
      "serial_str: %s bus: %03d address: %03d " \
      "port_number: %d speed: %s num_interfaces: %u " \
      "ep_data out: 0x%02x in: 0x%02x ep_comms in: 0x%02x",
      usbc_->vendor_id(),
      usbc_->product_id(),
      usbc_->serial_str().c_str(),
      usbc_->bus_number(),
      usbc_->device_address(),
      usbc_->port_number(),
      usbc_->device_speed_txt(),
      usbc_->num_interfaces(),
      usbc_->ep_data_out_addr(),
      usbc_->ep_data_in_addr(),
      usbc_->ep_comms_in_addr());
  }

public:
// declare ubx ros parameter
  UBLOX_DGNSS_NODE_LOCAL
  void set_or_declare_ubx_cfg_param(
    const ubx::cfg::ubx_cfg_item_t & ubx_ci,
    const ubx::value_t & ubx_value, bool is_initial = false)
  {
    rclcpp::ParameterValue p_value;

    // auto overrides is on, so if supplied as args/yaml already set
    bool p_set = has_parameter(ubx_ci.ubx_config_item);

    switch (ubx_ci.ubx_type) {
      case ubx::L:
        p_value = rclcpp::ParameterValue(static_cast<bool>(ubx_value.l));
        break;
      case ubx::E1:
      case ubx::U1:
        p_value = rclcpp::ParameterValue(ubx_value.u1);
        break;
      case ubx::X1:
        p_value = rclcpp::ParameterValue(ubx_value.x1);
        break;
      case ubx::I1:
        p_value = rclcpp::ParameterValue(ubx_value.i1);
        break;
      case ubx::E2:
      case ubx::U2:
        p_value = rclcpp::ParameterValue(ubx_value.u2);
        break;
      case ubx::X2:
        p_value = rclcpp::ParameterValue(ubx_value.x2);
        break;
      case ubx::I2:
        p_value = rclcpp::ParameterValue(ubx_value.i2);
        break;
      case ubx::E4:
      case ubx::U4: {
          int64_t value_u4 = (int64_t)ubx_value.u4;
          p_value = rclcpp::ParameterValue(value_u4);
        }
        break;
      case ubx::X4: {
          int64_t value_x4 = (int64_t)ubx_value.x4;
          p_value = rclcpp::ParameterValue(value_x4);
        }
        break;
      case ubx::I4:
        p_value = rclcpp::ParameterValue(ubx_value.i4);
        break;
      case ubx::R4:
        p_value = rclcpp::ParameterValue(ubx_value.r4);
        break;
      case ubx::U8: {
          int64_t value_u8 = (int64_t)ubx_value.u8;
          p_value = rclcpp::ParameterValue(value_u8);
        }
        break;
      case ubx::X8: {
          int64_t value_x8 = (int64_t)ubx_value.x8;
          p_value = rclcpp::ParameterValue(value_x8);
        }
        break;
      case ubx::I8:
        p_value = rclcpp::ParameterValue(ubx_value.i8);
        break;
      case ubx::R8:
        p_value = rclcpp::ParameterValue(ubx_value.r8);
        break;
      default:
        RCLCPP_WARN(get_logger(), "cfg_item: %s type not defined", ubx_ci.ubx_config_item);
    }

    rclcpp::Parameter p;
    p = rclcpp::Parameter(ubx_ci.ubx_config_item, p_value);
    if (is_initial && p_set) {
      // parameter set via argument override at startup
      p = get_parameter(ubx_ci.ubx_config_item);
    }

    // this following cache map contains values retrieved from the GPS device and needs to be set
    // before either declaring or setting the ROS2 parameter. The on set parameter callback
    // functions use this cache map to determine if the values have changed. If they have then
    // it sets the value on the gps device

    auto p_status = PARAM_LOADED;
    if (is_initial) {
      if (p_set) {
        p_status = PARAM_USER;
      } else {
        p_status = PARAM_INITIAL;
      }
    }

    cfg_param_cache_map_[p.get_name()] = {p_value, p_status};
    if (p_set) {
      RCLCPP_DEBUG(get_logger(), "cfg set_parameter name: %s", p.get_name().c_str());
      set_parameter(p);
    } else {
      auto d_value = declare_parameter(p.get_name(), p_value);
      if (d_value != p_value) {
        cfg_param_cache_map_[p.get_name()] = {d_value, PARAM_USER};
        RCLCPP_INFO(
          get_logger(), "cfg declare_parameter name: %s with arguments supplied value ",
          p.get_name().c_str());
      } else {
        RCLCPP_INFO(get_logger(), "cfg declare_parameter name: %s", p.get_name().c_str());
      }
    }
  }

// declare ubx paramters
  UBLOX_DGNSS_NODE_LOCAL
  void ubx_cfg_payload_parameters(std::shared_ptr<ubx::cfg::CfgValGetPayload> cfg_val_get_payload)
  {
    for (const auto kv : cfg_val_get_payload->cfg_data) {
      auto ubx_ci = ubx::cfg::ubxKeyCfgItemMap[kv.ubx_key_id];
      set_or_declare_ubx_cfg_param(ubx_ci, kv.ubx_value);
    }
  }

  rcl_interfaces::msg::SetParametersResult cfg_val_set_from_ubx_ci_p_state(
    ubx::cfg::ubx_cfg_item_t ubx_ci, param_state_t param_state)
  {
    auto parameter = rclcpp::Parameter(ubx_ci.ubx_config_item, param_state.value);
    return cfg_val_set_from_parameter(parameter);
  }

  rcl_interfaces::msg::SetParametersResult cfg_val_set_from_parameter(rclcpp::Parameter parameter)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // find the ubx cfg item
    bool valid = false;
    for (const auto & kv : ubx::cfg::ubxKeyCfgItemMap) {
      auto ubx_key_id = kv.first;
      auto ubx_cfg_item = kv.second;
      if (strcmp(ubx_cfg_item.ubx_config_item, parameter.get_name().c_str()) == 0) {
        valid = true;
        ubx::value_t value {0x0000000000000000};

        try {
          switch (ubx_cfg_item.ubx_type) {
            case ubx::L:
              value.l = parameter.as_bool();
              break;
            case ubx::E1:
            case ubx::U1:
              value.u1 = (ubx::u1_t)parameter.as_int();
              break;
            case ubx::X1:
              value.x1 = (ubx::x1_t)parameter.as_int();
              break;
            case ubx::I1:
              value.i1 = (ubx::i1_t)parameter.as_int();
              break;
            case ubx::E2:
            case ubx::U2:
              value.u2 = (ubx::u2_t)parameter.as_int();
              break;
            case ubx::X2:
              value.x2 = (ubx::x2_t)parameter.as_int();
              break;
            case ubx::I2:
              value.i2 = (ubx::i2_t)parameter.as_int();
              break;
            case ubx::E4:
            case ubx::U4:
              value.u4 = (ubx::u4_t)parameter.as_int();
              break;
            case ubx::X4:
              value.x4 = (ubx::x4_t)parameter.as_int();
              break;
            case ubx::I4:
              value.i4 = (ubx::i4_t)parameter.as_int();
              break;
            case ubx::R4:
              value.r4 = (ubx::r4_t)parameter.as_double();
              break;
            case ubx::U8:
              value.u8 = (ubx::u8_t)parameter.as_int();
              break;
            case ubx::X8:
              value.x8 = (ubx::x8_t)parameter.as_int();
              break;
            case ubx::I8:
              value.i8 = (ubx::i8_t)parameter.as_int();
              break;
            case ubx::R8:
              value.r8 = (ubx::r8_t)parameter.as_double();
              break;
            default:
              RCLCPP_WARN(
                get_logger(), "on_set_parameters_callback cfg_item: %s type not defined",
                ubx_cfg_item.ubx_config_item);
          }

          // using this parameter so update cache map and schedule to send to the gps device
          std::ostringstream voss;
          voss << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex;
          for (size_t i = 0; i < ubx_key_id.storage_size(); i++) {
            voss << +value.bytes[i];
          }
          RCLCPP_DEBUG(
            get_logger(), "cfg_item: %s appending to queue cfg_val_set_key: %s value: %s " \
            "storage_size: %lu",
            ubx_cfg_item.ubx_config_item,
            ubx_key_id.to_hex().c_str(), voss.str().c_str(), ubx_key_id.storage_size());

          ubx_cfg_->cfg_val_set_key_append(ubx_key_id, value);

          cfg_param_cache_map_[parameter.get_name()] =
          {parameter.get_parameter_value(), PARAM_VALSET};
        } catch (const rclcpp::ParameterTypeException & e) {
          valid = false;
          RCLCPP_DEBUG(
            get_logger(), "cfg_val_set_from_parameter ParamterTypeException: %s",
            e.what());
          result.reason = e.what();
          result.reason += " ";
          break;
        } catch (const std::exception & e) {
          valid = false;
          RCLCPP_DEBUG(get_logger(), "cfg_val_set_from_parameter exception: %s", e.what());
          result.reason = e.what();
          result.reason += " ";
          break;
        }
      }
      if (valid) {
        break;
      }
    }     // end for loop over ubxCfgItemMap

    if (!valid) {
      result.reason += parameter.get_name() + " is not valid!";
      result.successful = false;
    }

    return result;
  }

// on set parameters callback function
  UBLOX_DGNSS_NODE_LOCAL
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Define the parameters you want to exclude from processing
    std::set<std::string> excluded_params =
    {"hw_version", "sw_version", "version_extension", "unique_id"};


    try {
      for (const rclcpp::Parameter & parameter : parameters) {
        // Check if the parameter is in the excluded list
        if (excluded_params.find(parameter.get_name()) != excluded_params.end()) {
          RCLCPP_DEBUG(get_logger(), "Skipping parameter: %s", parameter.get_name().c_str());
          continue;  // Skip this parameter
        }

        auto cache_state = cfg_param_cache_map_[parameter.get_name()];
        rclcpp::ParameterValue cache_value = cache_state.value;
        if ((cache_state.status == PARAM_LOADED || cache_state.status == PARAM_VALSET) &&
          cache_value == parameter.get_parameter_value())
        {
          RCLCPP_DEBUG(
            get_logger(), "on_set_parameters_callback parameter: %s same in cache. Not updating.",
            parameter.get_name().c_str());
        } else if (is_initialising_ && cache_state.status == PARAM_INITIAL) {
          RCLCPP_DEBUG(
            get_logger(), "on_set_parameters_callback parameter: %s initial.",
            parameter.get_name().c_str());
        } else {
          if (cache_state.status == PARAM_USER) {
            RCLCPP_DEBUG(
              get_logger(), "on_set_parameters_callback parameter: %s args/yaml supplied.",
              parameter.get_name().c_str());
          }
          result = cfg_val_set_from_parameter(parameter);
        }

        if (!result.successful) {
          RCLCPP_WARN(
            get_logger(), "parameter %s not set - %s",
            parameter.get_name().c_str(), result.reason.c_str());
          break;
        } else {
          RCLCPP_INFO(
            get_logger(), "parameter set %s: %s",
            parameter.get_name().c_str(), parameter.value_to_string().c_str());
        }
      }

      if (!is_initialising_ && result.successful && ubx_cfg_->cfg_val_set_cfgdata_size() > 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "on_set_parameters_callback sending async cfg_val_set_poll for parameter changes... ");
        ubx_cfg_->cfg_val_set_transaction(0);
        ubx_cfg_->cfg_val_set_poll_async();
        ubx_cfg_->cfg_val_set_cfgdata_clear();
      }
    } catch (const ubx::UbxValueException & e) {
      RCLCPP_ERROR(get_logger(), "UbxValueException: %s", e.what());
      result.successful = false;
      result.reason = e.what();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Exception: %s",
        e.what());
      result.successful = false;
      result.reason = e.what();
    }

    return result;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void hot_start_callback(
    const std::shared_ptr<ublox_ubx_interfaces::srv::HotStart::Request> request,
    std::shared_ptr<ublox_ubx_interfaces::srv::HotStart::Response> response)
  {
    RCLCPP_WARN(
      get_logger(), "hot_start service called reset_type: %s",
      ubx::to_hex(request->reset_type).c_str());
    ubx_cfg_->cfg_rst_set_nav_bbr_mask(ubx::cfg::NAV_BBR_HOT_START);
    ubx_cfg_->cfg_rst_set_reset_mode(request->reset_type);
    ubx_cfg_->cfg_rst_command_async();
    (void)response;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void warm_start_callback(
    const std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart::Request> request,
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart::Response> response)
  {
    RCLCPP_WARN(
      get_logger(), "warm_start service called reset_type: %s",
      ubx::to_hex(request->reset_type).c_str());
    ubx_cfg_->cfg_rst_set_nav_bbr_mask(ubx::cfg::NAV_BBR_WARM_START);
    ubx_cfg_->cfg_rst_set_reset_mode(request->reset_type);
    ubx_cfg_->cfg_rst_command_async();
    (void)response;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void cold_start_callback(
    const std::shared_ptr<ublox_ubx_interfaces::srv::ColdStart::Request> request,
    std::shared_ptr<ublox_ubx_interfaces::srv::ColdStart::Response> response)
  {
    RCLCPP_WARN(
      get_logger(), "cold_start service called reset_type: %s",
      ubx::to_hex(request->reset_type).c_str());
    ubx_cfg_->cfg_rst_set_nav_bbr_mask(ubx::cfg::NAV_BBR_COLD_START);
    ubx_cfg_->cfg_rst_set_reset_mode(request->reset_type);
    ubx_cfg_->cfg_rst_command_async();
    (void)response;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void reset_odo_callback(
    const std::shared_ptr<ublox_ubx_interfaces::srv::ResetODO::Request> request,
    std::shared_ptr<ublox_ubx_interfaces::srv::ResetODO::Response> response)
  {
    RCLCPP_WARN(get_logger(), "reset_odo service");
    RCLCPP_DEBUG(get_logger(), "ubx_nav_resetodo poll_async ...");
    ubx_nav_->resetodo()->poll_async();
    (void)request;
    (void)response;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_esf_meas_callback(const ublox_ubx_msgs::msg::UBXEsfMeas & msg) const
  {
    if (msg.num_meas > 0 && msg.data.size() != msg.num_meas) {
      RCLCPP_WARN(
        get_logger(),
        "ubx_esf_meas_callback num_meas %d != data array size %ld - not sending to usb",
        msg.num_meas,
        msg.data.size());
      return;
    }

    if (ubx_esf_->usbc() == nullptr || !ubx_esf_->usbc()->devh_valid()) {
      RCLCPP_WARN(get_logger(), "usbc_ not valid - not sending ubx_esf_meas to device!");
      return;
    }

    if (!ubx_esf_->usbc()->attached()) {
      RCLCPP_WARN(get_logger(), "USB device not attached - not sending ubx_esf_meas to device!");
      return;
    }

    ubx_esf_->meas_full()->payload()->load_from_msg(msg);
    RCLCPP_DEBUG(
      get_logger(), "ubx_esf_meas_callback sending payload - %s",
      ubx_esf_->meas_full()->payload()->to_string().c_str());

    ubx_esf_->meas_full()->poll_async();
  }

  UBLOX_DGNSS_NODE_LOCAL
  void rtcm_callback(const rtcm_msgs::msg::Message & msg) const
  {
    if (usbc_ == nullptr || !usbc_->devh_valid()) {
      RCLCPP_WARN(get_logger(), "usbc_ not valid - not sending rtcm to device!");
      return;
    }
    if (!usbc_->attached()) {
      RCLCPP_WARN(get_logger(), "USB device not attached - not sending rtcm to device!");
      return;
    }
    // Additional check for endpoints
    if (usbc_->ep_data_out_addr() == 0 || usbc_->ep_data_out_addr() == 0xaaaa) {
      RCLCPP_WARN(get_logger(), "USB device endpoints not ready - not sending rtcm to device!");
      return;
    }

    std::ostringstream oss;
    std::vector<u_char> data_out;
    data_out.resize(msg.message.size());
    for (auto b : msg.message) {
      oss << std::hex << std::setfill('0') << std::setw(2) << +b;
      data_out.push_back(b);
    }

    RCLCPP_DEBUG(get_logger(), "rtcm_callback msg.message: 0x%s", oss.str().c_str());

    usbc_->write_buffer(data_out.data(), data_out.size());
  }

// handle host in from ublox gps to host callback
  UBLOX_DGNSS_NODE_LOCAL
  void ublox_in_callback(libusb_transfer * transfer_in)
  {
    rclcpp::Time ts = rclcpp::Clock().now();

    const char * remove_any_of = "\n\r";

    size_t len = transfer_in->actual_length;
    unsigned char * buf = transfer_in->buffer;

    if (len > 0) {
      // NMEA string starts with a $
      if (buf[0] == 0x24) {
        buf[len] = 0;
        for (size_t i = len - 2; i < len; i++) {
          if (strchr(remove_any_of, buf[i])) {
            buf[i] = 0;
          }
        }
        RCLCPP_INFO(get_logger(), "nmea: %s", buf);
      } else {
        // UBX starts with 0x65 0x62
        if (len > 2 && buf[0] == ubx::UBX_SYNC_CHAR_1 && buf[1] == ubx::UBX_SYNC_CHAR_2) {
          auto frame = std::make_shared<ubx::Frame>();
          frame->buf.reserve(len);
          frame->buf.resize(len);
          memcpy(frame->buf.data(), &buf[0], len);
          frame->from_buf_build();
          ubx_queue_frame_t queue_frame {ts, frame, FrameType::frame_in};
          {
            const std::lock_guard<std::mutex> lock(ubx_queue_mutex_);
            ubx_queue_.push_back(queue_frame);
          }

          // RTCM3 messages start with a 0xD3 for preamble, followed by 0x00
        } else {
          if (len > 2 && buf[0] == 0xD3 && buf[1] == 0x00) {
            std::vector<uint8_t> frame_buf;
            frame_buf.reserve(len);
            frame_buf.resize(len);
            memcpy(frame_buf.data(), &buf[0], len);
            rtcm_queue_frame_t queue_frame {ts, frame_buf, FrameType::frame_in};
            {
              const std::lock_guard<std::mutex> lock(rtcm_queue_mutex_);
              rtcm_queue_.push_back(queue_frame);
            }
          }
        }

        std::ostringstream os;
        os << "0x";
        for (size_t i = 0; i < len; i++) {
          os << std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
        }

        RCLCPP_DEBUG(get_logger(), "in - buf: %s", os.str().c_str());
      }
    } else {
      RCLCPP_DEBUG(get_logger(), "in - buf len is zero");
    }

    size_t num_transfer_in_queued = usbc_->queued_transfer_in_num();
    if (num_transfer_in_queued > 1) {
      RCLCPP_WARN(
        get_logger(), "too many transfer in transfers are queued (%lu)", num_transfer_in_queued);
    }
  }

// handle out to ublox gps device to host callback
  UBLOX_DGNSS_NODE_PUBLIC
  void ublox_out_callback(libusb_transfer * transfer_out)
  {
    rclcpp::Time ts = rclcpp::Clock().now();

    size_t len = transfer_out->actual_length;
    unsigned char * buf = transfer_out->buffer;

    // UBX starts with 0x65 0x62
    if (len > 2 && buf[0] == ubx::UBX_SYNC_CHAR_1 && buf[1] == ubx::UBX_SYNC_CHAR_2) {
      auto frame = std::make_shared<ubx::Frame>();
      frame->buf.resize(len);
      memcpy(frame->buf.data(), &buf[0], len);
      frame->from_buf_build();
      ubx_queue_frame_t queue_frame {ts, frame, FrameType::frame_out};
      {
        const std::lock_guard<std::mutex> lock(ubx_queue_mutex_);
        ubx_queue_.push_back(queue_frame);
      }
    }

    std::ostringstream os;
    os << "0x";
    for (int i = 0; i < transfer_out->actual_length; i++) {
      os << std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
    }

    RCLCPP_DEBUG(
      this->get_logger(), "out - status: %d length: %d buf: %s",
      transfer_out->status, transfer_out->actual_length,
      os.str().c_str());
  }

  UBLOX_DGNSS_NODE_PUBLIC
  void ublox_exception_callback(usb::UsbException e, void * user_data)
  {
    (void)user_data;
    RCLCPP_ERROR(this->get_logger(), "ublox exception: %s", e.what());
  }

  UBLOX_DGNSS_NODE_PUBLIC
  void hotplug_attach_callback()
  {
    if (is_initialising_) {
      RCLCPP_WARN(get_logger(), "usb hotplug attach - initial");
      return;
    }

    RCLCPP_WARN(get_logger(), "usb hotplug attach");

    // Strict device readiness check
    bool ready = usbc_ &&
      usbc_->devh_valid() &&
      usbc_->num_interfaces() > 0 &&
      usbc_->ep_data_in_addr() != 0 &&
      usbc_->ep_data_in_addr() != 0xaaaa &&
      usbc_->ep_data_out_addr() != 0 &&
      usbc_->ep_data_out_addr() != 0xaaaa;

    if (ready) {
      usbc_->init_async();
      RCLCPP_DEBUG(get_logger(), "ubx_mon_ver poll_async ...");
      ubx_mon_->ver()->poll_async();

      RCLCPP_DEBUG(get_logger(), "ublox_val_set_all_cfg_items_async() ...");
      ublox_val_set_all_cfg_items_async();

      // ... readiness checks ...
      if (ready && !async_initialised_) {
        RCLCPP_INFO(get_logger(), "ublox_dgnss_init_async start");
        ublox_dgnss_init_async();
        RCLCPP_INFO(get_logger(), "ublox_dgnss_init_async finished");
        async_initialised_ = true;
      }

    } else {
      RCLCPP_WARN(get_logger(), "Device not ready in hotplug_attach_callback, will retry in 100ms");
      auto self = this;
      rclcpp::TimerBase::SharedPtr retry_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [self]() {
          self->hotplug_attach_callback();
        });
      // Optionally store retry_timer in a member variable to keep it alive
    }
  }

  UBLOX_DGNSS_NODE_PUBLIC
  void hotplug_detach_callback()
  {
    RCLCPP_WARN(this->get_logger(), "usb hotplug detach");
  }

// UBLOX_DGNSS_NODE_PUBLIC
// void ublox_timer_callback() {
//   const char *remove_any_of = "\n\r";

//   static u_char buf[64*10+1];
//   int len;
//   try {
//     len = usbc_->read_chars(buf, sizeof(buf));
//     buf[len] = 0;
//     if (len > 0) {
//       for (int i = len -2; i < len; i++)
//         if (strchr(remove_any_of, buf[i]))
//           buf[i]=0;
//     }
//     if (buf[0]==0x24)
//       RCLCPP_INFO(this->get_logger(),"len: %d read: %s", len, &buf);
//     else {
//       std::ostringstream os;
//       os << "0x";
//       for (size_t i = 0; i < len; i++) {
//         os <<std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
//       }
//       RCLCPP_INFO(this->get_logger(),"len: %d buf: %s", len, os.str().c_str());
//     }
//   } catch (usb::UsbException& e) {
//     RCLCPP_WARN(this->get_logger(), "usb read_chars exception: %s", e.what());
//   } catch (usb::TimeoutException& e) {
//     RCLCPP_WARN(this->get_logger(), "usb read_chars timeout: %s", e.what());
//   };
// }

private:
  UBLOX_DGNSS_NODE_LOCAL
  void ubx_timer_callback()
  {
    // if we dont have anything to do just return
    if (ubx_queue_.size() == 0) {
      return;
    }

    while (ubx_queue_.size() > 0) {
      try {
        ubx_queue_frame_t f = ubx_queue_[0];
        switch (f.frame_type) {
          case FrameType::frame_in:
            ubx_queue_frame_in(&f);
            break;
          case FrameType::frame_out:
            ubx_queue_frame_out(&f);
            break;
          default:
            RCLCPP_ERROR(
              get_logger(), "Unknown ubx_queue frame_type: %d - doing nothing", f.frame_type);
        }
      } catch (const ubx::UbxValueException & e) {
        RCLCPP_ERROR(get_logger(), "ubx_queue_frame_in UbxValueException: %s", e.what());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "ubx_queue_frame_in exception: %s", e.what());
      }

      {
        const std::lock_guard<std::mutex> lock(ubx_queue_mutex_);
        ubx_queue_.pop_front();
      }
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void rtcm_timer_callback()
  {
    // if we dont have anything to do just return
    if (rtcm_queue_.size() == 0) {
      return;
    }

    while (rtcm_queue_.size() > 0) {
      try {
        rtcm_queue_frame_t f = rtcm_queue_[0];
        switch (f.frame_type) {
          case FrameType::frame_in:
            rtcm_queue_frame_in(&f);
            break;
          case FrameType::frame_out:
            RCLCPP_WARN(
              get_logger(),
              "Received an rtcm_queue_frame_t with frame_type as frame_out - doing nothing");
            break;
          default:
            RCLCPP_ERROR(
              get_logger(), "Unknown rtcm_queue frame_type: %d - doing nothing", f.frame_type);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "rtcm_queue_ exception: %s", e.what());
      }

      {
        const std::lock_guard<std::mutex> lock(rtcm_queue_mutex_);
        rtcm_queue_.pop_front();
      }
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void rtcm_queue_frame_in(rtcm_queue_frame_t * f)
  {
    std::ostringstream oss;
    for (auto b : f->buf) {
      oss << std::hex << std::setfill('0') << std::setw(2) << +b;
    }
    RCLCPP_DEBUG(get_logger(), "rtcm message payload - 0x%s", oss.str().c_str());
    auto msg = std::make_unique<rtcm_msgs::msg::Message>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate fields
    msg->message = f->buf;

    // Publish the message
    rtcm_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  bool ubx_frame_checksum_check(std::shared_ptr<ubx::Frame> ubx_f)
  {
    bool check_result = true;
    ubx::u1_t ck_a, ck_b;
    std::tie(ck_a, ck_b) = ubx_f->ubx_check_sum();

    if (ck_a != ubx_f->ck_a || ck_b != ubx_f->ck_b) {
      check_result = false;
    }
    return check_result;
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_queue_frame_in(ubx_queue_frame_t * f)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x in payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      f->ubx_frame->payload_to_hex().c_str());

    if (!ubx_frame_checksum_check(f->ubx_frame)) {
      RCLCPP_WARN(
        get_logger(), "ubx class: 0x%02x id: 0x%02x in checksum failed! Frame ignored ...",
        f->ubx_frame->msg_class,
        f->ubx_frame->msg_id);
      return;
    }

    switch (f->ubx_frame->msg_class) {
      case ubx::UBX_CFG:
        ubx_cfg_in_frame(f);
        break;
      case ubx::UBX_ACK:
        ubx_ack_frame(f);
        break;
      case ubx::UBX_MON:
        ubx_mon_in_frame(f);
        break;
      case ubx::UBX_INF:
        ubx_inf_in_frame(f);
        break;
      case ubx::UBX_NAV:
        ubx_nav_in_frame(f);
        break;
      case ubx::UBX_RXM:
        ubx_rxm_in_frame(f);
        break;
      case ubx::UBX_ESF:
        ubx_esf_in_frame(f);
        break;
      case ubx::UBX_SEC:
        ubx_sec_in_frame(f);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x unknown ... doing nothing", f->ubx_frame->msg_class);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_queue_frame_out(ubx_queue_frame_t * f)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x out payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      f->ubx_frame->payload_to_hex().c_str());

    if (!ubx_frame_checksum_check(f->ubx_frame)) {
      RCLCPP_WARN(
        get_logger(), "ubx class: 0x%02x id: 0x%02x checksum failed! Frame ignored ...",
        f->ubx_frame->msg_class,
        f->ubx_frame->msg_id);
      return;
    }

    switch (f->ubx_frame->msg_class) {
      case ubx::UBX_CFG:
        ubx_cfg_out_frame(f);
        break;
      case ubx::UBX_MON:
        ubx_mon_out_frame(f);
        break;
      case ubx::UBX_NAV:
        ubx_nav_out_frame(f);
        break;
      case ubx::UBX_RXM:
        ubx_rxm_out_frame(f);
        break;
      case ubx::UBX_ESF:
        ubx_esf_out_frame(f);
        break;
      case ubx::UBX_SEC:
        ubx_sec_out_frame(f);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "unkown class: 0x%02x unknown ... doing nothing", f->ubx_frame->msg_class);
    }
  }


  UBLOX_DGNSS_NODE_LOCAL
  void ubx_cfg_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_CFG_VALGET:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x cfg val get poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_CFG_VALSET:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x cfg val set sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_CFG_RST:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x cfg rst sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_mon_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_MON_VER:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x mon ver poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_NAV_CLOCK:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav clock poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_COV:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav cov poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_DOP:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav dop poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_POSECEF:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav pos ecef poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_HPPOSECEF:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav hp pos ecef poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_POSLLH:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav pos llh poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_HPPOSLLH:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav hp pos llh poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_ODO:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav odo poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_ORB:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav orb poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_SAT:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav sat poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_SIG:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav sig poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_RESETODO:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav reset odo poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_PVT:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav pvt poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_STATUS:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav status poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_RELPOSNED:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav relposned poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_TIMEUTC:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav timeutc poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_VELECEF:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav velecef poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_NAV_VELNED:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x nav velned poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_RXM_COR:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm cor poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_RXM_RTCM:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm rtcm poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_RXM_MEASX:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm measx poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_RXM_RAWX:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm rawx poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_RXM_SPARTN:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm spartn poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_RXM_SPARTNKEY:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x rxm spartnkey poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_esf_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_ESF_STATUS:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x esf status poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_ESF_MEAS:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x esf meas poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }
  UBLOX_DGNSS_NODE_LOCAL
  void ubx_sec_out_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_SEC_SIG:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x sec sig poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_SEC_SIGLOG:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x sec siglog poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      case ubx::UBX_SEC_UNIQID:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x sec uniqid poll sent to usb device",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }
  UBLOX_DGNSS_NODE_LOCAL
  void ubx_ack_frame(ubx_queue_frame_t * f)
  {
    std::shared_ptr<ubx::ack::AckAckPayload> payload_ack_ack;
    std::shared_ptr<ubx::ack::AckNakPayload> payload_ack_nak;
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_ACK_ACK:
        payload_ack_ack = std::make_shared<ubx::ack::AckAckPayload>(
          f->ubx_frame->payload,
          f->ubx_frame->length);
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x ack ack payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          payload_ack_ack->to_string().c_str());
        break;
      case ubx::UBX_ACK_NAK:
        payload_ack_nak = std::make_shared<ubx::ack::AckNakPayload>(
          f->ubx_frame->payload,
          f->ubx_frame->length);
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x ack nak payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          payload_ack_nak->to_string().c_str());
        // TODO(someday) investigate how to get a message about why it returned a nak
        // switch (payload_ack_nak->msg_id) {
        //   case ubx::UBX_CFG_VALSET:
        //       if (ubx_cfg_->cfg_val_set_frame().use_count()>0) {
        //         RCLCPP_WARN(get_logger(), "retrying last cfg val set ...");
        //         ubx_cfg_->cfg_val_set_poll_retry_async();
        //         // only retry once
        //         ubx_cfg_->cfg_val_set_frame_reset();
        //       }
        //     break;
        //   default:
        //     break;
        // }
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_cfg_in_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_CFG_VALGET:
        ubx_cfg_->set_cfg_val_get_frame(f->ubx_frame);
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x cfg polled payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_cfg_->cfg_val_get_payload()->to_string().c_str());
        ubx_cfg_payload_parameters(ubx_cfg_->cfg_val_get_payload());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_mon_ver_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::mon::ver::MonVerPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x mon ver polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto sw_version =
      std::string(reinterpret_cast<char *>(payload->sw_version));
    auto hw_version =
      std::string(reinterpret_cast<char *>(payload->hw_version));
    this->declare_parameter("sw_version", sw_version);
    this->declare_parameter("hw_version", hw_version);

    std::string version_extensions;
    for (const auto & e : payload->extension) {
      if (!version_extensions.empty()) {
        version_extensions += " ";       // Add a space before appending next element
      }
      version_extensions += e;       // Append the current element
    }

    // Declare the parameter with the aggregated string
    this->declare_parameter("version_extension", version_extensions);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_mon_comms_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::mon::comms::MonCommsPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x mon comms polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXMonComms>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    msg->version = payload->version;
    msg->n_ports = payload->nPorts;
    msg->tx_errors = payload->txErrors;

    for (size_t i = 0; i < 4; i++) {
      msg->prot_ids[i] = payload->protIds[i];
    }

    for (size_t i = 0; i < payload->nPorts; i++) {
      ublox_ubx_msgs::msg::CommsPortInfo port;
      port.port_id = payload->ports[i].portId;
      port.tx_pending = payload->ports[i].txPending;
      port.tx_bytes = payload->ports[i].txBytes;
      port.tx_usage = payload->ports[i].txUsage;
      port.tx_peak_usage = payload->ports[i].txPeakUsage;
      port.rx_pending = payload->ports[i].rxPending;
      port.rx_bytes = payload->ports[i].rxBytes;
      port.rx_usage = payload->ports[i].rxUsage;
      port.rx_peak_usage = payload->ports[i].rxPeakUsage;
      port.overrun_errs = payload->ports[i].overrunErrs;

      for (size_t j = 0; j < 4; j++) {
        port.msgs[i] = payload->ports[i].msgs[j];
      }

      port.skipped = payload->ports[i].skipped;

      msg->ports.push_back(port);
    }

    ubx_mon_comms_pub_->publish(std::move(msg));
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_mon_in_frame(ubx_queue_frame_t * f)
  {
    ubx_mon_->frame(f->ubx_frame);
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_MON_VER:
        ubx_mon_ver_pub(f, ubx_mon_->ver()->payload());
        break;
      case ubx::UBX_MON_COMMS:
        ubx_mon_comms_pub(f, ubx_mon_->comms()->payload());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }


  UBLOX_DGNSS_NODE_LOCAL
  void ubx_inf_in_frame(ubx_queue_frame_t * f)
  {
    ubx_inf_->frame(f->ubx_frame);
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_INF_DEBUG:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x inf debug payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_inf_->debug()->payload()->to_string().c_str());
        break;
      case ubx::UBX_INF_ERROR:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x inf error payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_inf_->error()->payload()->to_string().c_str());
        break;
      case ubx::UBX_INF_NOTICE:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x inf notice payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_inf_->notice()->payload()->to_string().c_str());
        break;
      case ubx::UBX_INF_TEST:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x inf test payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_inf_->test()->payload()->to_string().c_str());
        break;
      case ubx::UBX_INF_WARNING:
        RCLCPP_DEBUG(
          get_logger(), "ubx class: 0x%02x id: 0x%02x inf warning payload - %s",
          f->ubx_frame->msg_class, f->ubx_frame->msg_id,
          ubx_inf_->warning()->payload()->to_string().c_str());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_in_frame(ubx_queue_frame_t * f)
  {
    ubx_nav_->frame(f->ubx_frame);
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_NAV_CLOCK:
        ubx_nav_clock_pub(f, ubx_nav_->clock()->payload());
        break;
      case ubx::UBX_NAV_COV:
        ubx_nav_cov_pub(f, ubx_nav_->cov()->payload());
        break;
      case ubx::UBX_NAV_DOP:
        ubx_nav_dop_pub(f, ubx_nav_->dop()->payload());
        break;
      case ubx::UBX_NAV_EOE:
        ubx_nav_eoe_pub(f, ubx_nav_->eoe()->payload());
        break;
      case ubx::UBX_NAV_HPPOSECEF:
        ubx_nav_hp_pos_ecef_pub(f, ubx_nav_->hpposecef()->payload());
        break;
      case ubx::UBX_NAV_HPPOSLLH:
        ubx_nav_hp_pos_llh_pub(f, ubx_nav_->hpposllh()->payload());
        break;
      case ubx::UBX_NAV_ODO:
        ubx_nav_odo_pub(f, ubx_nav_->odo()->payload());
        break;
      case ubx::UBX_NAV_ORB:
        ubx_nav_orb_pub(f, ubx_nav_->orb()->payload());
        break;
      case ubx::UBX_NAV_SAT:
        ubx_nav_sat_pub(f, ubx_nav_->sat()->payload());
        break;
      case ubx::UBX_NAV_SIG:
        ubx_nav_sig_pub(f, ubx_nav_->sig()->payload());
        break;
      case ubx::UBX_NAV_POSECEF:
        ubx_nav_pos_ecef_pub(f, ubx_nav_->posecef()->payload());
        break;
      case ubx::UBX_NAV_POSLLH:
        ubx_nav_pos_llh_pub(f, ubx_nav_->posllh()->payload());
        break;
      case ubx::UBX_NAV_PVT:
        ubx_nav_pvt_pub(f, ubx_nav_->pvt()->payload());
        break;
      case ubx::UBX_NAV_RELPOSNED:
        ubx_nav_rel_pos_ned_pub(f, ubx_nav_->relposned()->payload());
        break;
      case ubx::UBX_NAV_STATUS:
        ubx_nav_status_pub(f, ubx_nav_->status()->payload());
        break;
      case ubx::UBX_NAV_SVIN:
        ubx_nav_svin_pub(f, ubx_nav_->svin()->payload());
        break;
      case ubx::UBX_NAV_TIMEUTC:
        ubx_nav_time_utc_pub(f, ubx_nav_->timeutc()->payload());
        break;
      case ubx::UBX_NAV_VELECEF:
        ubx_nav_vel_ecef_pub(f, ubx_nav_->velecef()->payload());
        break;
      case ubx::UBX_NAV_VELNED:
        ubx_nav_vel_ned_pub(f, ubx_nav_->velned()->payload());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_in_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_RXM_COR:
        ubx_rxm_->cor()->frame(f->ubx_frame);
        ubx_rxm_cor_pub(f, ubx_rxm_->cor()->payload());
        break;
      case ubx::UBX_RXM_RTCM:
        ubx_rxm_->rtcm()->frame(f->ubx_frame);
        ubx_rxm_rtcm_pub(f, ubx_rxm_->rtcm()->payload());
        break;
      case ubx::UBX_RXM_MEASX:
        ubx_rxm_->measx()->frame(f->ubx_frame);
        ubx_rxm_measx_pub(f, ubx_rxm_->measx()->payload());
        break;
      case ubx::UBX_RXM_RAWX:
        ubx_rxm_->rawx()->frame(f->ubx_frame);
        ubx_rxm_rawx_pub(f, ubx_rxm_->rawx()->payload());
        break;
      case ubx::UBX_RXM_SPARTN:
        ubx_rxm_->spartn()->frame(f->ubx_frame);
        ubx_rxm_spartn_pub(f, ubx_rxm_->spartn()->payload());
        break;
      case ubx::UBX_RXM_SPARTNKEY:
        ubx_rxm_->spartnkey()->frame(f->ubx_frame);
        ubx_rxm_spartnkey_pub(f, ubx_rxm_->spartnkey()->payload());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_esf_in_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_ESF_STATUS:
        ubx_esf_->status()->frame(f->ubx_frame);
        ubx_esf_status_pub(f, ubx_esf_->status()->payload());
        break;
      case ubx::UBX_ESF_MEAS:
        ubx_esf_->meas()->frame(f->ubx_frame);
        ubx_esf_meas_pub(f, ubx_esf_->meas()->payload());
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
          f->ubx_frame->msg_class,
          f->ubx_frame->msg_id);
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_sec_in_frame(ubx_queue_frame_t * f)
  {
    switch (f->ubx_frame->msg_id) {
      case ubx::UBX_SEC_SIG: {
          ubx_sec_->sig()->frame(f->ubx_frame);
          ubx_sec_sig_pub(f, ubx_sec_->sig()->payload());
          break;
        }
      case ubx::UBX_SEC_SIGLOG: {
          ubx_sec_->siglog()->frame(f->ubx_frame);
          ubx_sec_siglog_pub(f, ubx_sec_->siglog()->payload());
          break;
        }
      case ubx::UBX_SEC_UNIQID: {
          ubx_sec_->uniqid()->frame(f->ubx_frame);
          auto unique_id = ubx_sec_->uniqid()->payload()->unique_id;
          std::ostringstream oss;
          oss << std::hex << std::setfill('0') << std::uppercase
              << std::setw(2) << static_cast<int>(unique_id[0])
              << std::setw(2) << static_cast<int>(unique_id[1])
              << std::setw(2) << static_cast<int>(unique_id[2])
              << std::setw(2) << static_cast<int>(unique_id[3])
              << std::setw(2) << static_cast<int>(unique_id[4]);
          unique_id_ = oss.str();
          this->declare_parameter("unique_id", unique_id_);
          RCLCPP_INFO(
            get_logger(), "ubx sec unique_id: 0x%s",
            unique_id_.c_str());
          // ubx_sec_uniqid_pub(f, ubx_sec_->uniqid()->payload());
          break;
        }
      default: {
          RCLCPP_WARN(
            get_logger(), "ubx class: 0x%02x id: 0x%02x unknown ... doing nothing",
            f->ubx_frame->msg_class,
            f->ubx_frame->msg_id);
        }
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_vel_ecef_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::velecef::NavVelECEFPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav velecef polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavVelECEF>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->ecef_vx = payload->ecefVX;
    msg->ecef_vy = payload->ecefVY;
    msg->ecef_vz = payload->ecefVZ;
    msg->s_acc = payload->sAcc;

    ubx_nav_vel_ecef_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_vel_ned_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::velned::NavVelNEDPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav velned polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavVelNED>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->vel_n = payload->velN;
    msg->vel_e = payload->velE;
    msg->vel_d = payload->velD;
    msg->speed = payload->speed;
    msg->g_speed = payload->gSpeed;
    msg->heading = payload->heading;
    msg->s_acc = payload->sAcc;
    msg->c_acc = payload->cAcc;

    ubx_nav_vel_ned_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_time_utc_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::timeutc::NavTimeUTCPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav timeutc polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavTimeUTC>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->t_acc = payload->tAcc;
    msg->nano = payload->nano;
    msg->year = payload->year;
    msg->month = payload->month;
    msg->day = payload->day;
    msg->hour = payload->hour;
    msg->min = payload->min;
    msg->sec = payload->sec;
    msg->valid_tow = payload->valid.bits.validTOW;
    msg->valid_wkn = payload->valid.bits.validWKN;
    msg->valid_utc = payload->valid.bits.validUTC;
    msg->utc_std.id = payload->valid.bits.utcStandard;

    ubx_nav_time_utc_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_rel_pos_ned_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::relposned::NavRelPosNedPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav relposned polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavRelPosNED>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->ref_station_id = payload->refStationId;
    msg->itow = payload->iTOW;
    msg->rel_pos_n = payload->relPosN;
    msg->rel_pos_e = payload->relPosE;
    msg->rel_pos_d = payload->relPosD;
    msg->rel_pos_length = payload->relPosLength;
    msg->rel_pos_heading = payload->relPosHeading;
    msg->rel_pos_hp_n = payload->relPosHPN;
    msg->rel_pos_hp_e = payload->relPosHPE;
    msg->rel_pos_hp_d = payload->relPosHPD;
    msg->rel_pos_hp_length = payload->relPosHPLength;
    msg->acc_n = payload->accN;
    msg->acc_e = payload->accE;
    msg->acc_d = payload->accD;
    msg->acc_length = payload->accLength;
    msg->acc_heading = payload->accHeading;
    msg->gnss_fix_ok = payload->flags.bits.gnssFixOK;
    msg->diff_soln = payload->flags.bits.diffSoln;
    msg->rel_pos_valid = payload->flags.bits.relPosValid;
    msg->carr_soln.status = payload->flags.bits.carrSoln;
    msg->is_moving = payload->flags.bits.isMoving;
    msg->ref_pos_miss = payload->flags.bits.refPosMiss;
    msg->ref_obs_miss = payload->flags.bits.refObsMiss;
    msg->rel_pos_heading_valid = payload->flags.bits.relPosHeadingValid;
    msg->rel_pos_normalized = payload->flags.bits.relPosNormalized;

    ubx_nav_rel_pos_ned_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_pvt_pub(ubx_queue_frame_t * f, std::shared_ptr<ubx::nav::pvt::NavPvtPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav pvt polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavPVT>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->year = payload->year;
    msg->month = payload->month;
    msg->day = payload->day;
    msg->hour = payload->hour;
    msg->min = payload->min;
    msg->sec = payload->sec;
    msg->valid_date = payload->valid.bits.validDate;
    msg->valid_time = payload->valid.bits.validTime;
    msg->fully_resolved = payload->valid.bits.fullyResolved;
    msg->valid_mag = payload->valid.bits.validMag;
    msg->t_acc = payload->tAcc;
    msg->nano = payload->nano;
    msg->gps_fix.fix_type = payload->fixType;
    msg->gnss_fix_ok = payload->flags.bits.gnssFixOK;
    msg->diff_soln = payload->flags.bits.diffSoln;
    msg->psm.state = payload->flags.bits.psmState;
    msg->head_veh_valid = payload->flags.bits.headVehValid;
    msg->carr_soln.status = payload->flags.bits.carrSoln;
    msg->confirmed_avail = payload->flags2.bits.confirmedAvailable;
    msg->confirmed_date = payload->flags2.bits.confirmedDate;
    msg->confirmed_time = payload->flags2.bits.confirmedTime;
    msg->num_sv = payload->numSV;
    msg->lon = payload->lon;
    msg->lat = payload->lat;
    msg->height = payload->height;
    msg->hmsl = payload->hMSL;
    msg->h_acc = payload->hAcc;
    msg->v_acc = payload->vAcc;
    msg->vel_n = payload->velN;
    msg->vel_e = payload->velE;
    msg->vel_d = payload->velD;
    msg->g_speed = payload->gSpeed;
    msg->head_mot = payload->headMot;
    msg->s_acc = payload->sAcc;
    msg->head_acc = payload->headAcc;
    msg->p_dop = payload->pDOP;
    msg->invalid_llh = payload->flags3.bits.invalidLLH;
    msg->head_veh = payload->headVeh;
    msg->mag_dec = payload->magDec;
    msg->mag_acc = payload->magAcc;

    ubx_nav_pvt_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_pos_llh_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::posllh::NavPosLLHPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav pos llh polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavPosLLH>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->lon = payload->lon;
    msg->lat = payload->lat;
    msg->height = payload->height;
    msg->hmsl = payload->hMSL;
    msg->h_acc = payload->hAcc;
    msg->v_acc = payload->vAcc;

    ubx_nav_pos_llh_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_pos_ecef_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::posecef::NavPosECEFPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav pos ecef polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavPosECEF>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->ecef_x = payload->ecefX;
    msg->ecef_y = payload->ecefY;
    msg->ecef_z = payload->ecefZ;
    msg->p_acc = payload->pAcc;

    ubx_nav_pos_ecef_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_odo_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::odo::NavOdoPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav odo polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavOdo>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->iTOW;
    msg->distance = payload->distance;
    msg->total_distance = payload->totalDistance;
    msg->distance_std = payload->distanceStd;

    ubx_nav_odo_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_orb_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::orb::NavOrbPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav orb polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavOrb>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->itow;
    msg->num_sv = payload->num_sv;

    // Copying sv_info vector
    for (const auto & sv_info_payload : payload->sv_info) {
      ublox_ubx_msgs::msg::OrbSVInfo sv_info_msg;
      sv_info_msg.gnss_id = sv_info_payload.gnss_id;
      sv_info_msg.sv_id = sv_info_payload.sv_id;

      sv_info_msg.sv_flag.health = sv_info_payload.sv_flag.bits.health;
      sv_info_msg.sv_flag.visibility = sv_info_payload.sv_flag.bits.visibility;

      sv_info_msg.eph.eph_usability = sv_info_payload.eph.bits.eph_usability;
      sv_info_msg.eph.eph_source = sv_info_payload.eph.bits.eph_source;

      sv_info_msg.alm.alm_usability = sv_info_payload.alm.bits.alm_usability;
      sv_info_msg.alm.alm_source = sv_info_payload.alm.bits.alm_source;

      sv_info_msg.other_orb.ano_aop_usability = sv_info_payload.other_orb.bits.ano_aop_usability;
      sv_info_msg.other_orb.orb_type = sv_info_payload.other_orb.bits.orb_type;

      msg->sv_info.push_back(sv_info_msg);
    }

    ubx_nav_orb_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_sat_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::sat::NavSatPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav sat polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavSat>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->itow;
    msg->num_svs = payload->num_svs;

    // Copying sv_info vector
    for (const auto & sv_info_payload : payload->sat_data) {
      ublox_ubx_msgs::msg::SatInfo sv_info_msg;
      sv_info_msg.gnss_id = sv_info_payload.gnss_id;
      sv_info_msg.sv_id = sv_info_payload.sv_id;
      sv_info_msg.cno = sv_info_payload.cno;
      sv_info_msg.elev = sv_info_payload.elev;
      sv_info_msg.azim = sv_info_payload.azim;
      sv_info_msg.pr_res = sv_info_payload.pr_res;

      // Expanding flags
      sv_info_msg.flags.quality_ind = sv_info_payload.flags.bits.quality_ind;
      sv_info_msg.flags.sv_used = sv_info_payload.flags.bits.sv_used;
      sv_info_msg.flags.health = sv_info_payload.flags.bits.health;
      sv_info_msg.flags.diff_corr = sv_info_payload.flags.bits.diff_corr;
      sv_info_msg.flags.smoothed = sv_info_payload.flags.bits.smoothed;
      sv_info_msg.flags.orbit_source = sv_info_payload.flags.bits.orbit_source;
      sv_info_msg.flags.eph_avail = sv_info_payload.flags.bits.eph_avail;
      sv_info_msg.flags.alm_avail = sv_info_payload.flags.bits.alm_avail;
      sv_info_msg.flags.ano_avail = sv_info_payload.flags.bits.ano_avail;
      sv_info_msg.flags.aop_avail = sv_info_payload.flags.bits.aop_avail;
      sv_info_msg.flags.sbas_corr_used = sv_info_payload.flags.bits.sbas_corr_used;
      sv_info_msg.flags.rtcm_corr_used = sv_info_payload.flags.bits.rtcm_corr_used;
      sv_info_msg.flags.slas_corr_used = sv_info_payload.flags.bits.slas_corr_used;
      sv_info_msg.flags.spartn_corr_used = sv_info_payload.flags.bits.spartn_corr_used;
      sv_info_msg.flags.pr_corr_used = sv_info_payload.flags.bits.pr_corr_used;
      sv_info_msg.flags.cr_corr_used = sv_info_payload.flags.bits.cr_corr_used;
      sv_info_msg.flags.do_corr_used = sv_info_payload.flags.bits.do_corr_used;
      sv_info_msg.flags.clas_corr_used = sv_info_payload.flags.bits.clas_corr_used;

      msg->sv_info.push_back(sv_info_msg);
    }

    ubx_nav_sat_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_sig_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::sig::NavSigPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav sig polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavSig>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->itow;
    msg->num_sigs = payload->num_sigs;

    // Copying sig_data vector
    for (const auto & sig_data_payload : payload->sig_data) {
      ublox_ubx_msgs::msg::SigData sig_data_msg;
      sig_data_msg.gnss_id = sig_data_payload.gnss_id;
      sig_data_msg.sv_id = sig_data_payload.sv_id;
      sig_data_msg.sig_id = sig_data_payload.sig_id;
      sig_data_msg.freq_id = sig_data_payload.freq_id;
      sig_data_msg.pr_res = sig_data_payload.pr_res;
      sig_data_msg.cno = sig_data_payload.cno;
      sig_data_msg.quality_ind = sig_data_payload.quality_ind;
      sig_data_msg.corr_source = sig_data_payload.corr_source;
      sig_data_msg.iono_model = sig_data_payload.iono_model;

      // Expanding flags
      sig_data_msg.sig_flags.health = sig_data_payload.sig_flags.bits.health;
      sig_data_msg.sig_flags.pr_smoothed = sig_data_payload.sig_flags.bits.pr_smoothed;
      sig_data_msg.sig_flags.pr_used = sig_data_payload.sig_flags.bits.pr_used;
      sig_data_msg.sig_flags.cr_used = sig_data_payload.sig_flags.bits.cr_used;
      sig_data_msg.sig_flags.do_used = sig_data_payload.sig_flags.bits.do_used;
      sig_data_msg.sig_flags.pr_corr_used = sig_data_payload.sig_flags.bits.pr_corr_used;
      sig_data_msg.sig_flags.cr_corr_used = sig_data_payload.sig_flags.bits.cr_corr_used;
      sig_data_msg.sig_flags.do_corr_used = sig_data_payload.sig_flags.bits.do_corr_used;

      msg->sig_data.push_back(sig_data_msg);
    }

    ubx_nav_sig_pub_->publish(*msg);
  }


  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_hp_pos_llh_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::hpposllh::NavHPPosLLHPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav hp pos llh polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavHPPosLLH>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->invalid_lon = payload->flags.bits.invalid_lon;
    msg->invalid_lat = payload->flags.bits.invalid_lat;
    msg->invalid_height = payload->flags.bits.invalid_height;
    msg->invalid_hmsl = payload->flags.bits.invalid_hMSL;
    msg->invalid_lon_hp = payload->flags.bits.invalid_lonHp;
    msg->invalid_lat_hp = payload->flags.bits.invalid_latHp;
    msg->invalid_height_hp = payload->flags.bits.invalid_heightHp;
    msg->invalid_hmsl_hp = payload->flags.bits.invalid_hMSLHp;
    msg->itow = payload->iTOW;
    msg->lon = payload->lon;
    msg->lat = payload->lat;
    msg->height = payload->height;
    msg->hmsl = payload->hMSL;
    msg->lon_hp = payload->lonHp;
    msg->lat_hp = payload->latHp;
    msg->height_hp = payload->heightHp;
    msg->hmsl_hp = payload->hMSLHp;
    msg->h_acc = payload->hAcc;
    msg->v_acc = payload->vAcc;

    ubx_nav_hp_pos_llh_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_hp_pos_ecef_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::hpposecef::NavHPPosECEFPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav hp pos ecef polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavHPPosECEF>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->iTOW;
    msg->ecef_x = payload->ecefX;
    msg->ecef_y = payload->ecefY;
    msg->ecef_z = payload->ecefZ;
    msg->ecef_x_hp = payload->ecefXHp;
    msg->ecef_y_hp = payload->ecefYHp;
    msg->ecef_z_hp = payload->ecefZHp;
    msg->invalid_ecef_x = payload->flags.bits.invalid_ecefX;
    msg->invalid_ecef_y = payload->flags.bits.invalid_ecefY;
    msg->invalid_ecef_z = payload->flags.bits.invalid_ecefZ;
    msg->invalid_ecef_x_hp = payload->flags.bits.invalid_ecefXHp;
    msg->invalid_ecef_y_hp = payload->flags.bits.invalid_ecefYHp;
    msg->invalid_ecef_z_hp = payload->flags.bits.invalid_ecefZHp;

    ubx_nav_hp_pos_ecef_pub_->publish(*msg);
  }


  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_status_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::status::NavStatusPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav status payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavStatus>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->gps_fix.fix_type = payload->gpsFix;
    msg->gps_fix_ok = payload->flags.bits.gpsFixOK;
    msg->diff_soln = payload->flags.bits.diffSoln;
    msg->wkn_set = payload->flags.bits.wknSet;
    msg->tow_set = payload->flags.bits.towSet;
    msg->diff_corr = payload->fixStat.bits.diffCorr;
    msg->carr_soln_valid = payload->fixStat.bits.carrSolnValid;
    msg->map_matching.status = payload->fixStat.bits.mapMatching;
    msg->psm.state = payload->flags2.bits.psmState;
    msg->spoof_det.state = payload->flags2.bits.spoofDetState;
    msg->carr_soln.status = payload->flags2.bits.carrSoln;
    msg->ttff = payload->ttff;
    msg->msss = payload->msss;

    ubx_nav_status_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_svin_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::svin::NavSvinPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav svin payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavSvin>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->itow = payload->iTOW;
    msg->dur = payload->dur;
    msg->mean_x = payload->meanX;
    msg->mean_y = payload->meanY;
    msg->mean_z = payload->meanZ;
    msg->mean_x_hp = payload->meanXHP;
    msg->mean_y_hp = payload->meanYHP;
    msg->mean_z_hp = payload->meanZHP;
    msg->mean_acc = payload->meanAcc;
    msg->obs = payload->obs;
    msg->valid = payload->valid;
    msg->active = payload->active;

    ubx_nav_svin_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_eoe_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::eoe::NavEOEPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav eoe payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavEOE>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;

    ubx_nav_eoe_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_dop_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::dop::NavDOPPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav dop payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavDOP>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->g_dop = payload->gDOP;
    msg->p_dop = payload->pDOP;
    msg->t_dop = payload->tDOP;
    msg->v_dop = payload->vDOP;
    msg->h_dop = payload->hDOP;
    msg->n_dop = payload->nDOP;
    msg->e_dop = payload->eDOP;

    ubx_nav_dop_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_cov_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::cov::NavCovPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav cov payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavCov>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->version = payload->version;
    msg->pos_cor_valid = static_cast<bool>(payload->posCorValid);
    msg->vel_cor_valid = static_cast<bool>(payload->velCorValid);
    msg->pos_cov_nn = payload->posCovNN;
    msg->pos_cov_ne = payload->posCovNE;
    msg->pos_cov_nd = payload->posCovND;
    msg->pos_cov_ee = payload->posCovEE;
    msg->pos_cov_ed = payload->posCovED;
    msg->pos_cov_dd = payload->posCovDD;
    msg->vel_cov_nn = payload->velCovNN;
    msg->vel_cov_ne = payload->velCovNE;
    msg->vel_cov_nd = payload->velCovND;
    msg->vel_cov_ee = payload->velCovEE;
    msg->vel_cov_ed = payload->velCovED;
    msg->vel_cov_dd = payload->velCovDD;

    ubx_nav_cov_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_nav_clock_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::nav::clock::NavClockPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x nav clock payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXNavClock>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->clk_b = payload->clkB;
    msg->clk_d = payload->clkD;
    msg->t_acc = payload->tAcc;
    msg->f_acc = payload->fAcc;

    ubx_nav_clock_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_cor_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::cor::RxmCorPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm cor polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    // Create message
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmCor>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate main fields
    msg->version = payload->version;
    msg->ebno = payload->ebno;
    msg->msg_type = payload->msgType;
    msg->msg_sub_type = payload->msgSubType;

    // Populate status_info
    ublox_ubx_msgs::msg::CorStatusInfo status_info_msg;
    status_info_msg.protocol = payload->protocol;
    status_info_msg.err_status = payload->errStatus;
    status_info_msg.msg_used = payload->msgUsed;
    status_info_msg.correction_id = payload->correctionId;
    status_info_msg.msg_type_valid = payload->msgTypeValid;
    status_info_msg.msg_sub_type_valid = payload->msgSubTypeValid;
    status_info_msg.msg_input_handle = payload->msgInputHandle;
    status_info_msg.msg_encrypted = payload->msgEncrypted;
    status_info_msg.msg_decrypted = payload->msgDecrypted;
    msg->status_info = status_info_msg;

    // Publish the message
    ubx_rxm_cor_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_rtcm_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::rtcm::RxmRTCMPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm rtcm polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmRTCM>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->version = payload->version;
    msg->crc_failed = payload->flags.bits.crcFailed;
    msg->msg_used = payload->flags.bits.msgUsed;
    msg->sub_type = payload->subType;
    msg->ref_station = payload->refStation;
    msg->msg_type = payload->msgType;

    ubx_rxm_rtcm_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_measx_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::measx::RxmMeasxPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm measx polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmMeasx>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate the main fields
    msg->version = payload->version;
    msg->gps_tow = payload->gps_tow;
    msg->glo_tow = payload->glo_tow;
    msg->bds_tow = payload->bds_tow;
    msg->qzss_tow = payload->qzss_tow;
    msg->gps_tow_acc = payload->gps_tow_acc;
    msg->glo_tow_acc = payload->glo_tow_acc;
    msg->bds_tow_acc = payload->bds_tow_acc;
    msg->qzss_tow_acc = payload->qzss_tow_acc;
    msg->num_sv = payload->num_sv;
    msg->flags = static_cast<uint8_t>(payload->tow_set);

    // Populate the repeated satellite data
    for (const auto & sv_data_payload : payload->sv_data) {
      ublox_ubx_msgs::msg::MeasxData sv_data_msg;
      sv_data_msg.gnss_id = sv_data_payload.gnss_id;
      sv_data_msg.sv_id = sv_data_payload.sv_id;
      sv_data_msg.c_no = sv_data_payload.c_no;
      sv_data_msg.mpath_indic = sv_data_payload.mpath_indic;
      sv_data_msg.doppler_ms = sv_data_payload.doppler_ms;
      sv_data_msg.doppler_hz = sv_data_payload.doppler_hz;
      sv_data_msg.whole_chips = sv_data_payload.whole_chips;
      sv_data_msg.frac_chips = sv_data_payload.frac_chips;
      sv_data_msg.code_phase = sv_data_payload.code_phase;
      sv_data_msg.int_code_phase = sv_data_payload.int_code_phase;
      sv_data_msg.pseu_range_rms_err = sv_data_payload.pseu_range_rms_err;

      msg->sv_data.push_back(sv_data_msg);
    }

    // Publish the message
    ubx_rxm_measx_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_rawx_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::rawx::RxmRawxPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm rawx polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmRawx>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate the main fields
    msg->rcv_tow = payload->rcv_tow;
    msg->week = payload->week;
    msg->leap_s = payload->leap_s;
    msg->num_meas = payload->num_meas;
    msg->rec_stat.leap_sec = payload->rec_stat.bits.leap_sec;
    msg->rec_stat.clk_reset = payload->rec_stat.bits.clk_reset;
    msg->version = payload->version;

    // Populate the repeated measurement data
    for (const auto & meas_data_payload : payload->meas_data) {
      ublox_ubx_msgs::msg::RawxData meas_data_msg;
      meas_data_msg.pr_mes = meas_data_payload.pr_mes;
      meas_data_msg.cp_mes = meas_data_payload.cp_mes;
      meas_data_msg.do_mes = meas_data_payload.do_mes;
      meas_data_msg.gnss_id = meas_data_payload.gnss_id;
      meas_data_msg.sv_id = meas_data_payload.sv_id;
      meas_data_msg.sig_id = meas_data_payload.sig_id;
      meas_data_msg.freq_id = meas_data_payload.freq_id;
      meas_data_msg.locktime = meas_data_payload.locktime;
      meas_data_msg.c_no = meas_data_payload.cno;
      meas_data_msg.pr_stdev = meas_data_payload.pr_stdev;
      meas_data_msg.cp_stdev = meas_data_payload.cp_stdev;
      meas_data_msg.do_stdev = meas_data_payload.do_stdev;
      meas_data_msg.trk_stat.pr_valid = meas_data_payload.trk_stat.bits.pr_valid;
      meas_data_msg.trk_stat.cp_valid = meas_data_payload.trk_stat.bits.cp_valid;
      meas_data_msg.trk_stat.half_cyc = meas_data_payload.trk_stat.bits.half_cyc;
      meas_data_msg.trk_stat.sub_half_cyc = meas_data_payload.trk_stat.bits.sub_half_cyc;

      msg->rawx_data.push_back(meas_data_msg);
    }

    // Publish the message
    ubx_rxm_rawx_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_spartn_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::spartn::RxmSpartnPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm spartn polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    // Create message
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmSpartn>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate fields
    msg->version = payload->version;
    msg->msg_used = payload->msg_used;
    msg->sub_type = payload->subType;
    msg->msg_type = payload->msgType;

    // Publish the message
    ubx_rxm_spartn_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_rxm_spartnkey_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::rxm::spartnkey::RxmSpartnKeyPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x rxm spartnkey polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    // Create message
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXRxmSpartnKey>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate fields
    msg->version = payload->version;
    msg->num_keys = payload->numKeys;

    // Populate the repeated key info data
    for (const auto & key_info_payload : payload->keyInfos) {
      ublox_ubx_msgs::msg::SpartnKeyInfo key_info_msg;
      key_info_msg.reserved1 = key_info_payload.reserved1;
      key_info_msg.key_length_bytes = key_info_payload.keyLengthBytes;
      key_info_msg.valid_from_wno = key_info_payload.validFromWno;
      key_info_msg.valid_from_tow = key_info_payload.validFromTow;
      msg->key_info.push_back(key_info_msg);
    }

    // Concatenate the key data from each KeyPayload into key_payload
    msg->key_payload.clear();  // Ensure the key_payload array is empty before populating
    for (const auto & key_payload : payload->keyPayloads) {
      // Append the entire key vector from KeyPayload to msg->key_payload
      msg->key_payload.insert(
        msg->key_payload.end(), key_payload.key.begin(),
        key_payload.key.end());
    }

    // Publish the message
    ubx_rxm_spartnkey_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_esf_status_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::esf::status::ESFStatusPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x esf status polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXEsfStatus>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;
    msg->itow = payload->iTOW;
    msg->version = payload->version;
    msg->wt_init_status = payload->initStatus1.bits.wtInitStatus;
    msg->mnt_alg_status = payload->initStatus1.bits.mntAlgStatus;
    msg->ins_init_status = payload->initStatus1.bits.insInitStatus;
    msg->imu_init_status = payload->initStatus2.bits.imuInitStatus;
    msg->fusion_mode = payload->fusionMode;
    msg->num_sens = payload->numSens;

    for (int i = 0; i < payload->numSens; i++) {
      auto s_msg = std::make_unique<ublox_ubx_msgs::msg::ESFSensorStatus>();
      auto sensor = payload->sensorStatuses[i];
      s_msg->sensor_data_type = sensor.bits.sensStatus1.bits.type;
      s_msg->used = sensor.bits.sensStatus1.bits.used;
      s_msg->ready = sensor.bits.sensStatus1.bits.ready;
      s_msg->calib_status = sensor.bits.sensStatus2.bits.calibStatus;
      s_msg->time_status = sensor.bits.sensStatus2.bits.timeStatus;
      s_msg->freq = sensor.bits.freq;
      s_msg->fault_bad_meas = sensor.bits.faults.bits.badMeas;
      s_msg->fault_bad_ttag = sensor.bits.faults.bits.badTtag;
      s_msg->fault_missing_meas = sensor.bits.faults.bits.missingMeas;
      s_msg->fault_noisy_meas = sensor.bits.faults.bits.noisyMeas;

      msg->sensor_statuses.push_back(*s_msg);
    }

    ubx_esf_status_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_esf_meas_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::esf::meas::ESFMeasPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x esf status polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());
    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXEsfMeas>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    msg->time_tag = payload->timeTag;
    msg->time_mark_sent = payload->flags.bits.timeMarkSent;
    msg->time_mark_edge = payload->flags.bits.timeMarkEdge;
    msg->calib_ttag_valid = payload->flags.bits.calibTtagValid;
    msg->num_meas = payload->flags.bits.numMeas;
    msg->id = payload->id;

    uint numMeas = static_cast<uint>(payload->flags.bits.numMeas);
    for (uint i = 0; i < numMeas; i++) {
      auto data = payload->datum[i];
      auto data_msg = std::make_unique<ublox_ubx_msgs::msg::ESFMeasDataItem>();
      data_msg->data_field = data.bits.dataField;
      data_msg->data_type = data.bits.dataType;
      msg->data.push_back(*data_msg);
    }

    if (msg->calib_ttag_valid) {
      msg->calib_ttag = payload->calibTtags;
    }
    ubx_esf_meas_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_sec_sig_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::sec::sig::SecSigPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x sec sig polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXSecSig>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate the main fields
    msg->version = payload->version;


    if (msg->version >= 2) {
      // Information related to jamming/interference
      msg->jam_det_enabled = payload->sec_sig_flags.bits.jam_det_enabled;
      msg->jamming_state = payload->sec_sig_flags.bits.jam_state;

      // Information related to GNSS spoofing
      msg->spf_det_enabled = payload->sec_sig_flags.bits.spf_det_enabled;
      msg->spoofing_state = payload->sec_sig_flags.bits.spf_state;

      msg->jam_num_cent_freqs = payload->jam_num_cent_freqs;
      for (const auto & jam_state : payload->jam_state_cent_freqs) {
        ublox_ubx_msgs::msg::JamStateCentFreq jam_state_msg;
        jam_state_msg.cent_freq = jam_state.bits.cent_freq;
        jam_state_msg.jammed = jam_state.bits.jammed;
        msg->jam_state_cent_freqs.push_back(jam_state_msg);
      }
    } else {
      // Information related to jamming/interference
      msg->jam_det_enabled = payload->jam_flags.bits.jam_det_enabled;
      msg->jamming_state = payload->jam_flags.bits.jamming_state;

      // Information related to GNSS spoofing
      msg->spf_det_enabled = payload->spf_flags.bits.spf_det_enabled;
      msg->spoofing_state = payload->spf_flags.bits.spoofing_state;
    }
    // Publish the message
    ubx_sec_sig_pub_->publish(*msg);
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ubx_sec_siglog_pub(
    ubx_queue_frame_t * f,
    std::shared_ptr<ubx::sec::siglog::SecSigLogPayload> payload)
  {
    RCLCPP_DEBUG(
      get_logger(), "ubx class: 0x%02x id: 0x%02x sec siglog polled payload - %s",
      f->ubx_frame->msg_class, f->ubx_frame->msg_id,
      payload->to_string().c_str());

    auto msg = std::make_unique<ublox_ubx_msgs::msg::UBXSecSigLog>();

    // Populate the header
    msg->header.frame_id = frame_id_;
    msg->header.stamp = f->ts;

    // Populate the main fields
    msg->version = payload->version;
    msg->num_events = payload->num_events;

    // Populate the repeated events data
    for (const auto & event_payload : payload->events) {
      ublox_ubx_msgs::msg::SigLogEvent event_msg;
      event_msg.time_elapsed = event_payload.time_elapsed;
      event_msg.detection_type = event_payload.detection_type;
      event_msg.event_type = event_payload.event_type;

      msg->events.push_back(event_msg);
    }

    // Publish the message
    ubx_sec_sig_log_pub_->publish(*msg);
  }


  UBLOX_DGNSS_NODE_LOCAL
  void ublox_init_all_cfg_items_async()
  {
    ubx_cfg_->cfg_val_set_cfgdata_clear();
    ubx_cfg_->cfg_val_set_layer_ram(true);
    ubx_cfg_->cfg_val_set_transaction(0);     // transactionless

    std::string item_list;
    size_t i = 0;
    size_t n = 10;     // every n output a request
    for (auto ci : ubx::cfg::ubxKeyCfgItemMap) {
      auto ubx_ci = ci.second;
      ubx::value_t value {0x0000000000000000};
      RCLCPP_DEBUG(get_logger(), "init %lu cfg param: %s", i, ubx_ci.ubx_config_item);
      set_or_declare_ubx_cfg_param(ubx_ci, value, true);

      auto p_state = cfg_param_cache_map_[ubx_ci.ubx_config_item];
      if (p_state.status == PARAM_USER) {
        auto result = cfg_val_set_from_ubx_ci_p_state(ubx_ci, p_state);
        if (!result.successful) {
          RCLCPP_ERROR(get_logger(), "initial cfg val set exception: %s", result.reason.c_str());
        }
      }

      // every n send a poll request and reset the keys
      if (++i % n == 0) {
        if (ubx_cfg_->cfg_val_set_cfgdata_size() > 0) {
          RCLCPP_DEBUG(get_logger(), "cfg_val_set_poll_async ... %s", item_list.c_str());
          item_list = "";
          ubx_cfg_->cfg_val_set_poll_async();
          ubx_cfg_->cfg_val_set_cfgdata_clear();
        }
      }
    }
    // send the final sets
    if (ubx_cfg_->cfg_val_set_cfgdata_size() > 0) {
      RCLCPP_DEBUG(get_logger(), "cfg_val_set_poll_async ... %s", item_list.c_str());
      ubx_cfg_->cfg_val_set_poll_async();
      ubx_cfg_->cfg_val_set_cfgdata_clear();
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ublox_val_get_all_cfg_items_async()
  {
    ubx_cfg_->cfg_val_get_keys_clear();
    ubx_cfg_->cfg_set_val_get_layer_ram();

    std::string item_list;
    size_t i = 0;
    size_t n = 10;     // every n output a request
    for (auto ci : ubx::cfg::ubxKeyCfgItemMap) {
      auto ubx_key_id = ci.first;
      auto ubx_ci = ci.second;
      ubx_cfg_->cfg_val_get_key_append(ubx_key_id);

      item_list += ubx_ci.ubx_config_item;
      item_list += " ";

      RCLCPP_DEBUG(get_logger(), "load %lu cfg param: %s", i, ubx_ci.ubx_config_item);

      if (++i % n == 0) {
        RCLCPP_DEBUG(get_logger(), "cfg_val_get_poll_async ... %s", item_list.c_str());
        item_list = "";
        ubx_cfg_->cfg_val_get_poll_async();
        ubx_cfg_->cfg_val_get_keys_clear();
      }
    }
    if (ubx_cfg_->cfg_val_get_keys_size() > 0) {
      RCLCPP_DEBUG(get_logger(), "cfg_val_get_poll_async ... %s", item_list.c_str());
      ubx_cfg_->cfg_val_get_poll_async_all_layers();
      ubx_cfg_->cfg_val_get_keys_clear();
    }
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ublox_val_set_all_cfg_items_async()
  {
    ubx_cfg_->cfg_val_set_cfgdata_clear();
    ubx_cfg_->cfg_val_set_layer_ram(true);
    ubx_cfg_->cfg_val_set_transaction(1);

    std::string item_list;
    size_t i = 0;
    bool trans_start = false;
    size_t n = 1;     // every n output a request
    for (auto ci : ubx::cfg::ubxKeyCfgItemMap) {
      trans_start = true;
      auto ubx_ci = ci.second;

      auto p_state = cfg_param_cache_map_[ubx_ci.ubx_config_item];
      auto result = cfg_val_set_from_ubx_ci_p_state(ubx_ci, p_state);
      if (!result.successful) {
        RCLCPP_ERROR(get_logger(), "all cfg val set exception: %s", result.reason.c_str());
      }

      item_list += ubx_ci.ubx_config_item;
      item_list += " ";

      RCLCPP_DEBUG(get_logger(), "set %lu cfg param: %s", i, ubx_ci.ubx_config_item);

      if (++i % n == 0) {
        RCLCPP_DEBUG(get_logger(), "cfg_val_set_poll_async ... %s", item_list.c_str());
        item_list = "";
        ubx_cfg_->cfg_val_set_poll_async();
        ubx_cfg_->cfg_val_set_cfgdata_clear();
        ubx_cfg_->cfg_val_set_transaction(2);
      }
    }

    if (trans_start) {
      RCLCPP_DEBUG(
        get_logger(), "cfg_val_set_poll_async ... end transaction ... %s",
        item_list.c_str());
      ubx_cfg_->cfg_val_set_transaction(3);
      ubx_cfg_->cfg_val_set_poll_async();
      ubx_cfg_->cfg_val_set_cfgdata_clear();
    }
    ubx_cfg_->cfg_val_set_transaction(0);     // make sure its tranaction less
  }

  UBLOX_DGNSS_NODE_LOCAL
  void ublox_dgnss_init_async()
  {
    RCLCPP_DEBUG(get_logger(), "ubx_mon_ver poll_async ...");
    ubx_mon_->ver()->poll_async();
    RCLCPP_DEBUG(get_logger(), "ubx_sec_uniqid poll_async ...");
    ubx_sec_->uniqid()->poll_async();

    // RCLCPP_INFO(get_logger(), "cfg_val_get_poll_async ...");
    // ubx_cfg_->cfg_val_get_keys_clear();
    // ubx_cfg_->cfg_set_val_get_layer_ram();
    // ubx_cfg_->cfg_val_get_key_append(ubx::cfg::CFG_USBINPROT_NMEA);
    // ubx_cfg_->cfg_val_get_key_append(ubx::cfg::CFG_USBOUTPROT_NMEA);
    // ubx_cfg_->cfg_val_get_poll_async();

    // RCLCPP_INFO(get_logger(), "cfg_val_set_poll_async ...");
    // disable NMEA protocol on USB
    // ubx_cfg_->cfg_val_set_cfgdata_clear();
    // ubx_cfg_->cfg_val_set_layer_ram(true);
    // ubx_cfg_->cfg_val_set_transaction(0); // 0 = no transaction
    // // ubx_cfg_->cfg_val_set_key_append(ubx::cfg::CFG_USBINPROT_NMEA, false);
    // // ubx_cfg_->cfg_val_set_key_append(ubx::cfg::CFG_USBOUTPROT_NMEA, false);
    // // ubx_cfg_->cfg_val_set_key_append(ubx::cfg::CFG_USBINPROT_NMEA, true);
    // // ubx_cfg_->cfg_val_set_key_append(ubx::cfg::CFG_USBOUTPROT_NMEA, true);
    // ubx_cfg_->cfg_val_set_poll_async();

    RCLCPP_DEBUG(get_logger(), "ublox_init_all_cfg_items_async() ...");
    ublox_init_all_cfg_items_async();
    RCLCPP_DEBUG(get_logger(), "ublox_val_get_all_cfg_items_async() ...");
    ublox_val_get_all_cfg_items_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_clock poll_async ...");
    // ubx_nav_->clock()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_dop poll_async ...");
    // ubx_nav_->dop()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_posecef poll_async ...");
    // ubx_nav_->posecef()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_posllh poll_async ...");
    // ubx_nav_->posllh()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_hpposecef poll_async ...");
    // ubx_nav_->hpposecef()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_hpposllh poll_async ...");
    // ubx_nav_->hpposllh()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_odo poll_async ...");
    // ubx_nav_->odo()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_pvt poll_async ...");
    // ubx_nav_->pvt()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_status poll_async ...");
    // ubx_nav_->status()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_relposned poll_async ...");
    // ubx_nav_->relposned()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_timeutc poll_async ...");
    // ubx_nav_->timeutc()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_velecef poll_async ...");
    // ubx_nav_->velecef()->poll_async();
    // RCLCPP_INFO(get_logger(), "ubx_nav_velned poll_async ...");
    // ubx_nav_->velned()->poll_async();
  }
};
}  // namespace ublox_dgnss

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_dgnss::UbloxDGNSSNode)
