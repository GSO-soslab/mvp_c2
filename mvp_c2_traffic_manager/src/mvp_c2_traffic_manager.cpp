/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller, jason_miller@uri.edu
    Author: Lin Zhao, linzhao@uri.edu
    Year: 2023-2024

    Copyright (C) 2023-2024 Smart Ocean Systems Laboratory
*/

#include "mvp_c2_traffic_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using goby::glog;
using goby::util::as;

using namespace std::chrono_literals;

YAML::Node merge_maps(const YAML::Node& base, const YAML::Node& override_) {
  YAML::Node out(YAML::NodeType::Map);

  if (base && base.IsMap()) {
    for (const auto& kv : base) {                           // const auto&
      out[kv.first.Scalar()] = kv.second;
    }
  }

  if (override_ && override_["<<"]) {
    const YAML::Node m = override_["<<"];
    if (m.IsMap()) {
      for (const auto& kv : m) out[kv.first.Scalar()] = kv.second;
    } else if (m.IsSequence()) {
      for (const auto& elem : m) if (elem.IsMap())
        for (const auto& kv : elem) out[kv.first.Scalar()] = kv.second;
    }
  }

  if (override_ && override_.IsMap()) {
    for (const auto& kv : override_) {
      const std::string k = kv.first.Scalar();
      if (k == "<<") continue;
      out[k] = kv.second;
    }
  }
  return out;
}


MvpC2TrafficManager::MvpC2TrafficManager(std::string name) : Node(name)
{
    // ===================================================================== //
    // setup param
    // ===================================================================== //
    loadConfig();

    // ===================================================================== //
    // ROS2 setup
    // ===================================================================== //
    dccl_tx_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(comm_type_ + "/tx_request", 10,
        std::bind(&MvpC2TrafficManager::onDcclRx, this, std::placeholders::_1));
    // ===================================================================== //
    // setup main thread
    // ===================================================================== //
    loop_worker_ = std::thread([this] { loop(); });
    loop_worker_.detach();  

}

MvpC2TrafficManager::~MvpC2TrafficManager()
{
    rclcpp::shutdown();
}

void MvpC2TrafficManager::loop()
{
    // loop at 10Hz
    rclcpp::Rate rate(10); 

    while (rclcpp::ok())
    {
        buffer_.expire();
        mac_.do_work();
        rate.sleep();
    }
}


void MvpC2TrafficManager::loadConfig()
{
    this->declare_parameter<std::string>("type", "");
    this->get_parameter("type", comm_type_);

    RCLCPP_INFO(get_logger(),"%s MvpC2TraffficManager started!", comm_type_.c_str());

    // Load and parse the message config file
    std::string msg_file = ament_index_cpp::get_package_share_directory("mvp_c2_traffic_manager") +
                        "/config/traffic_manager.yaml";

    // Load and parse the corresponding tdma config file
    std::string tdma_file = ament_index_cpp::get_package_share_directory("mvp_c2_traffic_manager") +
                        "/config/tdma.yaml";


    RCLCPP_INFO(this->get_logger(), "Loading config files: %s & %s", msg_file.c_str(), tdma_file.c_str());
    //load the config file
    YAML::Node msg_root = YAML::LoadFile(msg_file);
    YAML::Node tdma_root = YAML::LoadFile(tdma_file);

    // parse the message config file
    try
    {
        YAML::Node node = msg_root["traffic_manager"][comm_type_];

        RCLCPP_INFO(this->get_logger(),"%s:", comm_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "  local_address: %d", node["local_address"].as<int>());
        
        config_.local_address = node["local_address"].as<int>();

        for (const auto & msg : node["messages"]) {
            const std::string message = msg.first.as<std::string>();
            config_.msg[message].ack = msg.second["ack"].as<bool>(false);
            config_.msg[message].blackout_time = msg.second["blackout_time"].as<int>(0);
            config_.msg[message].max_queue = msg.second["max_queue"].as<int>(0);
            config_.msg[message].newest_first = msg.second["newest_first"].as<bool>(true);
            config_.msg[message].ttl = msg.second["ttl"].as<int>(1800);
            config_.msg[message].value_base = msg.second["value_base"].as<int>(1);

            RCLCPP_INFO(this->get_logger(), "  Message: %s", message.c_str());
            RCLCPP_INFO(this->get_logger(), "    ack: %s", config_.msg[message].ack ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "    blackout_time: %d", config_.msg[message].blackout_time);
            RCLCPP_INFO(this->get_logger(), "    max_queue: %d", config_.msg[message].max_queue);
            RCLCPP_INFO(this->get_logger(), "    newest_first: %s", config_.msg[message].newest_first ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "    ttl: %d", config_.msg[message].ttl);
            RCLCPP_INFO(this->get_logger(), "    value_base: %d", config_.msg[message].value_base);
        }
    }
    catch (const YAML::Exception & e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
    }

    // parse the tdma config file
    std::vector<TdmaSlot> schedule;    
    try
    {
        YAML::Node tdma     = tdma_root["tdma"];
        YAML::Node defaults = tdma["default"];
        YAML::Node slots    = tdma[comm_type_]["slots"];

        for (const auto& item : slots) {
            YAML::Node merged = merge_maps(defaults, item);
            TdmaSlot s{
            merged["source"].as<int>(),
            merged["destination"].as<int>(),
            merged["slot_time"].as<int>(),
            merged["max_frame_bytes"].as<int>(),
            merged["max_num_frames"].as<int>(),
            merged["rate"].as<int>()
            };
            schedule.push_back(s);
        }
    } 
    catch (const YAML::Exception & e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s TDMA config file: %s", comm_type_.c_str(), e.what());
    }

    goby::acomms::protobuf::MACConfig cfg;
    cfg.set_modem_id(config_.local_address);
    cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    goby::acomms::connect(&mac_.signal_initiate_transmission, this, &MvpC2TrafficManager::initTransmission);
    
    for (const auto& s : schedule) {
        std::cout << comm_type_
                    << " TDMA slot: src=" << s.source
                    << " dst=" << s.destination
                    << " time=" << s.slot_time
                    << " bytes=" << s.max_frame_bytes
                    << " frames=" << s.max_num_frames
                    << " rate=" << s.rate << "\n";


        goby::acomms::protobuf::ModemTransmission* slot = cfg.add_slot();
        slot->set_src(s.source);
        slot->set_dest(s.destination);
        slot->set_rate(s.rate);
        slot->set_type(goby::acomms::protobuf::ModemTransmission::DATA);
        slot->set_slot_seconds(s.slot_time);
        slot->set_max_frame_bytes(s.max_frame_bytes);
        slot->set_max_num_frames(s.max_num_frames);
        mac_.push_back(*slot);

        if(s.source == config_.local_address) {
            // Add a dynamic buffer for each remote address we transmit to
            goby::acomms::protobuf::DynamicBufferConfig buffer_cfg;

            for (const auto & msg : config_.msg)
            {
                buffer_cfg.set_ack_required(msg.second.ack);
                buffer_cfg.set_blackout_time(msg.second.blackout_time);
                buffer_cfg.set_max_queue(msg.second.max_queue);
                buffer_cfg.set_newest_first(msg.second.newest_first);
                buffer_cfg.set_ttl(msg.second.ttl);
                buffer_cfg.set_value_base(msg.second.value_base);

                buffer_.create(s.destination, msg.first, buffer_cfg);

                buffer_cfg.Clear();

            }
        }

    }

    mac_.startup(cfg);

}

void MvpC2TrafficManager::onDcclRx(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    std::cout << "received dccl message of size: " << msg->data.size() << std::endl;
}


void MvpC2TrafficManager::initTransmission(const goby::acomms::protobuf::ModemTransmission& msg)
{
    std::cout << "starting transmission with these values: " << msg.ShortDebugString() << std::endl;

    try
    {
        auto out = buffer_.top(msg.dest());
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "No %s data to send from %d to %d", comm_type_.c_str(), msg.src(), msg.dest());
    }
    
    
       
}