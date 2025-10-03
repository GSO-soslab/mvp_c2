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
    tx_req_sub_ = this->create_subscription<mvp_c2_msgs::msg::DcclMsg>("mvp_c2/" + config_.comm_type + "/tx_request", 10,
        std::bind(&MvpC2TrafficManager::onTxRequest, this, std::placeholders::_1));

    modem_tx_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("mvp_c2/" + config_.comm_type + "/tx", 10);
    
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
    this->get_parameter("type", config_.comm_type);

    std::string filename;
    this->declare_parameter<std::string>("config", "");
    this->get_parameter("config", filename);

    RCLCPP_INFO(get_logger(),"%s MvpC2TraffficManager started!", config_.comm_type.c_str());

    // Load and parse the message config file
    std::string msg_file = ament_index_cpp::get_package_share_directory("mvp_c2_traffic_manager") +
                        "/config/" + filename + "_traffic_manager.yaml";

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
        YAML::Node node = msg_root["traffic_manager"][config_.comm_type];

        RCLCPP_INFO(this->get_logger(),"%s:", config_.comm_type.c_str());
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
        YAML::Node slots    = tdma[config_.comm_type]["slots"];

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
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s TDMA config file: %s", config_.comm_type.c_str(), e.what());
    }

    goby::acomms::protobuf::MACConfig cfg;
    cfg.set_modem_id(config_.local_address);
    cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    goby::acomms::connect(&mac_.signal_initiate_transmission, this, &MvpC2TrafficManager::initTransmission);
    
    for (const auto& s : schedule) {
        std::cout << config_.comm_type
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

void MvpC2TrafficManager::onTxRequest(const mvp_c2_msgs::msg::DcclMsg::SharedPtr msg)
{
    std::string debug(msg->data.begin(), msg->data.end());
    // std::cout << "Received dccl message:" <<std::endl << "Type: " << msg->type << std::endl << "Size: " << msg->data.size() << std::endl << "Data: " << goby::util::hex_encode(debug) << std::endl <<std::endl;
    buffer_.push({msg->dest, msg->type, goby::time::SteadyClock::now(), msg->data});
}


void MvpC2TrafficManager::initTransmission(const goby::acomms::protobuf::ModemTransmission& msg)
{
    std::cout << "starting transmission with these values: " << msg.ShortDebugString() << std::endl;
    std::vector<uint8_t> frame;

    while(frame.size() < msg.max_frame_bytes())
    {
        try
        {
            auto out = buffer_.top(msg.dest(), msg.max_frame_bytes() - frame.size());
            frame.insert(frame.end(), out.data.begin(), out.data.end());
            buffer_.erase(out);

            std::string debug(out.data.begin(), out.data.end());
            RCLCPP_INFO(this->get_logger(), "Adding %s message of size: %ld to frame with total size: %ld with data: %s", 
                out.subbuffer_id.c_str(), out.data.size(), frame.size(), goby::util::hex_encode(debug).c_str());
        }
        catch (goby::acomms::DynamicBufferNoDataException &)
        {
            if(frame.size() == 0)
            {
                RCLCPP_INFO(this->get_logger(), "No %s data to send from %d to %d", config_.comm_type.c_str(), msg.src(), msg.dest());
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Frame filled with %ld bytes", frame.size());
                std_msgs::msg::ByteMultiArray transmit;
                transmit.data = frame;
                modem_tx_pub_->publish(transmit);

                return;
            }
        }
    }
}