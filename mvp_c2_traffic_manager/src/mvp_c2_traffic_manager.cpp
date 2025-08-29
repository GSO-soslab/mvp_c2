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

using goby::glog;
using goby::util::as;

using namespace std::chrono_literals;

MvpC2TrafficManager::MvpC2TrafficManager(std::string name) : Node(name)
{
    RCLCPP_INFO(get_logger(), "MvpC2TraffficManager started!");

    // ===================================================================== //
    // setup param
    // ===================================================================== //

    // parseGobyParams();
    loadConfig();

    // ===================================================================== //
    // ROS2 setup
    // ===================================================================== //

    
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

        rate.sleep();
    }
}

void MvpC2TrafficManager::loadConfig() {
  this->declare_parameter<std::vector<std::string>>("load_config", std::vector<std::string>{});
  std::vector<std::string> configs;
  this->get_parameter("load_config", configs);

  for (const auto & type : configs) {

    RCLCPP_INFO(this->get_logger(), "Loading config for '%s'", type.c_str());

    // prefer int64_t declarations for ints
    this->declare_parameter<int64_t>(type + ".mac.local_address", 1);
    this->get_parameter(type + ".mac.local_address", config_[type].mac.local_address);

    this->declare_parameter<int64_t>(type + ".mac.local_slot_time", 30);
    this->get_parameter(type + ".mac.local_slot_time", config_[type].mac.local_slot_time);

    this->declare_parameter<int64_t>(type + ".mac.max_frame_bytes", 100);
    this->get_parameter(type + ".mac.max_frame_bytes", config_[type].mac.max_frame_bytes);

    RCLCPP_INFO(this->get_logger(), "  local_address: %d", config_[type].mac.local_address);
    RCLCPP_INFO(this->get_logger(), "  local_slot_time: %d", config_[type].mac.local_slot_time);
    RCLCPP_INFO(this->get_logger(), "  max_frame_bytes: %d", config_[type].mac.max_frame_bytes);

    RCLCPP_INFO(this->get_logger(), "  Remotes:");

    // ----- remotes dict: <type>.mac.remotes.<key> = <value> -----
    const std::string prefix = type + ".mac.remotes";
    auto pif = this->get_node_parameters_interface();
    const auto & overrides = pif->get_parameter_overrides(); // name -> ParameterValue

    // declare each remote so list/get will see them
    for (const auto & kv : overrides) {
    if (kv.first.rfind(prefix, 0) == 0 && !this->has_parameter(kv.first)) {
        this->declare_parameter<int64_t>(kv.first, 0);
    }
    }

    auto listed = pif->list_parameters({prefix}, /*depth=*/1);
    auto params = pif->get_parameters(listed.names);  // <-- fixed overload

    std::map<int,int> remotes;
    const std::string dot = prefix + ".";
    for (const auto & p : params) {
      const std::string & full = p.get_name();            // e.g. "acomms.mac.remotes.1"
      if (full.rfind(dot, 0) != 0) continue;
      const std::string suffix = full.substr(dot.size()); // "1"
      try {
        int key = std::stoi(suffix);
        int val = static_cast<int>(p.as_int());           // int64 -> int
        remotes.emplace(key, val);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Skip '%s': %s", full.c_str(), e.what());
      }
    }
    config_[type].mac.remotes = std::move(remotes);

    for(const auto & r : config_[type].mac.remotes) {
      RCLCPP_INFO(this->get_logger(), "    address %d: slot time %d", r.first, r.second);
    }

    // ----- messages -----
    RCLCPP_INFO(this->get_logger(), "  Messages:");
    this->declare_parameter<std::vector<std::string>>(type + ".messages.load",
                                                      std::vector<std::string>{});
    std::vector<std::string> messages;
    this->get_parameter(type + ".messages.load", messages);

    for (const auto & message : messages) {
        const std::string base = type + ".messages." + message + ".";
        this->declare_parameter<bool>(base + "ack", false);
        this->get_parameter(base + "ack", config_[type].msg[message].ack);

        this->declare_parameter<int64_t>(base + "blackout_time", 0);
        int64_t blackout64 = 0; this->get_parameter(base + "blackout_time", blackout64);
        config_[type].msg[message].blackout_time = static_cast<int>(blackout64);

        this->declare_parameter<int64_t>(base + "max_queue", 0);
        int64_t maxq64 = 0; this->get_parameter(base + "max_queue", maxq64);
        config_[type].msg[message].max_queue = static_cast<int>(maxq64);

        this->declare_parameter<bool>(base + "newest_first", true);
        this->get_parameter(base + "newest_first", config_[type].msg[message].newest_first);

        this->declare_parameter<int64_t>(base + "ttl", 1800);
        int64_t ttl64 = 1800; this->get_parameter(base + "ttl", ttl64);
        config_[type].msg[message].ttl = static_cast<int>(ttl64);

        this->declare_parameter<int64_t>(base + "value_base", 1);
        int64_t vb64 = 1; this->get_parameter(base + "value_base", vb64);
        config_[type].msg[message].value_base = static_cast<int>(vb64);

        RCLCPP_INFO(this->get_logger(), "    %s:", message.c_str());
        RCLCPP_INFO(this->get_logger(), "      ack: %s", config_[type].msg[message].ack ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "      blackout_time: %d", config_[type].msg[message].blackout_time);
        RCLCPP_INFO(this->get_logger(), "      max_queue: %d", config_[type].msg[message].max_queue);
        RCLCPP_INFO(this->get_logger(), "      newest_first: %s", config_[type].msg[message].newest_first ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "      ttl: %d", config_[type].msg[message].ttl);
        RCLCPP_INFO(this->get_logger(), "      value_base: %d", config_[type].msg[message].value_base);
        
    }
  }
}