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

    printf("MvpC2TrafficManager: loading config...\n");
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


    this->declare_parameter<std::vector<std::string>>("load_config", std::vector<std::string>());
    std::vector<std::string> configs;
    this->get_parameter("load_config", configs);


    for(const auto & type : configs)
    {
        this->declare_parameter<int>(type+".mac.local_address", 1);
        this->get_parameter(type+".mac.local_address", config_[type].mac.local_address);

        this->declare_parameter<int>(type+".mac.max_frame_bytes", 100);
        this->get_parameter(type+".mac.max_frame_bytes", config_[type].mac.max_frame_bytes);

        auto listed = this->list_parameters({type+".mac.remotes"}, 1);

        // Collect full names, e.g. "remotes.1"
        std::vector<std::string> names = listed.names;

        std::vector<rclcpp::Parameter> params;
        this->get_parameters(names, params);

        std::map<int,int> remotes;
        for (const auto & p : params) {
        // Drop the "remotes." prefix
        std::string suffix = p.get_name().substr(std::string("remotes.").size());
        int key   = std::stoi(suffix);
        int value = static_cast<int>(p.as_int());  // ROS stores ints as int64_t
        remotes.emplace(key, value);
        }

        for (auto &[k,v] : remotes) {
        RCLCPP_INFO(this->get_logger(), "Remote %d -> %d", k, v);
        }

        RCLCPP_INFO(get_logger(), "Loading config from: %s", type.c_str());
        RCLCPP_INFO(get_logger(), "  local_address: %d", config_[type].mac.local_address);
        // RCLCPP_INFO(get_logger(), "  remote_address size: %d", (int)config_[type].mac.remote_address.size());
        RCLCPP_INFO(get_logger(), "  max_frame_bytes: %d", config_[type].mac.max_frame_bytes);
        // RCLCPP_INFO(get_logger(), "  mac_slot_time size: %d", (int)config_[type].mac.mac_slot_time.size());


        // load message config
        this->declare_parameter<std::vector<std::string>>(type+".messages.load", std::vector<std::string>());
        std::vector<std::string> messages;
        this->get_parameter(type+".messages.load", messages);

        for (const auto & message : messages)
        {
            this->declare_parameter<bool>(type+".messages."+message+".ack", false);
            this->get_parameter(type+".messages."+message+".ack", config_[type].msg[message].ack);

            this->declare_parameter<int>(type+".messages."+message+".blackout_time", 0);
            this->get_parameter(type+".messages."+message+".blackout_time", config_[type].msg[message].blackout_time);

            this->declare_parameter<int>(type+".messages."+message+".max_queue", 0);
            this->get_parameter(type+".messages."+message+".max_queue", config_[type].msg[message].max_queue);

            this->declare_parameter<bool>(type+".messages."+message+".newest_first", true);
            this->get_parameter(type+".messages."+message+".newest_first", config_[type].msg[message].newest_first);      

            this->declare_parameter<int>(type+".messages."+message+".ttl", 1800);
            this->get_parameter(type+".messages."+message+".ttl", config_[type].msg[message].ttl);            

            this->declare_parameter<int>(type+".messages."+message+".value_base", 1);
            this->get_parameter(type+".messages."+message+".value_base", config_[type].msg[message].value_base);
            
            RCLCPP_INFO(get_logger(), "%s", message.c_str());
            RCLCPP_INFO(get_logger(), "    ack: %s", config_[type].msg[message].ack ? "true" : "false");
            RCLCPP_INFO(get_logger(), "    blackout_time: %d", config_[type].msg[message].blackout_time);
            RCLCPP_INFO(get_logger(), "    max_queue: %d", config_[type].msg[message].max_queue);
            RCLCPP_INFO(get_logger(), "    newest_first: %s", config_[type].msg[message].newest_first ? "true" : "false");
            RCLCPP_INFO(get_logger(), "    ttl: %d", config_[type].msg[message].ttl);
            RCLCPP_INFO(get_logger(), "    value_base: %d", config_[type].msg[message].value_base);
        }



    }
}
