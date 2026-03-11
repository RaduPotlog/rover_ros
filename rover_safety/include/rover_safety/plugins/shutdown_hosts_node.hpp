
// Copyright 2025 Mechatronics Academy
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

#ifndef ROVER_SAFETY_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_
#define ROVER_SAFETY_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_

#include <algorithm>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "rover_safety/plugins/shutdown_host.hpp"

#include "rover_safety/behavior_tree_utils.hpp"

namespace rover_safety
{

class ShutdownHosts : public BT::StatefulActionNode
{
public:
  
    explicit ShutdownHosts(
        const std::string & name, 
        const BT::NodeConfiguration & conf)
    : BT::StatefulActionNode(name, conf)
    {
        this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(name));
    }

    virtual ~ShutdownHosts() = default;

    virtual bool updateHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts) = 0;

    virtual BT::NodeStatus postProcess()
    {
        if (this->failed_hosts_.size() == 0) {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }

    std::vector<std::size_t> const getFailedHosts() 
    { 
        return this->failed_hosts_; 
    }

protected:
  
    BT::NodeStatus onStart()
    {
        if (!updateHosts(this->hosts_)) {
            RCLCPP_ERROR_STREAM(*this->logger_, getLoggerPrefix(name()) << "Cannot update hosts!");
            
            return BT::NodeStatus::FAILURE;
        }

        removeDuplicatedHosts(this->hosts_);
    
        if (this->hosts_.size() <= 0) {
            RCLCPP_ERROR_STREAM(*this->logger_, getLoggerPrefix(name()) << "Hosts list is empty! Check configuration!");
    
            return BT::NodeStatus::FAILURE;
        }
    
        this->hosts_to_check_.resize(this->hosts_.size());
        std::iota(this->hosts_to_check_.begin(), this->hosts_to_check_.end(), 0);
    
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        if (this->hosts_to_check_.size() <= 0) {
            return postProcess();
        }

        if (this->check_host_index_ >= this->hosts_to_check_.size()) {
            this->check_host_index_ = 0;
        }

        auto host_index = this->hosts_to_check_[this->check_host_index_];
        auto host = this->hosts_[host_index];
        host->call();

        switch (host->getState()) {
            case ShutdownHostState::RESPONSE_RECEIVED:
                RCLCPP_INFO_STREAM(
                    *this->logger_, getLoggerPrefix(name())
                    << "Device at: " << host->getIp() << " response:\n"
                    << host->getOutput());

                check_host_index_++;
                break;

            case ShutdownHostState::SUCCESS:
                RCLCPP_INFO_STREAM(
                    *this->logger_, getLoggerPrefix(name())
                    << "Successfully shutdown device at: " << host->getIp());
                    this->succeeded_hosts_.push_back(host_index);
                    this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
                
                break;

            case ShutdownHostState::FAILURE:
                RCLCPP_WARN_STREAM(
                    *this->logger_, getLoggerPrefix(name())
                    << "Failed to shutdown device at: " << host->getIp()
                    << "\nError: " << host->getError());

                this->failed_hosts_.push_back(host_index);
                this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
                
                break;

            case ShutdownHostState::SKIPPED:
                RCLCPP_WARN_STREAM(
                    *this->logger_, getLoggerPrefix(name())
                    << "Device at: " << host->getIp() << " not available, skipping...");

                this->skipped_hosts_.push_back(host_index);
                this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
                
                break;

            default:
                this->check_host_index_++;
                
                break;
        }

        return BT::NodeStatus::RUNNING;
    }

    void removeDuplicatedHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts)
    {
        auto comp = [](
            const std::shared_ptr<ShutdownHostInterface> & lhs,
            const std::shared_ptr<ShutdownHostInterface> & rhs) { return *lhs < *rhs; };

        std::set<std::shared_ptr<ShutdownHostInterface>, decltype(comp)> seen(comp);

        hosts.erase(
            std::remove_if(hosts.begin(), hosts.end(), [&](const std::shared_ptr<ShutdownHostInterface> & host) {
          
                if (!seen.count(host)) {
                    seen.insert(host);
                    return false;
                } else {
                    RCLCPP_WARN_STREAM(
                        *this->logger_, getLoggerPrefix(name()) << "Found duplicate host: " << host->getIp()
                        << " Processing only the first occurrence.");
          
                    return true;
                }
            }),

        hosts.end());
    }

    void onHalted()
    {
        for (auto & host : this->hosts_) {
            host->halt();
        }
    }

    std::shared_ptr<rclcpp::Logger> logger_;
    std::size_t check_host_index_ = 0;
    std::vector<std::shared_ptr<ShutdownHostInterface>> hosts_;
    std::vector<std::size_t> hosts_to_check_;
    std::vector<std::size_t> skipped_hosts_;
    std::vector<std::size_t> succeeded_hosts_;
    std::vector<std::size_t> failed_hosts_;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_
