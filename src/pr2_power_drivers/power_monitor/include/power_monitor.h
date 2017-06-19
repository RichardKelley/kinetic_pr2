/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef POWER_MONITOR_POWER_MONITOR_H
#define POWER_MONITOR_POWER_MONITOR_H

#include <stdlib.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"

#include "pr2_msgs/BatteryServer2.h"
#include "pr2_msgs/PowerBoardState.h"
#include "pr2_msgs/PowerState.h"

#include "power_monitor/PowerMonitorConfig.h"

#include "power_state_estimator.h"

namespace power_monitor {

/**
 * This class listens to BatteryServer2 and PowerBoardState messages and publishes PowerState messages with
 * estimates of the time remaining until the robot switches off (if discharging) or until the robot is fully
 * charged (if charging).
 */
class PowerMonitor
{
public:
    PowerMonitor();

    bool setActiveEstimator(PowerStateEstimator::Type estimator_type);

    void batteryServerUpdate(const boost::shared_ptr<const pr2_msgs::BatteryServer2>& battery_server);
    void powerNodeUpdate(const boost::shared_ptr<const pr2_msgs::PowerBoardState>& power_board_state);
    void configCallback(power_monitor::PowerMonitorConfig& config, uint32_t level);

private:
    void addEstimator(PowerStateEstimator* estimator);

    PowerObservation extractObservation();

    void onPublishTimer(const ros::TimerEvent& e);

    void publishPowerState();

    std::string masterStateToString(int8_t master_state) const;

    ros::Time getLastBatteryUpdate() const;

private:
    dynamic_reconfigure::Server<power_monitor::PowerMonitorConfig> config_server_;

    boost::mutex update_mutex_;
    boost::mutex publish_mutex_;

    int8_t master_state_;
    std::map<int, boost::shared_ptr<const pr2_msgs::BatteryServer2> > battery_servers_;

    std::map<std::string,               PowerStateEstimator::Type>               estimator_types_;
    std::map<PowerStateEstimator::Type, boost::shared_ptr<PowerStateEstimator> > estimators_;

    boost::shared_ptr<PowerStateEstimator> active_estimator_;

    ros::Timer      power_state_pub_timer_;
    ros::Publisher  power_state_pub_;
    ros::Subscriber battery_server_sub_;
    ros::Subscriber power_node_sub_;

    PowerObservation observation_;

    double battery_update_timeout_;
};

}

#endif /* POWER_MONITOR_POWER_MONITOR_H */
