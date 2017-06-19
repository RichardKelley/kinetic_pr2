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

#include "power_monitor.h"

using namespace std;
using namespace power_monitor;

PowerMonitor::PowerMonitor() : master_state_(-1),
                               battery_update_timeout_(120)
{
    ros::NodeHandle node;
    ros::NodeHandle pnh("~");

    static const string battery_server_topic = "battery/server2";
    static const string power_board_node     = "power_board";
    string estimator_type_str                = "advanced";
    double freq                              = 0.1;

    pnh.getParam("estimation_method",    estimator_type_str);
    pnh.getParam("frequency",            freq);
    pnh.getParam("battery_update_timeout", battery_update_timeout_);

    ros::Duration(2).sleep();

    // Register the estimators
    addEstimator(new FuelGaugePowerStateEstimator());
    addEstimator(new AdvancedPowerStateEstimator());

    // Setup the dynamic_reconfigure callback
    dynamic_reconfigure::Server<PowerMonitorConfig>::CallbackType config_callback = boost::bind(&PowerMonitor::configCallback, this, _1, _2);
    config_server_.setCallback(config_callback);

    // Set the active estimation method
    if (estimator_types_.size() == 0)
    {
        ROS_FATAL("No power state estimators defined. Shutting down");
        exit(1);
    }
    map<string, PowerStateEstimator::Type>::const_iterator i = estimator_types_.find(estimator_type_str);
    if (i == estimator_types_.end())
    {
        // Requested estimator is unknown. Default to first estimator
        string first_estimator_type_str = estimator_types_.begin()->first;
        ROS_ERROR("Unknown power state estimator type: %s. Defaulting to %s", estimator_type_str.c_str(), first_estimator_type_str.c_str());
        setActiveEstimator(estimator_types_.begin()->second);
    }
    else
        setActiveEstimator(i->second);

    power_state_pub_       = node.advertise<pr2_msgs::PowerState>("power_state", 5, true);
    power_state_pub_timer_ = node.createTimer(ros::Duration(1.0 / freq), &PowerMonitor::onPublishTimer, this);
    battery_server_sub_    = node.subscribe(battery_server_topic, 10, &PowerMonitor::batteryServerUpdate, this);
    power_node_sub_        = node.subscribe(power_board_node + "/state", 10, &PowerMonitor::powerNodeUpdate, this);
}

void PowerMonitor::addEstimator(PowerStateEstimator* est)
{
    estimator_types_[est->getMethodName()] = est->getMethodType();
    estimators_[est->getMethodType()] = boost::shared_ptr<PowerStateEstimator>(est);
}

void PowerMonitor::configCallback(PowerMonitorConfig& config, uint32_t level)
{
    setActiveEstimator((PowerStateEstimator::Type) config.power_state_estimator_);
}

bool PowerMonitor::setActiveEstimator(PowerStateEstimator::Type estimator_type)
{
    map<PowerStateEstimator::Type, boost::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.find(estimator_type);
    if (i == estimators_.end())
        return false;
    if (active_estimator_ == i->second)
        return true;

    if (active_estimator_ == boost::shared_ptr<PowerStateEstimator>())
        ROS_INFO("Power state estimator set to %s", i->second->getMethodName().c_str());
    else
        ROS_INFO("Power state estimator changed from %s to %s", active_estimator_->getMethodName().c_str(), i->second->getMethodName().c_str());

    active_estimator_ = i->second;

    return true;
}

void PowerMonitor::batteryServerUpdate(const boost::shared_ptr<const pr2_msgs::BatteryServer2>& battery_server)
{
    boost::mutex::scoped_lock lock(update_mutex_);

    ROS_DEBUG("Received battery message: voltage=%.2f", battery_server->battery[0].battery_register[0x9] / 1000.0f);

    battery_servers_[battery_server->id] = battery_server;
}

void PowerMonitor::powerNodeUpdate(const boost::shared_ptr<const pr2_msgs::PowerBoardState>& power_board_state)
{
    boost::mutex::scoped_lock lock(update_mutex_);

    ROS_DEBUG("Received power board state message: %s", masterStateToString(power_board_state->master_state).c_str());

    master_state_ = power_board_state->master_state;

    // Publish the power state immediately if we're shutting down. Want to ensure we record these data points.
    if (master_state_ == pr2_msgs::PowerBoardState::MASTER_SHUTDOWN)
    {
      ROS_WARN("Power board reports imminant shutdown");
      publishPowerState();
    }
}

string PowerMonitor::masterStateToString(int8_t master_state) const
{
    switch (master_state)
    {
        case pr2_msgs::PowerBoardState::MASTER_NOPOWER:  return "No Power";
        case pr2_msgs::PowerBoardState::MASTER_STANDBY:  return "Standby";
        case pr2_msgs::PowerBoardState::MASTER_ON:       return "On";
        case pr2_msgs::PowerBoardState::MASTER_OFF:      return "Off";
        case pr2_msgs::PowerBoardState::MASTER_SHUTDOWN: return "Shutdown";
        default:                                         return "Unknown";
    }
}

PowerObservation PowerMonitor::extractObservation()
{
    boost::mutex::scoped_lock lock(update_mutex_);

    vector<BatteryObservation> batteries;
    for (map<int, boost::shared_ptr<const pr2_msgs::BatteryServer2> >::iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
    {
        const pr2_msgs::BatteryServer2* bs = i->second.get();

        ros::Time stamp = bs->header.stamp;

        for (unsigned int j = 0; j < bs->battery.size(); j++)
        {
            const pr2_msgs::BatteryState2& b = bs->battery[j];

            bool         ac_present = b.power_present;
            float        voltage    = b.battery_register[0x9] / 1000.0f;
            float        current    = b.battery_register[0xA] / 1000.0f;
            unsigned int rsc        = b.battery_register[0x0D];
            float        rem_cap    = b.battery_register[0x0F] / 1000.0f;
            unsigned int tte_min    = b.battery_register[0x12];
            unsigned int ttf_min    = b.battery_register[0x13];

            ros::Duration tte;
            if (tte_min > 0)
                tte = ros::Duration(tte_min * 60, 0);
            else
                tte = ros::Duration(-1, 0);

            ros::Duration ttf;
            if (ttf_min > 0)
                ttf = ros::Duration(ttf_min * 60, 0);
            else
                ttf = ros::Duration(-1, 0);

            if (voltage == 0.0)
                continue;

            batteries.push_back(BatteryObservation(stamp, ac_present, voltage, current, rsc, rem_cap, tte, ttf));

            ROS_DEBUG("Battery %d.%d: %6.2f V %6.2f A %6.2f W (soc: %d, cap: %6.2f, tte: %dm, ttf: %dm)", bs->id, j + 1, voltage, current, current * voltage, rsc, rem_cap, tte_min, ttf_min);
        }
    }

    return PowerObservation(ros::Time::now(), master_state_, batteries);
}

ros::Time PowerMonitor::getLastBatteryUpdate() const
{
  ros::Time rv;
  for (map<int, boost::shared_ptr<const pr2_msgs::BatteryServer2> >::const_iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
  {
    const pr2_msgs::BatteryServer2* bs = i->second.get();
    if (bs->header.stamp > rv)
      rv = bs->header.stamp;
  }

  return rv;
}

void PowerMonitor::onPublishTimer(const ros::TimerEvent& e)
{
  // Don't publish power data if we haven't received battery data in a timeout, #4851
  if (battery_update_timeout_ > 0 && ((ros::Time::now() - getLastBatteryUpdate()).toSec() > battery_update_timeout_))
  {
    ROS_WARN_THROTTLE(60, "Power monitor not publishing estimate, batteries have not recently updated. Check diagnostics.");
    return;
  }

  publishPowerState();
}

void PowerMonitor::publishPowerState()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    // Extract info from the battery server
    PowerObservation obs = extractObservation();
    if (obs.getBatteries().size() == 0)
    {
        ROS_DEBUG("Nothing observed");
        return;
    }

    ROS_DEBUG("Power: %6.1f W. Min voltage: %6.2f V", obs.getTotalPower(), obs.getMinVoltage());

    // Give every estimator the chance to record this observation
    for (map<PowerStateEstimator::Type, boost::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.begin(); i != estimators_.end(); i++)
        i->second.get()->recordObservation(obs);

    // Use the active estimator to estimate the time and capacity remaining
    ros::Time t = ros::Time::now();
    if (active_estimator_->canEstimate(t))
    {
        PowerStateEstimate estimate = active_estimator_->estimate(t);
        ROS_DEBUG("Remaining: %.0f min (%d%%)", estimate.time_remaining.toSec() / 60, estimate.relative_capacity);

        // Publish the power state estimate
        pr2_msgs::PowerState ps;
        ps.header.stamp      = ros::Time::now();
        ps.AC_present        = obs.getAcCount();
        ps.power_consumption = obs.getTotalPower();
        ps.prediction_method = active_estimator_->getMethodName();
        ps.relative_capacity = (int8_t) estimate.relative_capacity;
        ps.time_remaining    = estimate.time_remaining;
        power_state_pub_.publish(ps);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_monitor");
    PowerMonitor monitor;
    ros::spin();
    return 0;
}
