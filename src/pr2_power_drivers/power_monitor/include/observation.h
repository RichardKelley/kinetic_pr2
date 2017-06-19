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

#ifndef POWER_MONITOR_OBSERVATION_H
#define POWER_MONITOR_OBSERVATION_H

#include <stdlib.h>

#include "ros/ros.h"

#include "pr2_msgs/PowerBoardState.h"

namespace power_monitor {

class BatteryObservation;

/**
 * Stores information reported by the power system used to estimate the power state.
 */
class PowerObservation
{
public:
    PowerObservation();
    PowerObservation(const ros::Time& stamp, int8_t master_state, const std::vector<BatteryObservation>& batteries);

    const ros::Time&                       getStamp() const;
    int8_t                                 getMasterState() const;
    const std::vector<BatteryObservation>& getBatteries() const;

    unsigned int  getAcCount() const;
    float         getTotalPower() const;
    float         getMinVoltage() const;
    unsigned int  getMinRelativeStateOfCharge() const;
    float         getTotalRemainingCapacity() const;
    ros::Duration getMinTimeToEmpty(const ros::Time& t) const;
    ros::Duration getMaxTimeToFull(const ros::Time& t) const;

private:
    ros::Time                       stamp_;
    int8_t                          master_state_;
    std::vector<BatteryObservation> batteries_;
};

/**
 * Stores information reported by a single battery.
 */
class BatteryObservation
{
public:
    BatteryObservation(const ros::Time& stamp, bool ac_present, float voltage, float current,
                       unsigned int relative_state_of_charge, float remaining_capacity, const ros::Duration& time_to_empty, const ros::Duration& time_to_full);

    const ros::Time& getStamp() const;

    bool                 isAcPresent() const;
    float                getVoltage() const;
    float                getCurrent() const;
    unsigned int         getRelativeStateOfCharge() const;
    float                getRemainingCapacity() const;
    const ros::Duration& getTimeToEmpty() const;
    const ros::Duration& getTimeToFull() const;

    float getPower() const;

private:
    ros::Time     stamp_;
    bool          ac_present_;
    float         voltage_;                   // V
    float         current_;                   // A (-ve if discharging)
    unsigned int  relative_state_of_charge_;  // % [0-100]
    float         remaining_capacity_;        // Ah
    ros::Duration time_to_empty_;
    ros::Duration time_to_full_;
};

}

#endif /* POWER_MONITOR_OBSERVATION_H */
