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

#include "power_state_estimator.h"

#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "ros/ros.h"

using namespace std;
using namespace power_monitor;

// PowerObservation

PowerObservation::PowerObservation() { }

PowerObservation::PowerObservation(const ros::Time& stamp, int8_t master_state, const vector<BatteryObservation>& batteries) : stamp_(stamp), master_state_(master_state), batteries_(batteries) { }

const ros::Time&                  PowerObservation::getStamp()       const { return stamp_;        }
int8_t                            PowerObservation::getMasterState() const { return master_state_; }
const vector<BatteryObservation>& PowerObservation::getBatteries()   const { return batteries_;    }

unsigned int PowerObservation::getAcCount() const
{
    unsigned int ac_count = 0;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        if (batteries_[i].isAcPresent())
            ac_count++;

    return ac_count;
}

float PowerObservation::getTotalPower() const
{
    float total_power = 0.0f;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        total_power += batteries_[i].getPower();

    return total_power;
}

/**
  * Returns 9999.9 if no batteries observed.
  */
float PowerObservation::getMinVoltage() const
{
    float min_voltage = 9999.9f;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        min_voltage = min(min_voltage, batteries_[i].getVoltage());

    return min_voltage;
}

/**
  * Returns 999 if no batteries observed.
  */
unsigned int PowerObservation::getMinRelativeStateOfCharge() const
{
    unsigned int min_rsc = 999;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        min_rsc = min(min_rsc, batteries_[i].getRelativeStateOfCharge());

    return min_rsc;
}

float PowerObservation::getTotalRemainingCapacity() const
{
    float rem_cap = 0.0f;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        rem_cap += batteries_[i].getRemainingCapacity();

    return rem_cap;
}

/**
  * Returns ros::Duration(-1, 0) if all batteries are charging.
  */
ros::Duration PowerObservation::getMinTimeToEmpty(const ros::Time& t) const
{
    ros::Duration min_tte(-1, 0);

    int count = 0;

    for (unsigned int i = 0; i < batteries_.size(); i++)
    {
        const BatteryObservation& b = batteries_[i];

        if (b.isAcPresent())
            continue;

        ros::Duration staleness = t - b.getStamp();

        ros::Duration tte(0);
        if (staleness < b.getTimeToEmpty())
            tte = b.getTimeToEmpty() - staleness;

        if (count == 0)
            min_tte = tte;
        else
            min_tte = min(min_tte, tte);

        count++;
    }

    return min_tte;
}

/**
  * Returns ros::Duration(-1, 0) if no batteries are charging.
  */
ros::Duration PowerObservation::getMaxTimeToFull(const ros::Time& t) const
{
    ros::Duration max_ttf(-1, 0);

    int count = 0;

    for (unsigned int i = 0; i < batteries_.size(); i++)
    {
        const BatteryObservation& b = batteries_[i];

        if (!b.isAcPresent())
            continue;

        ros::Duration staleness = t - b.getStamp();

        ros::Duration ttf(0);
        if (staleness < b.getTimeToFull())
            ttf = b.getTimeToFull() - staleness;

        if (count == 0)
            max_ttf = ttf;
        else
            max_ttf = max(max_ttf, ttf);

        count++;
    }

    return max_ttf;
}

// BatteryObservation

BatteryObservation::BatteryObservation(const ros::Time& stamp, bool ac_present, float voltage, float current,
                                       unsigned int relative_state_of_charge, float remaining_capacity,
                                       const ros::Duration& time_to_empty, const ros::Duration& time_to_full)
    : stamp_(stamp), ac_present_(ac_present), voltage_(voltage), current_(current), relative_state_of_charge_(relative_state_of_charge),
      remaining_capacity_(remaining_capacity), time_to_empty_(time_to_empty), time_to_full_(time_to_full)
{
}

const ros::Time& BatteryObservation::getStamp() const { return stamp_; }

bool                 BatteryObservation::isAcPresent()              const { return ac_present_;               }
float                BatteryObservation::getVoltage()               const { return voltage_;                  }
float                BatteryObservation::getCurrent()               const { return current_;                  }
unsigned int         BatteryObservation::getRelativeStateOfCharge() const { return relative_state_of_charge_; }
float                BatteryObservation::getRemainingCapacity()     const { return remaining_capacity_;       }
const ros::Duration& BatteryObservation::getTimeToEmpty()           const { return time_to_empty_;            }
const ros::Duration& BatteryObservation::getTimeToFull()            const { return time_to_full_;             }

float BatteryObservation::getPower() const
{
    return voltage_ * current_;
}
