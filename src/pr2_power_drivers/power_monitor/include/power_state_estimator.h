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

#ifndef POWER_MONITOR_POWER_STATE_ESTIMATOR_H
#define POWER_MONITOR_POWER_STATE_ESTIMATOR_H

#include <stdlib.h>

#include "ros/ros.h"

#include "observation.h"

namespace power_monitor {

struct PowerStateEstimate
{
    ros::Duration time_remaining;
    unsigned int  relative_capacity;
};

/**
 * PowerStateEstimator can takes observable power inputs (PowerObservation) and estimates the hidden power state (PowerStateEstimate).
 */
class PowerStateEstimator
{
public:
    enum Type { FuelGauge, Advanced };

    PowerStateEstimator();

    virtual std::string               getMethodName() const = 0;
    virtual PowerStateEstimator::Type getMethodType() const = 0;
    virtual PowerStateEstimate        estimate(const ros::Time& t) = 0;

    virtual void recordObservation(const PowerObservation& obs);

    virtual bool canEstimate(const ros::Time& t) const;

protected:
    PowerObservation obs_;
};

/** A simple power state estimator which relies on the values reported by the batteries themselves
  * for the minimum & maximum time remaining and the minimum remaining capacity.
  */
class FuelGaugePowerStateEstimator : public PowerStateEstimator
{
public:
    std::string               getMethodName() const;
    PowerStateEstimator::Type getMethodType() const;
    PowerStateEstimate        estimate(const ros::Time& t);
};

/** A more advanced power state estimator which is informed by the history of the batteries.
  */
class AdvancedPowerStateEstimator : public PowerStateEstimator
{
public:
    struct LogRecord
    {
        uint32_t     sec;
        int8_t       master_state;
        int          charging;
        float        total_power;
        float        min_voltage;
        unsigned int min_relative_state_of_charge;
        float        total_remaining_capacity;
    };

    AdvancedPowerStateEstimator();

    std::string               getMethodName() const;
    PowerStateEstimator::Type getMethodType() const;
    PowerStateEstimate        estimate(const ros::Time& t);

    virtual void recordObservation(const PowerObservation& obs);

private:
    static const std::string DEFAULT_LOG_FILE;

    static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters=",");

    bool readObservations(std::vector<LogRecord>& log);
    bool saveObservation(const PowerObservation& obs) const;
    bool hasEverDischarged() const;

    std::vector<LogRecord> log_;

    std::string log_filename_;
};

}

#endif /* POWER_MONITOR_POWER_STATE_ESTIMATOR_H */
