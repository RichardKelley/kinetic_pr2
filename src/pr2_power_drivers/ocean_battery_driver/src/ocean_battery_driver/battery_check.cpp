/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

///\author Kevin Watts
///\brief Battery self test. Checks that all batteries responding

#include <iostream>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>

#include <ros/rate.h>
#include <ros/time.h>

#include "ocean.h"

namespace po = boost::program_options;
using namespace std;
using namespace willowgarage::ocean;

class BatteryServerChecker
{
private:
  int id_;
  string device_;
  int timeout_;
  ocean os;
  volatile bool stopRequest;
  
  vector<bool> present;
  vector<bool> inhibited;
  vector<bool> no_good;
  vector<ros::Time> last_update;

  boost::shared_ptr<boost::thread> runThread_;

  void run()
  {
    ros::Rate my_rate(50);

    while (!stopRequest)
    {
      os.run();
      for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
      {
        present[i]     = os.server.battery[i].present;
        inhibited[i]   = os.server.battery[i].inhibited;
        no_good[i]     = os.server.battery[i].power_no_good;
        last_update[i] = os.server.battery[i].last_battery_update;
      }
        
      my_rate.sleep();
    }
  }

public:
  BatteryServerChecker(int id, const string &dev, int timeout):
    id_(id), device_(dev), timeout_(timeout), os(id, 0), stopRequest(false)
  {
    os.initialize(dev);

    present.resize(os.server.MAX_BAT_COUNT);
    inhibited.resize(os.server.MAX_BAT_COUNT);
    no_good.resize(os.server.MAX_BAT_COUNT);
    last_update.resize(os.server.MAX_BAT_COUNT);
  }

  void start()
  {
    runThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&BatteryServerChecker::run, this)));
  }
      
  void stop()
  {
    stopRequest = true;
 
    boost::posix_time::time_duration timeout = boost::posix_time::seconds(5);
    runThread_->timed_join(timeout);
  }

  bool batteryOK() const
  {
    bool ok = true; // Checks to make sure OK
    bool stale = false; // All batteries must have updated within timeout

    for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
    {
      ok = ok && present[i]; 
      stale = ((ros::Time::now() - last_update[i]).toSec() > timeout_) || stale;
    }

    return ok && !stale;
  }

  string getStatus() const
  {
    stringstream ss;
    ss << "Port: " << device_ << "\n";

    bool ng = false;
    for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
      {
	ss << "\tBattery " << i << ":\n";
	ss << "\t\tPresent: " << (present[i] ? "Yes" : "No") << "\n";
	ss << "\t\tInhibited: " << (inhibited[i] ? "Yes" : "No") << "\n";
	ss << "\t\tNo Good: " << (no_good[i] ? "Yes" : "No") << "\n";
        ss << "\t\tHas updated: " << (last_update[i] > ros::Time(1) ? "Yes" : "No") << "\n";
        ss << "\t\tTime Since Update: " << (ros::Time::now() - last_update[i]).toSec() << "\n";
        ng = ng || no_good[i];
      }
    if (ng)
      ss << "Warning: \"No Good\" flag enabled. This may cause battery problems.\n";

    return ss.str();
  }
};

int main(int argc, char** argv)
{
  int duration, min_duration, timeout;
  po::options_description desc("battery_check port1 port2 ... [options]");
  desc.add_options()
    ("help,h", "Print help message and exit")
    ("verbose,v", "Verbose battery output")
    ("timeout,t", po::value<int>(&timeout)->default_value(15), "Timeout before stale")
    ("duration,d", po::value<int>(&duration)->default_value(60), "Maximum duration of test")
    ("min-duration,m", po::value<int>(&min_duration)->default_value(20), "Minimum duration of test")
    ("port", po::value<vector<string> >(), "Battery ports to check");

  po::positional_options_description p;
  p.add("port", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  bool verbose = vm.count("verbose");

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  if (!vm.count("port"))
  {
    fprintf(stderr, "No ports specified. Unable to check batteries.\n");
    return 1;
  }

  if (min_duration < timeout)
  {
    fprintf(stderr, "Timeout must be greater than the minimum duration. Unable check batteries.\n");
    return 1;
  }

  vector<string> ports(vm["port"].as< vector<string> >());
  vector<string>::iterator it;

  if (verbose)
  {
    cout << "Checking ports: ";
    for (it = ports.begin(); it != ports.end(); ++it)
      cout << *it << ", ";
    cout << "\n";
  }

  if (verbose)
    cout << "Running battery check. Waiting for battery drivers to initialize\n";

  ros::Time::init();

  vector<boost::shared_ptr<BatteryServerChecker> > checkers;
  for (uint i = 0; i < ports.size(); ++i)
  {    
    checkers.push_back(boost::shared_ptr<BatteryServerChecker>(new BatteryServerChecker(i, ports[i], timeout)));
    checkers[i]->start();                       
  }

  if (verbose)
    cout << "Battery monitoring started\n";

  ros::Rate my_rate(2);
  ros::Time startTime = ros::Time::now();

  ros::Duration min(min_duration);
  ros::Duration max(duration);

  bool all_ok = false;
  while (true)
  {
    my_rate.sleep();
    
    if (ros::Time::now() - startTime < min)
      continue;

    if (ros::Time::now() - startTime > max)
      break;
   
    bool now_ok = true;
    for (uint i = 0; i < checkers.size(); ++i)
      now_ok = checkers[i]->batteryOK() && now_ok;

    if (now_ok)
    {
      all_ok = true;
      break;
    }
  }

  if (verbose)
    cout << "Stopping battery monitors\n";
 
  for (uint i = 0; i < checkers.size(); ++i)
    checkers[i]->stop();
 
  if (verbose)
  {
    fprintf(stdout, "Battery status reports:\n");
    for (uint i = 0; i < checkers.size(); ++i)
      cout << checkers[i]->getStatus().c_str();
  }

  if (all_ok)
  {
    fprintf(stdout, "All batteries OK\n");
    return 0;
  }

  fprintf(stderr, "Not all batteries reported OK.\n");
  cerr << "Status: \n";
  for (uint i = 0; i < checkers.size(); ++i)
    cerr << "\tDevice " << i << ": " << (checkers[i]->batteryOK() ? string("OK") : string("Error")) << "\n";
  
  return 1;
}
