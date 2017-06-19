
#include <iostream>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include <log4cxx/logger.h>

#include "ocean.h"
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "rosconsole/macros_generated.h"
#include "pr2_msgs/BatteryServer.h" //This is here to send a copy of the previous message

using namespace std;
using namespace ros;
using namespace willowgarage::ocean;
namespace po = boost::program_options;

static const float BATTERY_TEMP_WARN = 50.0;

float toFloat(const int &value)
{
  int tmp = value;
  if(tmp & 0x8000)
    tmp = tmp - 65536;
  float result = tmp / 1000.0;
  return result;
}

float tempToCelcius(const int &iKelvin)
{
  float fKelvin = (float) (iKelvin * 0.1);
  return (fKelvin - (float) 273.15);
}

class server
{
  public:

    server( const int &majorID, const std::string &dev, const int debug_level = 0 ) : 
      majorID(majorID), debug_level(debug_level), serial_device("/dev/ttyUSB0"), stopRequest(false),
      has_warned_temp_alarm_(false), has_warned_no_good_(false)
    {
      std::stringstream ss;

      ros::NodeHandle private_handle("~");

      if(dev.empty())
      {
        string tmp_device;
        ss.str("");
        ss << "/dev/ttyUSB" << majorID; //create a default device based on majorID
        serial_device = ss.str();

        ss.str("");
        ss << "port" << majorID;
        bool result = private_handle.getParam( ss.str(), tmp_device );
        if(result == true)
        {
          ROS_INFO("Using %s from getParam.\n", ss.str().c_str());
          serial_device = tmp_device;
        }
        else
          ROS_INFO("Defaulting to: %s", serial_device.c_str());

      }
      else
      {
        ROS_INFO("Overriding device with argument: %s", dev.c_str() );
        serial_device = dev;
      }

      private_handle.param("lag_timeout", lag_timeout_, 60);
      private_handle.param("stale_timeout", stale_timeout_, 120);

      //
      //printf("device=%s  debug_level=%d\n", argv[1], atoi(argv[2]));
      //cout << "device=" << serial_device <<  "  debug_level=" << debug_level << endl;

    }

    void start()
    {
      myThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&server::run, this)));
    }


    void stop()
    {
      stopRequest = true;
      myThread->join();
    }

  private:

    ros::NodeHandle handle;
    int majorID;
    int debug_level;
    std::string serial_device;
    volatile bool stopRequest;
    boost::shared_ptr<boost::thread> myThread;
    int lag_timeout_, stale_timeout_;
    bool has_warned_temp_alarm_, has_warned_no_good_;

    void run()
    {
      std::stringstream ss;

      //
      //  Need to make the queue size big enough that each thread can publish without
      //  concern that one message it quickly replaced by another threads message.
      //
      ros::Publisher pub    = handle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
      ros::Publisher bs2    = handle.advertise<pr2_msgs::BatteryServer2>("battery/server2", 10);
      ros::Publisher bs     = handle.advertise<pr2_msgs::BatteryServer>("battery/server", 10);

      ros::Rate rate(100);   //set the rate we scan the device for input
      diagnostic_msgs::DiagnosticArray msg_out;
      diagnostic_updater::DiagnosticStatusWrapper stat;
      Time lastTime, currentTime, startTime;
      Duration MESSAGE_TIME(2,0);    //the message output rate
      ocean os( majorID, debug_level);
      os.initialize(serial_device.c_str());
      //os.read_file(serial_device.c_str());

      pr2_msgs::BatteryServer oldserver;
      oldserver.battery.resize(4);

      lastTime = Time::now();
      startTime = Time::now();

      while(handle.ok() && (stopRequest == false))
      {
        rate.sleep();
        //ros::spinOnce();
        currentTime = Time::now();

        if((os.run() > 0) && ((currentTime - lastTime) > MESSAGE_TIME))
        {

          // First publish our internal data
          os.server.header.stamp = ros::Time::now();
          bs2.publish(os.server);

          oldserver.id = os.server.id;
          oldserver.lastTimeSystem = os.server.last_system_update.sec;
          oldserver.timeLeft = os.server.time_left.toSec()/60;
          oldserver.averageCharge = os.server.average_charge;
          oldserver.message = os.server.message;
          oldserver.lastTimeController = os.server.last_controller_update.sec;
          oldserver.present = os.server.battery[0].present * 1 + os.server.battery[1].present * 2 + os.server.battery[2].present * 4 + os.server.battery[3].present * 8;
          oldserver.charging = os.server.battery[0].charging * 1 + os.server.battery[1].charging * 2 + os.server.battery[2].charging * 4 + os.server.battery[3].charging * 8;
          oldserver.discharging = os.server.battery[0].discharging * 1 + os.server.battery[1].discharging * 2 + os.server.battery[2].discharging * 4 + os.server.battery[3].discharging * 8;
          //oldserver.reserved = os.server.battery[0].reserved * 1 + os.server.battery[1].reserved * 2 + os.server.battery[2].reserved * 4 + os.server.battery[3].reserved * 8;
          oldserver.powerPresent = os.server.battery[0].power_present * 1 + os.server.battery[1].power_present * 2 + os.server.battery[2].power_present * 4 + os.server.battery[3].power_present * 8;
          oldserver.powerNG = os.server.battery[0].power_no_good * 1 + os.server.battery[1].power_no_good * 2 + os.server.battery[2].power_no_good * 4 + os.server.battery[3].power_no_good * 8;
          oldserver.inhibited = os.server.battery[0].inhibited * 1 + os.server.battery[1].inhibited * 2 + os.server.battery[2].inhibited * 4 + os.server.battery[3].inhibited * 8;

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            oldserver.battery[xx].lastTimeBattery = os.server.battery[xx].last_battery_update.sec;
            for(unsigned int yy = 0; yy < os.regListLength; ++yy)
            {
              oldserver.battery[xx].batReg[yy] = os.server.battery[xx].battery_register[yy];
              oldserver.battery[xx].batRegFlag[yy] = os.server.battery[xx].battery_update_flag[yy];
              oldserver.battery[xx].batRegTime[yy] = os.server.battery[xx].battery_register_update[yy].sec;
            }
          }

          oldserver.header.stamp = ros::Time::now();
          bs.publish(oldserver);

          lastTime = currentTime;

          stat.values.clear();

          ss.str("");
          ss << "IBPS " << majorID;
          stat.name = ss.str();
          stat.level = diagnostic_msgs::DiagnosticStatus::OK;
          stat.message = "OK";
          
          stat.add("Time Remaining (min)", (os.server.time_left.toSec()/60));
          stat.add("Average charge (percent)", os.server.average_charge );
          Duration elapsed = currentTime - os.server.last_system_update;
          stat.add("Time since update (s)", elapsed.toSec());

          msg_out.header.stamp = ros::Time::now();
          msg_out.status.push_back(stat);

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            stat.values.clear();
            
            ss.str("");
            ss << "Smart Battery " << majorID << "." << xx;
            stat.name = ss.str();
            stat.level = diagnostic_msgs::DiagnosticStatus::OK;
            stat.message = "OK";

            if(!os.server.battery[xx].present)
            {
              stat.add("Battery Present", "False");
              stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
              stat.message = "Not present";
            }
            else
            {
              stat.add("Battery Present", "True");
              stat.add("Charging", (os.server.battery[xx].charging) ? "True":"False");
              stat.add("Discharging", (os.server.battery[xx].discharging) ? "True":"False");
              stat.add("Power Present", (os.server.battery[xx].power_present) ? "True":"False");
              stat.add("No Good", (os.server.battery[xx].power_no_good) ? "True":"False");
              stat.add("Charge Inhibited", (os.server.battery[xx].inhibited) ? "True":"False");

              int16_t voltage = -1;
              int16_t design_voltage = -1;
              int16_t relative_charge = -1;
              int16_t absolute_charge = -1;
              
              for(unsigned int yy = 0; yy < os.regListLength; ++yy)
              {
                unsigned addr = os.regList[yy].address;
                if(os.server.battery[xx].battery_update_flag[addr])
                {
                  ss.str("");
                  if(os.regList[yy].unit != "")
                    ss << os.regList[yy].name << " (" << os.regList[yy].unit << ")";
                  else
                    ss << os.regList[yy].name;
                  
                  switch(addr) {
                     case 0x1b:  //Address of manufactureDate
                        {
                           std::stringstream date;
                           date.str("");

                           unsigned int day = os.server.battery[xx].battery_register[addr] & 0x1F;
                           unsigned int month = (os.server.battery[xx].battery_register[addr] >> 5) & 0xF;
                           unsigned int year = (os.server.battery[xx].battery_register[addr] >> 9) + 1980;
                           date << month << "/" << day << "/" << year;

                           stat.add("Manufacture Date (MDY)", date.str());
                        }
                        break;
                     case 0x8:  //Address of Temperature
                        {
                           float celsius = tempToCelcius(os.server.battery[xx].battery_register[addr]);
                           if(celsius > BATTERY_TEMP_WARN)
                           {
                              ostringstream warn;
                              warn << "High Temperature Warning > " << BATTERY_TEMP_WARN << "C";
                              stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, warn.str());
                           }

                           stat.add("Temperature (C)", celsius);
                        }
                        break;
                     case 0x1c: // Serial Number
                        {
                           stat.add("Serial Number", os.server.battery[xx].battery_register[addr]);

                           std::stringstream hw_id;
                           hw_id << os.server.battery[xx].battery_register[addr];

                           stat.hardware_id = hw_id.str();
                        }
                        break;
                     case 0x09: // Voltage
                        voltage = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x19: // Design Voltage
                        design_voltage = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x0d: // Relative state of charge
                        relative_charge = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x0e: // Absolute state of charge
                        absolute_charge = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     default:
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                  }
                }
              }

              elapsed = currentTime - os.server.battery[xx].last_battery_update;
              if (os.server.battery[xx].last_battery_update >= Time(1))
                stat.add("Time since update (s)", elapsed.toSec());
              else
                stat.add("Time since update (s)", "N/A");
              
              if ( absolute_charge >= 0 && relative_charge >= 0 ) {
                 if( absolute_charge < relative_charge/2 ) {
                    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery capacity low, please replace.");
                 }
              }

              if ( voltage >= 0 && design_voltage >= 0 ) {
                 if( voltage < design_voltage/2 ) {
                    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery voltage too low, please replace.");
                 }
              }

              // Mark batteries as stale if they don't update
              // Give them "grace period" on startup to update before we report error
              bool updateGracePeriod = (currentTime - startTime).toSec() < stale_timeout_;
              if (os.server.battery[xx].last_battery_update <= Time(1) && updateGracePeriod)
                stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Waiting for battery update");
              else if (stale_timeout_ > 0 && elapsed.toSec() > stale_timeout_)
                stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "No updates");
              else if (lag_timeout_ > 0 && elapsed.toSec() > lag_timeout_)
                stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Stale updates");


              // Warn for over temp alarm
              // If power present and not charging, not full, and temp >= 46C
              // 0x0d is "Relative State of Charge"
              if (os.server.battery[xx].power_present && !os.server.battery[xx].charging 
                  && os.server.battery[xx].battery_register[0x0d] < 90
                  && tempToCelcius(os.server.battery[xx].battery_register[0x8]) > 46.0)
                {
                  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Charge Inhibited, High Temperature");
                  if (!has_warned_temp_alarm_)
                    {
                      ROS_WARN("Over temperature alarm found on battery %d. Battery will not charge.", xx);
                      has_warned_temp_alarm_ = true;
                    }
                }

              // Check for battery status code, not sure if this works.
              if (os.server.battery[xx].battery_register[0x16] & 0x1000)
                  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Over Temp Alarm");

              // Report a console warning if battery is "No Good"
              // This may be a problem with the battery, but we're not sure.
              if (os.server.battery[xx].power_no_good && !has_warned_no_good_)
                {
                  ROS_WARN("Battery %d reports \"No Good\".", xx);
                  has_warned_no_good_ = true;
                }
                          
              msg_out.status.push_back(stat);
            }
          }

          pub.publish(msg_out);
          msg_out.status.clear();

        }
      }
    }

};

int main(int argc, char** argv)
{
  string tmp_device;
  int debug_level;
  int max_ports;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("debug", po::value<int>(&debug_level)->default_value(0), "debug level")
    ("count", po::value<int>(&max_ports)->default_value(4), "number of ports to monitor")
    ("dev", po::value<string>(&tmp_device), "serial device to open");

  po::variables_map vm;
  po::store(po::parse_command_line( argc, argv, desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  ros::init(argc, argv, "ocean_server");
  ros::NodeHandle handle;
  ros::NodeHandle private_handle("~");

  //majorID = serial_device.at(serial_device.length() - 1) - '0';

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  fprintf(stderr, "Logger Name: %s\n", ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  private_handle.getParam( "number_of_ports", max_ports );
  ROS_INFO("number_of_ports=%d", max_ports);
  private_handle.getParam( "debug_level", debug_level );
  ROS_DEBUG("debug_level=%d", debug_level);

  vector<server> server_list;

  for(int xx = 0; xx < max_ports; ++xx)
    server_list.push_back(server( xx, tmp_device, debug_level));

  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx].start();

  ros::spin(); //wait for ros to shut us down


  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx].stop();

}
