
#pragma once

#include <time.h>
#include <string>
#include "ros/ros.h"
#include "pr2_power_board/PowerBoardCommand.h"
#include "pr2_power_board/PowerBoardCommand2.h"
#include "pr2_msgs/BatteryServer2.h"
#include "boost/thread/mutex.hpp"

class Interface 
{
  public:

    int recv_sock;
    int send_sock;
    Interface();
    ~Interface() {Close();}
    void Close();
    int Init( const std::string &address_str);
    int InitReceive(const std::string &address_str);
    void AddToReadSet(fd_set &set, int &max_sock) const;
    bool IsReadSet(fd_set set) const;
};


class Device 
{
  public:
    ros::Time message_time;
    
    const TransitionMessage &getTransitionMessage()
    {
      return tmsg;
    }
    void setTransitionMessage(const TransitionMessage &newtmsg);
    
    const PowerMessage &getPowerMessage()
    {
      return pmsg;
    }
    void setPowerMessage(const PowerMessage &newpmsg);
    
    Device();
    ~Device() { };
  private:
    bool tmsgset;
    TransitionMessage tmsg;
    bool pmsgset;
    PowerMessage pmsg;  //last power message recived from device
};


class PowerBoard
{
  public:
    PowerBoard( const ros::NodeHandle node_handle, const std::string &address_str );
    bool commandCallback( pr2_power_board::PowerBoardCommand::Request &req_,
                          pr2_power_board::PowerBoardCommand::Response &res_);
    bool commandCallback2( pr2_power_board::PowerBoardCommand2::Request &req_,
                          pr2_power_board::PowerBoardCommand2::Response &res_);
    void init();
    void collectMessages();
    void sendMessages();
    int collect_messages();
    int process_message(const PowerMessage *msg, int len);
    int process_transition_message(const TransitionMessage *msg, int len);
    const char* master_state_to_str(char state);
    const char* cb_state_to_str(char state);
    int list_devices(void);
    int send_command(int circuit_breaker, const std::string &command, unsigned flags);
    int requestMessage(const unsigned int message);

    void checkFanSpeed(); // Check battery temperatures and send command for fan speed

  private:
    ros::NodeHandle node_handle;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    ros::Publisher diags_pub;
    ros::Publisher state_pub;
    ros::Subscriber battery_sub_;

    std::map<int, float> battery_temps_;
    bool fan_high_;

    int getFanDuty(); // Duty cycle to send to fan. 0 if no fan command
  
    void batteryCB(const pr2_msgs::BatteryServer2::ConstPtr &msgPtr);

    pr2_power_board::PowerBoardCommand::Request req_;
    pr2_power_board::PowerBoardCommand::Response res_;
    boost::mutex library_lock_;
    uint64_t ip_address;

};
