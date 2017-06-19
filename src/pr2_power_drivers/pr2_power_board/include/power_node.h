
#pragma once

#include <time.h>
#include "ros/ros.h"
#include "pr2_power_board/PowerBoardCommand.h"
#include "boost/thread/mutex.hpp"

class Interface 
{
  public:

    struct ifreq interface;
    int recv_sock;
    int send_sock;
    Interface(const char* ifname);
    ~Interface() {Close();}
    void Close();
    int Init(sockaddr_in *port_address, sockaddr_in *broadcast_address);
    int InitReceive();
    void AddToReadSet(fd_set &set, int &max_sock) const;
    bool IsReadSet(fd_set set) const;
    sockaddr_in ifc_address;
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
    PowerBoard( const ros::NodeHandle node_handle, unsigned int serial_number = 0 );
    bool commandCallback( pr2_power_board::PowerBoardCommand::Request &req_,
                          pr2_power_board::PowerBoardCommand::Response &res_);
    void init();
    void collectMessages();
    void sendMessages();
    int collect_messages();
    int process_message(const PowerMessage *msg, int len);
    int process_transition_message(const TransitionMessage *msg, int len);
    const char* master_state_to_str(char state);
    const char* cb_state_to_str(char state);
    int list_devices(void);
    int send_command(unsigned int serial_number, int circuit_breaker, const std::string &command, unsigned flags);

  private:
    ros::NodeHandle node_handle;
    ros::ServiceServer service;
    ros::Publisher diags_pub;
    ros::Publisher state_pub;

    pr2_power_board::PowerBoardCommand::Request req_;
    pr2_power_board::PowerBoardCommand::Response res_;
    boost::mutex library_lock_;
    unsigned int serial_number;
};
