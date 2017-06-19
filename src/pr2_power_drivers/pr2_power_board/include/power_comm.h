/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POWER_COMM_H
#define POWER_COMM_H

static const unsigned CURRENT_MESSAGE_REVISION = 3;
#define CURRENT_MESSAGE_SIZE (sizeof(PowerMessage))
static const unsigned MINIMUM_MESSAGE_REVISION = 2;
#define REVISION_2_MESSAGE_SIZE (sizeof(MessageHeader) + sizeof(StatusStruct_Rev2))

static const unsigned TRANSITION_MESSAGE_REVISION = 2;
static const unsigned COMMAND_MESSAGE_REVISION = 2;
static const unsigned STATUS_MESSAGE_REVISION = 2;

static const unsigned MESSAGE_ID_POWER = 0;
static const unsigned MESSAGE_ID_COMMAND = 1;
static const unsigned MESSAGE_ID_TRANSITION = 2;
static const unsigned MESSAGE_ID_STATUS = 3;
static const unsigned POWER_PORT = 6801; // port power board

enum Master_State { MASTER_NOPOWER, MASTER_STANDBY, MASTER_ON, MASTER_OFF, MASTER_SHUTDOWN };
enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
enum CB_Command { NONE = 0, COMMAND_START = 1, COMMAND_STOP = 2, COMMAND_RESET = 3, COMMAND_DISABLE = 4 };

typedef struct
{
  unsigned int    message_revision; //32 bit 
  unsigned int    serial_num;       //32 bit  Unique ID number
  char            text[32];         //Description identifier
  unsigned int    message_id;
  unsigned int    data_length;      //Length of the following structure
} __attribute__((__packed__)) MessageHeader;

typedef struct
{
  //Software State  0 = default, 1=Start, 2=Stop, 3=reset, 4=disable
  unsigned char   CB0_state;
  unsigned char   CB1_state;
  unsigned char   CB2_state;
  unsigned char   DCDC_state;  // 1=on, 0=off

  //Status
  float           input_voltage;
  float           input_current;
  float           DCDC_12V_aux;
  float           DCDC_12V_cpu0;
  float           CB0_voltage;
  float           CB1_voltage;
  float           CB2_voltage;
  float           ambient_temp;
  unsigned int    fan0_speed;
  unsigned int    fan1_speed;
  unsigned int    fan2_speed;
  unsigned int    fan3_speed;
  unsigned char   CB0_status;
  unsigned char   CB1_status;
  unsigned char   CB2_status;
  unsigned char   estop_button_status;
  unsigned char   estop_status;
  unsigned char   pca_rev;
  unsigned char   pcb_rev;
  unsigned char   major_rev;
  unsigned char   minor_rev;
  float           min_input_voltage;
  float           max_input_current;
  float           DCDC_12V_cpu1;
  float           DCDC_12V_user;
  float           battery_voltage[4];
} __attribute__((__packed__)) StatusStruct;

typedef struct
{
  //Software State  0 = default, 1=Start, 2=Stop, 3=reset, 4=disable
  unsigned char   CB0_state;   // CB_State enum
  unsigned char   CB1_state;
  unsigned char   CB2_state;
  unsigned char   DCDC_state;  // Master_State enum

  //Status
  float           input_voltage;
  float           input_current;
  float           DCDC_12V_out_voltage;
  float           DCDC_19V_out_voltage;
  float           CB0_voltage;
  float           CB1_voltage;
  float           CB2_voltage;
  float           ambient_temp;
  unsigned int    fan0_speed; 
  unsigned int    fan1_speed;
  unsigned int    fan2_speed;
  unsigned int    fan3_speed;
  unsigned char   CB0_status;  //0-off 1-on
  unsigned char   CB1_status;
  unsigned char   CB2_status;
  unsigned char   estop_button_status;
  unsigned char   estop_status;
  unsigned char   pca_rev;
  unsigned char   pcb_rev;
  unsigned char   major_rev;
  unsigned char   minor_rev;
  float           min_input_voltage;
  float           max_input_current;

} __attribute__((__packed__)) StatusStruct_Rev2;

typedef struct 
{
	MessageHeader header;
	StatusStruct  status;
} __attribute__((__packed__)) PowerMessage;

#define COMMAND_FLAG_RESET_STATS            0x1   //reset main statistics
#define COMMAND_FLAG_RESET_TRANSITION_STATS 0x2   //reset the transition statistics

typedef struct
{
  unsigned char   CB0_command; //CB_Command enum
  unsigned char   CB1_command;
  unsigned char   CB2_command;
  unsigned char   DCDC_command;
  unsigned char   fan0_command;
  unsigned char   fan1_command;
  unsigned char   fan2_command;
  unsigned char   fan3_command;
  unsigned int    flags;  //see COMMAND_FLAGS above
} __attribute__((__packed__)) CommandStruct;

typedef struct
{
  MessageHeader header;
  CommandStruct command;
} __attribute__((__packed__)) CommandMessage;


typedef struct
{
  unsigned char   stop_count;
  unsigned char   estop_count;
  unsigned char   trip_count;
  unsigned char   fail_18V_count;
  unsigned char   disable_count;
  unsigned char   start_count;
  unsigned char   pump_fail_count;
  unsigned char   reset_count;
} __attribute__((__packed__)) TransitionCount;

typedef struct
{
  MessageHeader header;
  TransitionCount cb[3]; // one for each circuit breaker
} __attribute__((__packed__)) TransitionMessage;


typedef struct 
{
	MessageHeader header;
  int message_to_get;
} __attribute__((__packed__)) GetMessage;

#endif
