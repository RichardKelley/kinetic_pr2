#!/usr/bin/env python
import roslib; roslib.load_manifest("pr2_power_board") 

import sys
import os
import string

import rospy
from pr2_power_board.srv import *
from pr2_msgs.msg import PowerBoardState

current_state = PowerBoardState()

def callback(data):
  global current_state
  current_state = data
  #print "New State: %d" % ( current_state.circuit_state[1] )

if __name__ == "__main__":

  # block until the add_two_ints service is available
  rospy.wait_for_service('power_board/control', 5)

  rospy.init_node('test_power')

  # create a handle to the add_two_ints service
  control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
  state = rospy.Subscriber("/power_board/state", PowerBoardState, callback)

  pause_on_fail = 0
  breaker_number = 1  # The breaker we are toggling
  other_breaker1 = 0
  other_breaker2 = 2
  command_list = ["start", "stop"]
  serial = 0
  flags = 0
  try_count = 1000
  delay = rospy.Duration(3.0)
  fail_count = 0
  last_fail_count = 0

  while (try_count > 0):
    try:
        command = command_list[ try_count % len(command_list) ]
        print "%d: Requesting %d to %s"%(try_count, breaker_number, command)
        
        # simplified style
        resp1 = control(serial, breaker_number, command, flags)
        print "    Response: ", resp1
        if resp1.retval != 0:
          fail_count = fail_count + 1
          print "Response to control is bad"

        rospy.sleep(delay)
        for x in current_state.circuit_state:
          print "Current State: %d" % ( x )

        if (command == "start") and ( current_state.circuit_state[breaker_number] != PowerBoardState.STATE_ON ):
          fail_count = fail_count + 1
          print "FAIL: Test circuit did not turn ON"

        if (command == "stop") and ( current_state.circuit_state[breaker_number] != PowerBoardState.STATE_STANDBY ):
          fail_count = fail_count + 1
          print "FAIL: Test circuit not in STANDBY"

        if ( current_state.circuit_state[other_breaker1] != PowerBoardState.STATE_ON ) or ( current_state.circuit_state[other_breaker2] != PowerBoardState.STATE_ON ):
          fail_count = fail_count + 1
          print "FAIL: The other circuit is not ON"

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
 

    try_count = try_count - 1
    if pause_on_fail and fail_count > last_fail_count:
      print "Pause on Fail"
      line = sys.stdin.readline()
      last_fail_count = fail_count
 
  print "\n"
  print "Test complete: "
  print "  Test breaker_number = %d" % (breaker_number)
  print "  Fail Count = %d " % (fail_count)
  print "\n"

