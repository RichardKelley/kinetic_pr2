#!/usr/bin/env python

PKG = 'pr2_power_board' # this package name
NAME = 'fake_powerboard'

import roslib; roslib.load_manifest(PKG)

import sys
import time

import rospy
from pr2_msgs.msg import *

from optparse import OptionParser

import threading
from pr2_power_board.srv import PowerBoardCommand

class PowerBoard(threading.Thread):
  def __init__(self, options):
    threading.Thread.__init__(self)
    self.setDaemon(True)
    
    self.options = options

    self.pb = PowerBoardState()
    self.pb.header.stamp = rospy.rostime.get_rostime()
    self.pb.serial_num = 9999
    self.pb.input_voltage = 80.123456
    self.pb.circuit_state = [3,3,3]
    self.pb.circuit_voltage = [70.123456,70.234567,70.345678]
    if self.options.run_stop: self.pb.run_stop = 1
    else: self.pb.run_stop = 0
    if self.options.wireless_stop: self.pb.wireless_stop = 1
    else: self.pb.wireless_stop = 0

    self.pub = rospy.Publisher("power_board/state", PowerBoardState)

  def power_board_control(self, msg):
    if msg.command == "disable":
      self.pb.circuit_state[msg.breaker_number] = 0
      self.pb.circuit_voltage[msg.breaker_number] = 1
    if msg.command == "stop":
      if self.pb.circuit_state[msg.breaker_number] != 0:
        self.pb.circuit_state[msg.breaker_number] = 1
        self.pb.circuit_voltage[msg.breaker_number] = 19
    if msg.command == "reset":
      if self.pb.circuit_state[msg.breaker_number] == 0:
        self.pb.circuit_state[msg.breaker_number] = 1
    if msg.command == "start":
      if self.pb.circuit_state[msg.breaker_number] != 0:
        self.pb.circuit_state[msg.breaker_number] = 3
        self.pb.circuit_voltage[msg.breaker_number] = 70
  
  def run(self):
    while 1:
      self.pub.publish(self.pb)
      time.sleep(1)

def talker(options):
  rospy.init_node(NAME, anonymous=True)
  pb = PowerBoard(options)
  s = rospy.Service('power_board/control', PowerBoardCommand, pb.power_board_control)

  pb.start()
  rospy.spin()


if __name__ == '__main__':
  parser = OptionParser()
  parser.add_option('-r', '--run_stop', action='store_false', default=True)
  parser.add_option('-w', '--wireless_stop', action='store_false', default=True)
  (options, args) = parser.parse_args()

  try:
    talker(options)
  except KeyboardInterrupt, e:
    pass
  print "exiting"

