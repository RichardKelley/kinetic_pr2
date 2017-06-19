# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from python_qt_binding.QtCore import QSize
from rqt_robot_dashboard.widgets import BatteryDashWidget


class PR2Battery(BatteryDashWidget):
    """
    Dashboard widget to display PR2 battery state.
    """
    #TODO When nonbutton Dashboard objects are available rebase this widget
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        # Use green icons above 60 percent
        icons = []
        charge_icons = []
        icons.append(['ic-battery-0.svg'])
        icons.append(['ic-battery-20.svg'])
        icons.append(['ic-battery-40.svg'])
        icons.append(['ic-battery-60-green.svg'])
        icons.append(['ic-battery-80-green.svg'])
        icons.append(['ic-battery-100-green.svg'])
        charge_icons.append(['ic-battery-charge-0.svg'])
        charge_icons.append(['ic-battery-charge-20.svg'])
        charge_icons.append(['ic-battery-charge-40.svg'])
        charge_icons.append(['ic-battery-charge-60-green.svg'])
        charge_icons.append(['ic-battery-charge-80-green.svg'])
        charge_icons.append(['ic-battery-charge-100-green.svg'])
        super(PR2Battery, self).__init__('PR2 Battery', icons=icons, charge_icons=charge_icons)

        self._power_consumption = 0.0
        self._pct = 0
        self._time_remaining = rospy.rostime.Duration(0)
        self._ac_present = 0
        # use inherited function instead of accessing variable
        self.set_charging(False)

        # use default size and set margin to 0 to display the full battery symbol
        self.setFixedSize(self._icons[1].actualSize(QSize(60, 100)))
        self.setMargin(0)

        self.update_perc(0)

    def set_power_state(self, msg):
        """
        Sets button state based on msg

        :param msg: message containing the power state of the PR2
        :type msg: pr2_msgs.PowerState
        """
        # unset stale status, else the battery will stick to the stale loop, the unset_stale function is never called elsewhere
        self.unset_stale()
        last_pct = self._pct
        # self._plugged in has been changed to self._charging
        last_charging = self._charging
        last_time_remaining = self._time_remaining

        self._power_consumption = msg.power_consumption
        self._time_remaining = msg.time_remaining
        self._pct = msg.relative_capacity / 100.0
        # use inherited function instead of accessing variable
        self.set_charging(msg.AC_present)
        if (last_pct != self._pct or last_charging != self._charging or last_time_remaining != self._time_remaining):
            drain_str = "remaining"
            if (self._charging):
                drain_str = "to full charge"
                # use battery name varibale
                self.setToolTip("%s: %.2f%% \nTime %s: %d Minutes" % (self._name, self._pct * 100.0, drain_str, self._time_remaining.to_sec() / 60.0))
            self.update_perc(msg.relative_capacity)

    def set_stale(self):
        # use super function of set_stale instead of copy/paste
        super(PR2Battery, self).set_stale()
        self._time_remaining = rospy.rostime.Duration(0)
        self._power_consumption = 0
