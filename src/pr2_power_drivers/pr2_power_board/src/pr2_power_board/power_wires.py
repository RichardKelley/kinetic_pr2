#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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
#

#import roslib
#roslib.load_manifest(PKG)

import sys
import rospy
from diagnostic_msgs.msg import *


def recurse_tree(element, messages, wiremap):
    errors = []
    print "Looking at ", element
    if element in wiremap:
        if "children" in wiremap[element]:
            for child in wiremap[element]["children"]:
                errors_return = recurse_tree(child, messages, wiremap)
                errors.extend(errors_return)

                try:
                    value = float(messages[ wiremap[element]['component']][wiremap[element]['value']])
                    child_value = float(messages[ wiremap[child]['component']][wiremap[child]['value']])
                    tolerance = wiremap[element]['tolerance']
                    if abs(value - child_value) / value > tolerance/100.0:
                        errors.append("difference between %f (%s) and %f (%s) voltages exceeds tolerance %f percent"%(value, element, child_value, child, tolerance))
                    else:
                        rospy.logdebug("%s passed"%child)
                except KeyError, e:
                    errors.append("badly formed parameters for element %s: %s"%(element, e));
        else:
            print "No children of element: ", element
    else:
        errors.append("no element %s"% element)
        #print "wiremap is", wiremap
    return errors


def test(latest_status, parameters):
    #print latest_status
    results = {}

    if "wiring_tree" in parameters:
        wiremap = parameters["wiring_tree"]
    else:
        results['error'] = ["power_wires: no wiring_tree found"]
        return results

    if 'root' in parameters:
        results['error'] = []
        for root in parameters["root"]:
            results['error'].extend( recurse_tree(root, latest_status, wiremap))
        # clean up if no iterations happend
        if results['error'] == []:
            del results['error']
    else:
        results['error'] = ["power_wires: no root found"]
        return results


    return results

    
