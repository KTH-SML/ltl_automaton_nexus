#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

#=============================
#   Monitor nexus status and
#    returns load state
#=============================
# Load state is either:
# - loaded
# - unloaded
class NexusLoadMonitor(object):
	def __init__(self, load_sensor_topic):
		# unloaded by default
		self.state = "unloaded"

		self.set_sub_pub(load_sensor_topic)

	def set_sub_pub(self, load_sensor_topic):
		self.nexus_load_sub = rospy.Subscriber(load_sensor_topic, Bool, self.load_state_callback, queue_size=100)

	def load_state_callback(self, msg=Bool):
		if msg.data:
			self.state = "loaded"
		else:
			self.state = "unloaded"

