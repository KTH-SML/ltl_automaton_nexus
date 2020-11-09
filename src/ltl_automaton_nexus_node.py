#!/usr/bin/env python
import rospy

# Interfaces between LTL planner node and lower level controls
# The node is reponsible for outputting current agent state based
# on TS and agent output.
# The node converts TS action to interpretable commands using
# action attributes defined in the TS config

class LTLController(object):
	def __init__(self):

	# Get params from ROS param server and config files
	def init_params(self):
		#TODO: express ts file location as a parameter
		#TODO: express LTL formula as parameters (strings)

	# Import TS and action attributes from file
	def import_ts_from_file(self):


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_nexus',anonymous=False)
    ltl_nexus = LTLController()
    rospy.spin()