#!/usr/bin/env python
import rospy

from ltl_automaton_msgs.srv import ClosestState

#=====================================
#      Monitor nexus load and
#         return load state
#=====================================
class NexusLoadStateMonitor(object):
	    def __init__(self):
        self.state=None

        self.init_params()

        # Setup pose callback
        self.setup_pub_sub()

    #-----------------
    # Init parameters
    #-----------------
    def init_params(self):
        # Get region dict of transition system from textfile parameter
        self.load_def_dict = import_ts_from_file(rospy.get_param('transition_system_textfile'))['state_models']['nexus_load']

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def setup_pub_sub(self):
        # Setup subscriber to nexus load state
        self.nexus_load_sub = rospy.Subscriber("nexus_load_sensor", Bool, self.load_state_callback, i, queue_size=100)

        # Publisher of current region
        self.current_load_state_pub = rospy.Publisher("current_load_state", String, latch=True, queue_size=10)

        # Initialize closest state service
        self.closest_state_srv = rospy.Service('closest_load_state', ClosestState, self.closest_state_callback)

    #---------------------------------------
    # Publish load state from sensor output
    #---------------------------------------
    def load_state_callback(self, msg, id):
        if msg.data:
            self.state = "loaded"
        else:
            self.state = "unloaded"

        self.current_load_state_pub.publish(self.state)

    #------------------------------
    # Handle closest state request
    #------------------------------
    def closest_state_callback(self, req):
        res = ClosestStateResponse()
        # Reply if state is known
        if self.state:
            # From current state, find would to be state in TS
            for connected_state in self.load_def_dict["nodes"][self.state]["connected_to"].keys():
                # If connected state action is requested action, add to closest
                if self.load_def_dict["nodes"][self.state]["connected_to"][connected_state] == req.action_input:
                    res.closest_state = str(connected_state)

            # Publish response. If no closest found, returns empty
            return res


#============================
#            Main            
#============================
if __name__ == '__main__':
    rospy.init_node('nexus_load_monitor',anonymous=False)
    nexus_load_monitor = NexusLoadStateMonitor()
    rospy.spin()