#!/usr/bin/env python
import rospy
import sys
import yaml
#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemState
# Import state monitors
from region_2d_pose_monitor import Region2DPoseStateMonitor
from nexus_load_monitor import NexusLoadMonitor

#=================================================================
#  Interfaces between LTL planner node and lower level controls
#                       -----------------
# The node is reponsible for outputting current agent state based
# on TS and agent output.
# The node converts TS action to interpretable commands using
# action attributes defined in the TS config file
#=================================================================
class LTLController(object):
    def __init__(self):

        self.init_params()

        self.create_monitors()

        self.set_pub_sub()

        self.main_loop()

    # Get params from ROS param server and config files
    def init_params(self):
        self.agent_name = rospy.get_param('agent_name', "nexus")

        # Get TS from param
        self.transition_system = self.import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Init state message with TS
        self.ltl_state_msg = TransitionSystemState()
        self.ltl_state_msg.state_dimension_names = self.transition_system["state_dim"]

    # Import TS and action attributes from file
    def import_ts_from_file(self, transition_system_textfile):
        try:
            # Get dictionary from yaml text file
            transition_system = yaml.load(transition_system_textfile)
            # TODO: Add a proper parse and check (dimensions, attr,...)
            return transition_system
        except:
            raise ValueError("cannot load transition system from textfile")

    #-------------------------------------------------------------------------
    # Create monitoring object for every state dimension in transition system
    #-------------------------------------------------------------------------
    def create_monitors(self):
        self.monitors = []
        for dimension in self.transition_system["state_dim"]:
            print "checking dimension states"
            print dimension
            if (dimension == "2d_pose_region"):
                region_2d_pose_monitor = Region2DPoseStateMonitor(self.transition_system["state_models"]["2d_pose_region"], self.agent_name+"_pose")
                self.monitors.append(region_2d_pose_monitor)
            elif (dimension == "nexus_load"):
                nexus_load_monitor = NexusLoadMonitor(self.agent_name+"_load")
                self.monitors.append(nexus_load_monitor)
            else:
                raise ValueError("state type [%s] is not supported by LTL Nexus" % (dimension))
                # module_name = dimension+"_state_monitor"
                # object_type = dimension.capitalize()+"StateMonitor"
                # object_name = dimension+"_state_monitor"
                # importlib.import_module(module_name, package=None)
                # exec '%s = %r()' % (object_name, object_type)
                # monitors.append(object_name)

    def set_pub_sub(self):
        # Setup LTL state publisher
        self.ltl_state_pub = rospy.Publisher("nexus_ltl_state", TransitionSystemState, latch=True, queue_size=10)

    def main_loop(self):
        self.curr_ltl_state = [None, None]
        self.prev_ltl_state = [None, None]
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Check state
            for i in range(len(self.monitors)):
                self.curr_ltl_state[i] = self.monitors[i].state

            # If current state is different from previous state
            # update message and publish it
            if (self.curr_ltl_state[0] and self.curr_ltl_state[1]) and not (self.curr_ltl_state == self.prev_ltl_state):
                rospy.loginfo("state has changed")
                # Publish msg
                self.ltl_state_msg.header.stamp = rospy.Time.now()
                self.ltl_state_msg.states = self.curr_ltl_state
                self.ltl_state_pub.publish(self.ltl_state_msg)
                # Update previous state
                self.prev_ltl_state = self.curr_ltl_state

            rospy.loginfo("State is %s" %(self.curr_ltl_state))

            rate.sleep()



#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_nexus',anonymous=False)
    try:
        ltl_nexus = LTLController()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Nexus node: %s" %(e))
        sys.exit(0)
    