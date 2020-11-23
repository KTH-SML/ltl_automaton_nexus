#!/usr/bin/env python
import rospy
import sys
import yaml
import std_msgs
from copy import deepcopy
#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemState
# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# Import modules for commanding the nexus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

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
        self.curr_ltl_state = [None, None]
        self.prev_ltl_state = [None, None]

        self.agent_name = rospy.get_param('agent_name', "nexus")

        # Get TS from param
        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Init state message with TS
        self.ltl_state_msg = TransitionSystemState()
        self.ltl_state_msg.state_dimension_names = self.transition_system["state_dim"]

        # Initialize running time and index of command received and executed
        self.t0 = rospy.Time.now()
        self.t = self.t0
        self.plan_index = 0

        # Setup navigation commands for nexus
        self.navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #-------------------------------------------------------------------------
    # Create monitoring object for every state dimension in transition system
    #-------------------------------------------------------------------------
    def create_monitors(self):
        for i in range(len(self.transition_system["state_dim"])):
            print "checking dimension states"
            dimension = self.transition_system["state_dim"][i]
            print dimension
            if (dimension == "2d_pose_region"):
                # Setup subscriber to 2D pose region monitor
                self.nexus_region_sub = rospy.Subscriber("current_region", String, self.region_state_callback, i, queue_size=100)
            elif (dimension == "nexus_load"):
                # Setup subscriber to nexus load state
                self.nexus_load_sub = rospy.Subscriber("nexus_load_sensor", Bool, self.load_state_callback, i, queue_size=100)
            else:
                raise ValueError("state type [%s] is not supported by LTL Nexus" % (dimension))

    def set_pub_sub(self):
        # Setup LTL state publisher
        self.ltl_state_pub = rospy.Publisher("ts_state", TransitionSystemState, latch=True, queue_size=10)

        # Setup subscriber to ltl_automaton_core next_move_cmd
        self.next_move_sub = rospy.Subscriber("next_move_cmd", std_msgs.msg.String, self.next_move_callback, queue_size=1)

    def load_state_callback(self, msg, id):
        if msg.data:
            self.curr_ltl_state[id] = "loaded"
        else:
            self.curr_ltl_state[id] = "unloaded"

    def region_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    def next_move_callback(self, msg):
        '''Recieve next_move_cmd from ltl_automaton_core planner and convert into robot action to implement'''

        # Update running time and augment plan index
        self.t = rospy.Time.now()-self.t0
        self.plan_index += 1

        # Extract command message string
        cmd_str =  msg.data
        action_dict = None

        # Check if next_move_cmd is 'None', which is output by ltl_automaton_core if the current state is not in the TS
        if cmd_str == "None":

            # To do: Handle when ltl_automaton_core encounteres state outside of TS (i.e. next_move_cmd = 'None')
            rospy.logwarn('None next_move_cmd sent to LTL Nexus')
        else:

            # Check if next_move_cmd is in list of actions from transition_system
            for act in self.transition_system['actions']:
                if str(act) == cmd_str:

                    # Extract action types, attributes, etc. in dictionary
                    action_dict = self.transition_system['actions'][str(act)]

                    break

            # Raise error if next_move_cmd does not exist in transition system
            if not(action_dict):
                raise ValueError("next_move_cmd not found in LTL Nexus transition system")

        # Send action_dict to nexus_action()
        self.nexus_action(action_dict)

    def nexus_action(self, act_dict):
        '''Read components of act_dict associated with current command and output control to nexus'''    

        if act_dict['type'] == 'move':
            # Extract pose to move to:
            pose = act_dict['attr']['pose']

            # Set new navigation goal and send
            GoalMsg = MoveBaseGoal()
            GoalMsg.target_pose.header.seq = self.plan_index
            GoalMsg.target_pose.header.stamp = self.t
            GoalMsg.target_pose.header.frame_id = 'map'
            GoalMsg.target_pose.pose.position.x = pose[0][0]
            GoalMsg.target_pose.pose.position.y = pose[0][1]
            #quaternion = quaternion_from_euler(0, 0, goal[2])
            GoalMsg.target_pose.pose.orientation.x = pose[1][1]
            GoalMsg.target_pose.pose.orientation.y = pose[1][2]
            GoalMsg.target_pose.pose.orientation.z = pose[1][3]
            GoalMsg.target_pose.pose.orientation.w = pose[1][0]
            self.navigation.send_goal(GoalMsg)

        #if act_dict['type'] == 'nexus_pick':
            # TO DO
            

        #if act_dict['type'] == 'nexus_drop':
            # TO DO

    def main_loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # If current state is different from previous state
            # update message and publish it
            if not (self.curr_ltl_state == self.prev_ltl_state):
                # Update previous state
                self.prev_ltl_state = deepcopy(self.curr_ltl_state)
                # If both states are initialized (not None), publish message
                if (self.curr_ltl_state[0] and self.curr_ltl_state[1]):
                    # Publish msg
                    self.ltl_state_msg.header.stamp = rospy.Time.now()
                    self.ltl_state_msg.states = self.curr_ltl_state
                    self.ltl_state_pub.publish(self.ltl_state_msg)
                

            #rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
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
    