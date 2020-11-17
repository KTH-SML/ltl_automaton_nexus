#!/usr/bin/env python
import rospy
import math
from rospy.msg import AnyMsg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped
from roslib.message import get_message_class
from tf.transformations import euler_from_quaternion

#=====================================
#      Monitor agent pose and
#    returns 2D_pose_region state
#=====================================
class Region2DPoseStateMonitor(object):
    # Takes as input the region dictionary from the TS dictionary
    def __init__(self, region_dict, localization_topic):
        self.region_dict = region_dict
        self.state=None

        self.init_params()

        # Setup pose callback
        self.setup_pub_sub(localization_topic)

    #-----------------
    # Init parameters
    #-----------------
    def init_params(self):
        # Generate list of station regions
        self.stations = filter(lambda elem: self.region_dict["nodes"][elem]["attr"]["type"] == "station",
                               self.region_dict["nodes"].keys())
        # Generate list of square regions
        self.squares = filter(lambda elem: self.region_dict["nodes"][elem]["attr"]["type"] == "square",
                              self.region_dict["nodes"].keys())

    #----------------------------------------
    # Setup subscriber to agent localization
    #----------------------------------------
    def setup_pub_sub(self, localization_topic):
       #Subscribe to topic with any message so that callback can process any type of pose message
        rospy.Subscriber(localization_topic, AnyMsg, self.omnipose_callback)

    #-------------------------------------------------------------------
    # Check agent region using pose and update current region if needed
    #-------------------------------------------------------------------
    def check_region(self, pose):
        # If current region is known, 
        if self.state:
            print "previous state exist"

            # If current region is a station, we check that agent has left first
            #--------------------------------------------------------------------
            # Check if current region was left (using hysteresis)
            if (self.region_dict["nodes"][self.state]["attr"]["type"] == "station"):
                agent_has_left = not self.is_in_station(pose, self.state,
                                                              self.region_dict["nodes"][self.state]["attr"]["dist_hysteresis"],
                                                              self.region_dict["nodes"][self.state]["attr"]["angle_hysteresis"])

                # If agent has left current regions, check all connected regions
                if not agent_has_left:
                    return True
                else:
                    print "agent has left curr reg"
                    # Check all connected regions
                    if self.update_state(pose, self.region_dict["nodes"][self.state]["connected_to"].keys()):
                        return True
            
            # If current region is square check first if agent has enter a connected station
            #--------------------------------------------------------------------------------
            # stations are overlaid on top of squares and should be check even if agent has not left square
            else:
                # Get list of connected station and check if in it
                connected_stations_list = filter(lambda elem: elem in self.stations,
                                                 self.region_dict["nodes"][self.state]["connected_to"].keys())
                # Check if agent is in the stations
                if self.update_state(pose, connected_stations_list):
                    return True

                # If agent is not in station, check is previous square regions has been left
                agent_has_left = not self.is_in_square(pose, self.state,
                                                             self.region_dict["nodes"][self.state]["attr"]["hysteresis"])
                # If agent has left current regions, check all connected regions
                if not agent_has_left:
                    return True
                else:
                    print "agent has left curr reg"
                    # Check all connected squares
                    connected_stations_list = filter(lambda elem: elem in self.squares,
                                                     self.region_dict["nodes"][self.state]["connected_to"].keys())

                    if self.update_state(pose, self.region_dict["nodes"][self.state]["connected_to"].keys()):
                        return True


        # If not found in connected regions or no previous region, check all regions
        print "previous state does not exist or agent not in connected regions"
        if self.update_state(pose, self.region_dict["nodes"].keys()):
            return True

        # Return false if region could not be found from pose
        return False
        print "agent not found"

    #-----------------------
    # Update current region
    #-----------------------
    # Go through all regions given by a key list and returns true is pose is in region
    # Starting with station to give priorities for stations over squares
    def update_state(self, pose, region_keys):
        # Check stations first
        for reg in region_keys:
            if reg in self.stations:
                if self.is_in_station(pose, reg):
                    self.state = str(reg)
                    print "agent found in reg %s" %(reg)
                    return True
        # Then check squares
        for reg in region_keys:
            if reg in self.squares:
                if self.is_in_square(pose, reg):
                    self.state = str(reg)
                    print "agent found in reg %s" %(reg)
                    return True

    #------------------------------------
    # Check if pose is in a given square
    #------------------------------------
    # square dict format: {connected_to: {}, attr: {type, pose, length, hysteresis}}
    # Only position is checked in square, not orientation
    def is_in_square(self, pose, square, hysteresis = 0):
        print "square is"
        print square
        print "-----------"
        square_pose = self.region_dict["nodes"][square]["attr"]["pose"]
        square_side_length = self.region_dict["nodes"][square]["attr"]["length"]

        rospy.logwarn("Checking for agent in square region %s, with pose %s, side lenght %i and hysteresis %i" %(square, square_pose, square_side_length, hysteresis))
        rospy.logwarn(pose)
        rospy.logwarn("-----------")

        dist_x = abs(square_pose[0][0] - pose.position.x)
        dist_y = abs(square_pose[0][1] - pose.position.y)

        rospy.logwarn("dist x is %i and dist y is %i, AND square_side_length/2 + hysteresis is %f" %(dist_x, dist_y, (float)(square_side_length)/2 + hysteresis))

        # If distance to center on both x and y is inferior to half of the square side lenght plus hysteresis, agent is in region
        if (dist_x < (float)(square_side_length)/2 + hysteresis) and (dist_y < (float)(square_side_length)/2 + hysteresis):
            print "is in square"
            return True
        else:
            print "is not in square"
            return False

    #-------------------------------------
    # Check if pose is in a given station
    #-------------------------------------
    # station dict format: {connected_to: {}, attr: {type, pose, radius, angle_threshold, dist_hysteresis, angle_hysteresis}}
    # Station is a disk, with orientation being checked as well
    def is_in_station(self, pose, station, dist_hysteresis = 0, angle_hysteresis = 0):
        print "station is"
        print station
        print "-----------"
        station_pose = self.region_dict["nodes"][station]["attr"]["pose"]
        station_radius = self.region_dict["nodes"][station]["attr"]["radius"]
        angle_threshold = self.region_dict["nodes"][station]["attr"]["angle_threshold"]

        rospy.logwarn("Checking for agent in station region %s, with pose %s, radius %i, dist hysteresis %i, angle_hysteresis %i"
                     %(station, station_pose, station_radius, dist_hysteresis, angle_hysteresis))
        rospy.logwarn(pose)
        rospy.logwarn("-----------")


        dist = self.dist_2d_err(pose, station_pose)
        angle = self.yaw_angle_err(pose, station_pose)

        rospy.logwarn("dist is %i and angle is %i" %(dist, angle))

        # If distance is inferior to radius plus hysteresis,
        # or if angle is inferior to threshold plus hysteresis, agent is in of region
        if (dist < station_radius + dist_hysteresis) and (angle < angle_threshold + angle_hysteresis):
            print "True"
            return True
        else:
            print "False"
            return False
        
    #-------------------------------
    # Compute 2D euclidian distance
    # between a pose and a pose msg
    #-------------------------------
    # Pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def dist_2d_err(self, pose, center_pose):
        return math.sqrt((center_pose[0][0] - pose.position.x)**2
                       + (center_pose[0][1] - pose.position.y)**2)

    #-------------------------------
    #  Compute angle diff (in rad)
    # between a pose and a pose msg
    #-------------------------------
    # Pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def yaw_angle_err(self, pose, center_pose):
        # Get euler angle from pose quaternion
        (roll, pitch, yaw) = euler_from_quaternion([pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w])
        return abs(yaw - center_pose[1][0])

    #-------------------------------------
    #        callback function for
    #      the pose of each obstacle
    #-------------------------------------
    def omnipose_callback(self, anymsg):
        # Callback is allowed to subscribe with any message, so it can subscribe to 
        # both Pose and PoseWithCovarianceStamped message types.
        # The message type is checked and the message is deserialized in the proper way. It is then handled in an appropriate way.
        try:
            message_type = anymsg._connection_header['type']
            msg_class = get_message_class(message_type)
            pose_msg = msg_class().deserialize(anymsg._buff)

            rospy.logwarn("Message received is:")
            rospy.logwarn(pose_msg)
            rospy.logwarn("-----------")

            #If message type is geometry_msgs/Pose
            if (message_type == 'geometry_msgs/Pose'):
                #Check pose for region
                self.check_region(pose_msg)

            #If message type is geometry_msgs/PoseWithCovariance
            elif (message_type == 'geometry_msgs/PoseWithCovariance'):
                #Check pose for region
                self.check_region(pose_msg.pose)

            #If message type is geometry_msgs/PoseStamped
            elif (message_type == 'geometry_msgs/PoseStamped'):
                #Check pose for region
                self.check_region(pose_msg.pose)

            #If message type is geometry_msgs/PoseWithCovarianceStamped
            elif (message_type == 'geometry_msgs/PoseWithCovarianceStamped'):
                #Check pose for region
                self.check_region(pose_msg.pose.pose)
            #Else if message is not of a supported type
            else:
                rospy.logerr('Region state monitor: message type '+message_type+' is not supported for 2D pose region monitoring')
        except ValueError as e:
            rospy.logerr('Region state monitor: could not process incoming message: ' + e)