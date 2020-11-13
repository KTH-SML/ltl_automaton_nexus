#!/usr/bin/env python
import rospy
import math
from rospy.msg import AnyMsg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped
from tf.transformations import euler_from_quaternion

#=====================================
#      Monitor agent pose and
#    returns 2D_pose_region state
#=====================================
class RegionStateMonitor(object):
    # Takes as input the region dictionary from the TS dictionary
    def __init__(self, region_dict, localization_topic):
        self.region_dict = region_dict

        self.init_params()

        # Setup pose callback
        self.setup_pub_sub(localization_topic)

    #-----------------
    # Init parameters
    #-----------------
    def init_params(self):
        # Generate list of station regions
        self.stations = get_stations(self.region_dict)
        # Generate list of square regions
        self.squares = get_squares(self.region_dict)

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
        if self.curr_reg:
            # Check if current region was left (using hysteresis)
            if (self.region_dict["nodes"][self.curr_reg]["attr"]["type"] == "station"):
                agent_has_left = self.is_in_station(pose, self.region_dict["nodes"][self.curr_reg],
                                                          self.region_dict["nodes"][self.curr_reg]["attr"]["dist_hysteresis"],
                                                          self.region_dict["nodes"][self.curr_reg]["attr"]["angle_hysteresis"])
            else:
                agent_has_left = self.is_in_square(pose, self.region_dict["nodes"][self.curr_reg],
                                                         self.region_dict["nodes"][self.curr_reg]["attr"]["hysteresis"])

            # If agent has left current regions:
            if agent_has_left:
                # Check all connected station
                if self.update_curr_reg(pose, self.region_dict["nodes"][self.curr_reg]["connected_to"].keys()):
                    return True

        # If not found in connected regions or no previous region, check all regions
        else:
            if self.update_curr_reg(pose, self.region_dict["nodes"].keys()):
                return True

        # Return false if region could not be found from pose
        return False

    #-----------------------
    # Update current region
    #-----------------------
    # Go through all regions given by a key list and returns true is pose is in region
    # Starting with station to give priorities for stations over squares
    def update_curr_reg(self, pose, region_keys):
        # Check stations first
        for reg in region_keys:
            if reg in self.stations:
                if self.is_in_station(pose, reg):
                    self.curr_reg = reg
                    return True
        # Then check squares
        for reg in region_keys:
            if reg in self.squares:
                if self.is_in_square(pose, reg):
                    self.curr_reg = reg
                    return True

    #--------------------------------------------------------------------------
    # Filter keys by returning only "station" regions from a region dictionary
    #--------------------------------------------------------------------------
    def get_stations(self, region_dict):
        stations = []
        for reg in region_dict:
            if (region_dict["nodes"][reg]["attr"]["type"] == "station"):
                stations.append(reg)
        return stations

    #-------------------------------------------------------------------------
    # Filter keys by returning only "square" regions from a region dictionary
    #-------------------------------------------------------------------------
    def get_squares(self, region_dict):
        squares = []
        for reg in region_dict:
            if (region_dict["nodes"][reg]["attr"]["type"] == "square"):
                squares.append(reg)
        return squares

    #------------------------------------
    # Check if pose is in a given square
    #------------------------------------
    # square dict format: {connected_to: {}, attr: {type, pose, length, hysteresis}}
    # Only position is checked in square, not orientation
    def is_in_square(self, pose, square, hysteresis = 0):
        square_pose = square["attr"]["length"]
        square_side_length = square["attr"]["pose"]

        dist_x = abs(square_pose[0] - pose.position.x)
        dist_y = abs(square_pose[0] - pose.position.y)

        # If distance to center on both x and y is inferior to half of the square side lenght plus hysteresis, agent is in region
        if (dist_x/2 < square_side_length + hysteresis) and (dist_y/2 < square_side_length + hysteresis):
            return True
        else:
            return False

    #-------------------------------------
    # Check if pose is in a given station
    #-------------------------------------
    # square dict format: {connected_to: {}, attr: {type, pose, radius, angle_threshold, dist_hysteresis, angle_hysteresis}}
    # Station is a disk, with orientation being checked as well
    def is_in_station(self, pose, station, dist_hysteresis = 0, angle_hysteresis = 0):
        station_pose = station["attr"]["pose"]
        station_radius = station["attr"]["radius"]
        angle_threshold = station["attr"]["angle_threshold"]

        dist = self.dist_2d_err(pose, station_pose)
        angle = self.yaw_angle_err(pose, station_pose)

        # If distance is inferior to radius plus hysteresis,
        # or if angle is inferior to threshold plus hysteresis, agent is in of region
        if (dist < radius + dist_hysteresis) and (angle < angle_threshold + angle_hysteresis):
            return True
        else:
            return False
        
    #-------------------------------
    # Compute 2D euclidian distance
    # between a pose and a pose msg
    #-------------------------------
    # Pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def dist_2d_err(pose, center_pose):
        return math.sqrt((center_pose[0][0] - pose.position.x)**2
                       + (center_pose[0][1] - pose.position.y)**2)

    #-------------------------------
    #  Compute angle diff (in rad)
    # between a pose and a pose msg
    #-------------------------------
    # Pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def yaw_angle_err(pose, center_pose):
        # Get euler angle from pose quaternion
        (roll, pitch, yaw) = euler_from_quaternion([pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w])
        return abs(center_yaw - center_pose[1][0])

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