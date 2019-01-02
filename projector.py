#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import Queue
import sys
import rospy
import math
import rospkg
import yaml

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
    Quaternion,
    PoseWithCovariance,
    Twist,
    TransformStamped,
    Pose2D,
)
import numpy as np
from tf import transformations
import tf2_ros
import time
from multirobot_costmap_projector.msg import PublicFootprint

def sign (x):
    if x < 0 : 
        return -1 
    else :
        return 1 

#----- Load parameters ------# 

ros_package = rospkg.RosPack()
param_path = ros_package.get_path('robot_unique_parameters')
param_path += '/service_setting/service_setting.yaml'
f = open(param_path, 'r')
params_raw = f.read()
f.close()
robot_id = yaml.load(params_raw)['AMR_ID']

# robot_id = "AMR250_4" # Should Load at robot_unque parameters 
map_id = "1F" # Should get from navi_center 

footprint= [[-0.57,0.36],[0.57,0.36],[0.57,-0.36],[-0.57,-0.36]]
resolution = 0.05

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
def expand_footprint(footprint_transformed):
    '''
    Input : 
        footprint_transformed = [[x1 , y1] , [x2, y2] , [x3, y3], [x4, y4] ....] : Footprint that want to expand into polygon
    
    '''
    output = []
    for i in range(len(footprint)): # For every polygon edge
        #------ Get vertex end and start point --------# 
        vertex_start  = footprint_transformed[i]
        if i == len(footprint) - 1 :  # Last line 
            vertex_end = footprint_transformed[0]
        else:
            vertex_end = footprint_transformed[i+1]
        
        #########################
        ####   Vector Step   ####
        #########################
        '''
        #----- Cal slope ----#
        dx = vertex_end[0] - vertex_start[0]
        dy = vertex_end[1] - vertex_start[1]
        if dx == 0:
            dx = 0.01
        slope = dy / dx
        #----- Cal dx ,dy direction ------#
        if dx < 0 : 
            x_dir = -1
        else :
            x_dir = 1
        if dy < 0 : 
            y_dir = -1
        else:
            y_dir = 1
        print "slope: ", slope
        print "dx" , dx
        print "dy" , dy
        #----- Cal num point in line ------#
        num_point_in_line = int(round(math.sqrt(dx*dx + dy*dy) / resolution))

        #----- Cal step ------#
        step_x = math.sqrt(resolution * resolution / (1+ slope*slope))
        step = ( step_x * x_dir, step_x* abs(slope) * y_dir)

        for i in range(num_point_in_line+1):
            output.append([vertex_start[0] + step[0] * i , vertex_start[1] + step[1] * i])
        '''
        #####################################
        ###   Bresenham's line algorithm  ###
        #####################################
        deltax = vertex_end[0] - vertex_start[0]
        deltay = vertex_end[1] - vertex_start[1]
        if deltax == 0: # To avoid inf slope
            deltax = 0.01

        deltaerr = abs(deltay / deltax) # slope
        error = 0.0 # No error at start
        
        if deltaerr <= 1: # 0~45 degree
            y = vertex_start[1]
            for i in range(int(round(abs(deltax) / resolution, 0))):
                x = vertex_start[0] + i * resolution * sign(deltax)
                output.append([x,y]) 
                error = error + deltaerr * resolution
                if error >= 0.5 * resolution:
                    y = y + sign(deltay) * resolution
                    error = error - 1 * resolution
        else:  # 45~90 degree
            x = vertex_start[0]
            deltaerr = 1 / deltaerr
            for i in range(int(round(abs(deltay) / resolution, 0))):
                y = vertex_start[1] + i * resolution * sign(deltay)
                output.append([x,y])
                error = error + deltaerr * resolution
                if error >= 0.5 * resolution:
                    x = x + sign(deltax) * resolution
                    error = error - 1 * resolution
    return output 
    
def transform_2_translation_quaternion(transform_m):
    """
    This function help transfer the geometry_msgs.msg.TransformStamped
    into (translation, quaternion) <-- lists
    """
    # trans_v = transform_m.transform.translation
    # quat_v = transform_m.transform.rotation
    #
    trans = [transform_m.transform.translation.x, transform_m.transform.translation.y, transform_m.transform.translation.z]
    quaternion = [transform_m.transform.rotation.x, transform_m.transform.rotation.y, transform_m.transform.rotation.z, transform_m.transform.rotation.w]
    # trans = np.array([transform_m.transform.translation.x, transform_m.transform.translation.y, transform_m.transform.translation.z])
    # quaternion = np.array([transform_m.transform.rotation.x, transform_m.transform.rotation.y, transform_m.transform.rotation.z, transform_m.transform.rotation.w])
    return (trans, quaternion)

def get_pose2D_base_in_map(map_frame, base_frame):
    """
    This is the fuction for getting the robot pose (/base_foorprint) referring to /map
    The function is mainly for global-location initialization.
    """
    # Get pose_2D from tf to reduce the effect of delay

    # [x, y, theta].'
    pose_2D = np.zeros((3,1))
    #
    # Try tf
    try:
    #if True:
        # From /map to /base_footprint

        # For outputing the stamp
        # stamp_amclPose = self.goal_stamp # self.tf_buffer.getLatestCommonTime(self.map_frame, self.base_frame)
        # stamp_amclPose = rospy.Time.now()
        # print "~~~ Delay of the amcl_pose: ", (rospy.Time.now() - stamp_amclPose).to_sec(), "sec."
        #
        
        transform_m = tf_buffer.lookup_transform(map_frame, base_frame, rospy.Time(0))
        #

        """
        now = rospy.Time.now()
        transform_m = self.tf_buffer.lookup_transform(self.map_frame,self.base_frame, now, rospy.Duration(1.0))
        """
        """
        # last_odom = rospy.Time.now()
        last_odom = self.tf_buffer.getLatestCommonTime(self.base_frame, self.odom_frame)
        last_amcl_update = self.tf_buffer.getLatestCommonTime(self.odom_frame, self.map_frame) # Note that its from /usb_cam to /tag_xx
        transform_m = self.tf_buffer.lookup_transform_full(self.map_frame, last_amcl_update,
                                                                    self.base_frame, last_odom,
                                                                    self.odom_frame,
                                                                    rospy.Duration(0.1))
        stamp_amclPose = last_odom
        """
        
        # Translation of data types
        (trans, quaternion) = transform_2_translation_quaternion(transform_m)
        # end Translation of data types

        #
        pose_2D[0,0] = trans[0] # self._amcl_pose.position.x
        pose_2D[1,0] = trans[1] # self._amcl_pose.position.y
        # pose = self._amcl_pose
        # quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        
    #else:
    except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        # print "tf exception while getting robot pose"
        rospy.loginfo("[projector] tf exception while getting robot pose.")
        #
        return None
    #
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    pose_2D[2,0] = euler[2] # yaw
    # Reduced-order covariance matrix
    # cov_2D = np.dot(np.dot(self._T_subState, self._amcl_cov), np.transpose(self._T_subState) )
    #
    return (pose_2D)#  stamp_amclPose)
def digitalize(v_in, resolution = resolution):
    '''
    resolution : map_resolution (m)
    Input: v_in 
    Output : v_dig
    '''
    # print "[digitalize] " , round(v_in/resolution, 0)
    v_in -= resolution/2
    return round(round(v_in/resolution, 0) *resolution + resolution/2, len(str(resolution))-1)


def main(args):
    rospy.init_node('projector')
    '''
    #----------   Parameters Loading  ------------#
    # LOCAL paramters loading from package /param
    #param_path = rospy.get_param("~param_path")
    #f = open(param_path,'r')
    #params_raw = f.read()
    #f.close()
    #param_dict = yaml.load(params_raw)

    # GLOBAL paramters loading from rosparam server
    is_parameters_set = False
    while not is_parameters_set:
        try:
            #Find all param relative to the namespace.
            param_dict.update(rospy.get_param(rospy.get_name())) # "/projector"
            is_parameters_set = True
        except:
            rospy.loginfo("projector parameters are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue

    #Delete trash in param_dict
    # del param_dict['param_path']
    #----------   Parameters Loading  Finish ------------#
    '''
    # Conversions of parameters
    # List of tag
    # param_dict['tag_pose_id'] = np.array(param_dict['tag_pose_id'])


    # Intialize the DockingNavigation object
    # docking_navigation = DockingNavigation(param_dict)
    projectBlockPub = rospy.Publisher('/move_base/global_costmap/fake_obstacle_layer/markedPoses', PoseArray, queue_size=1)
    projectSyncPub  = rospy.Publisher('/multirobot/position', PublicFootprint, queue_size=1)
    
    # Call process at 10Hz
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pos_arr = PoseArray()
        rc = get_pose2D_base_in_map("map", "base_footprint")
        if rc == None : 
            r.sleep()
            continue
        #----publish to sync -----#
        #pos = PoseStamped()
        #pos.pose.position.x = [0,0]
        #pos.pose.position.y = [1,0]
        #projectSyncPub.publish(pos)
        # print rc[0,0] , rc[1,0], rc[2,0]
        #--------------  Rotate and translate footprint --------------#
        rotation_matrix = np.matrix([ [math.cos(rc[2,0]), -math.sin(rc[2,0])],  [math.sin(rc[2,0]), math.cos(rc[2,0])] ])
        translation_matrix =  np.matrix([[rc[0,0]], [rc[1,0]]])
        # print rotation_matrix
        footprint_transformed = []
        footprint_sync = PublicFootprint()
        for i in footprint:
            input_matrix = np.matrix([[i[0]] , [i[1]]]) # ( x , y )
            # print input_matrix
            ### Calcuate
            output = (rotation_matrix * input_matrix  + translation_matrix).A1 # Flatten result
            tmp = Pose2D()
            tmp.x = output[0]
            tmp.y = output[1]
            footprint_sync.footprints.append(tmp)
            footprint_transformed.append([digitalize(output[0]) , digitalize(output[1])])
        #--- Add header to publicFootprint ----# 
        footprint_sync.header.stamp = rospy.Time.now()
        footprint_sync.header.frame_id = 'map'
        footprint_sync.map_id = map_id
        footprint_sync.robot_id = robot_id
        projectSyncPub.publish(footprint_sync)

        print "footprint_transformed: ", footprint_transformed
        #-----------  expand_footprint  -------------#
        t_start = time.time()
        footprint_expand_list = expand_footprint(footprint_transformed)
        print "Time Used: " , str(time.time() - t_start)
        # ----- Draw footprint on global_costmap -------# 
        for i in footprint_expand_list:
            tmp = Pose()
            tmp.position.x  = i[0]
            tmp.position.y  = i[1]
            tmp.position.z  = 0
            pos_arr.poses.append(tmp)
        projectBlockPub.publish(pos_arr)
        r.sleep()
    # Done
if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
