#!/usr/bin/env python
import Queue
import sys
import rospy
import math
import rospkg
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    PolygonStamped,
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
from multirobot_costmap_projector.msg import PublicFootprint # self-define msg type

def sign (x):
    '''
    Utility Function 
    '''
    if x < 0 : 
        return -1 
    else :
        return 1 

# map_id = "1F" # TODO Should get from navi_center topic /current_floor 

# footprint= [[-0.57,0.36],[0.57,0.36],[0.57,-0.36],[-0.57,-0.36]]
# footprint_subcribe = None 
# resolution = 0.05 # map resolution 


class COSTMAP_PROJECTOR():
    def __init__(self):
        # TF2 
        self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Inputs
        self.robot_id = self.getRobotID() 
        self.map_id = None #'1F' # TODO Should get from navi_center topic /current_floor 
        self.resolution = 0.05 

        # Publish
        self.projectSyncPub  = rospy.Publisher('/multirobot/position', PublicFootprint, queue_size=1)
        self.projectBlockPub = rospy.Publisher('/move_base/global_costmap/fake_obstacle_layer/markedPoses', PoseArray, queue_size=1)

        # Container
        self.footprint = PublicFootprint() # Subscribe to /move_base/local_map/footprint
        self.object_to_project = {} # dict(), = {'AMR250#4' : 'PublishFootprint', 'AMR250#5' : 'PublishFootprint', Navi_center_fake_projection }

    def getRobotID(self):
        '''
        retrun robot_id 
        '''
        ros_package = rospkg.RosPack()
        param_path = ros_package.get_path('robot_unique_parameters')
        param_path += '/service_setting/service_setting.yaml'
        f = open(param_path, 'r')
        params_raw = f.read()
        f.close()
        return yaml.load(params_raw)['AMR_ID'] # Load at service_setting.yaml

    def expand_footprint(self,pos_arr , polygon):
        '''
        Transform points -> line 
        Input : 
            polygon : 
                geometry_msgs/Polygon polygon
                    geometry_msgs/Point32[] points
                        float32 x
                        float32 y
                        float32 z
            pos_arr : PoseArray that publish to fakeObstacleLayer
        output: 
            nothing , will append result in pos_arr
        '''
        for i in range(len(polygon.points)): # For every polygon edge
            #------ Get vertex end and start point --------# 
            vertex_start  = polygon.points[i]
            if i == len(polygon.points) - 1 :  # Last line 
                vertex_end = polygon.points[0]
            else:
                vertex_end = polygon.points[i+1]
            
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
                output.append([vertex_start[0] + step[0] * i , vertex_start[1] + step[1] * i])# TODO 
            '''
            #####################################
            ###   Bresenham's line algorithm  ###
            #####################################
            deltax = vertex_end.x - vertex_start.x
            deltay = vertex_end.y - vertex_start.y
            if deltax == 0: # To avoid inf slope
                deltax = 0.01

            deltaerr = abs(deltay / deltax) # slope
            error = 0.0 # No error at start
            
            if deltaerr <= 1: # 0~45 degree
                y = vertex_start.y
                for i in range(int(round(abs(deltax) / self.resolution, 0))):
                    x = vertex_start.x + i * self.resolution * sign(deltax)
                    tmp_pose = Pose()
                    tmp_pose.position.x = x 
                    tmp_pose.position.y = y 
                    pos_arr.poses.append(tmp_pose) 
                    error = error + deltaerr * self.resolution
                    if error >= 0.5 * self.resolution:
                        y = y + sign(deltay) * self.resolution
                        error = error - 1 * self.resolution
            else:  # 45~90 degree
                x = vertex_start.x
                deltaerr = 1 / deltaerr
                for i in range(int(round(abs(deltay) / self.resolution, 0))):
                    y = vertex_start.y + i * self.resolution * sign(deltay)
                    tmp_pose = Pose()
                    tmp_pose.position.x = x 
                    tmp_pose.position.y = y 
                    pos_arr.poses.append(tmp_pose) 
                    error = error + deltaerr * self.resolution
                    if error >= 0.5 * self.resolution:
                        x = x + sign(deltax) * self.resolution
                        error = error - 1 * self.resolution
        return 
        
    def transform_2_translation_quaternion(self, transform_m):
        """
        This function help transfer the geometry_msgs.msg.TransformStamped
        into (translation, quaternion) <-- lists
        """

        trans = [transform_m.transform.translation.x, transform_m.transform.translation.y, transform_m.transform.translation.z]
        quaternion = [transform_m.transform.rotation.x, transform_m.transform.rotation.y, transform_m.transform.rotation.z, transform_m.transform.rotation.w]
        return (trans, quaternion)

    def get_pose2D_base_in_map(self,map_frame, base_frame):
        """
        This is the fuction for getting the robot pose (/base_foorprint) referring to /map
        The function is mainly for global-location initialization.
        """
        # Get pose_2D from tf to reduce the effect of delay
        pose_2D = np.zeros((3,1))
        #
        # Try tf
        try:
            transform_m = self.tf_buffer.lookup_transform(map_frame, base_frame, rospy.Time(0))
            #
            
            # Translation of data types
            (trans, quaternion) = self.transform_2_translation_quaternion(transform_m)
            # end Translation of data types

            #
            pose_2D[0,0] = trans[0] # self._amcl_pose.position.x
            pose_2D[1,0] = trans[1] # self._amcl_pose.position.y
            euler = transformations.euler_from_quaternion(quaternion)
            
        except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("[projector] tf exception while getting robot pose.")
            return None
        pose_2D[2,0] = euler[2] # yaw
        return (pose_2D)
    
    def digitalize(self, v_in):
        '''
        resolution : map_resolution (m)
        Input: v_in 
        Output : v_dig
        '''
        # print "[digitalize] " , round(v_in/resolution, 0)
        v_in -= self.resolution/2
        return round(round(v_in/self.resolution, 0) *self.resolution + self.resolution/2, len(str(self.resolution))-1)

    def self_footprint_CB(self, msg):
        '''
        tmp_footprint = []
        for i in msg.polygon.points:
            tmp = [i.x, i.y]
            tmp_footprint.append(tmp)
        footprint_subcribe = tmp_footprint
        '''
        # ------- Publish footprint to /multirobot/position --------# 
        self.footprint.header = msg.header
        self.footprint.map_id = self.map_id
        
        self.footprint.robot_id = self.robot_id
        self.footprint.polygon = msg.polygon
        if self.footprint.robot_id != None and self.footprint.map_id != None: 
            self.projectSyncPub.publish(self.footprint)
        else: 
            print ("Get self_footprint but Information not fully gather yet.")

    def multi_footprint_CB(self, msg):
        if msg.map_id == self.map_id: # and msg.robot_id != self.robot_id: # project it TODO  
            print (str(msg))
            self.object_to_project[msg.robot_id] = msg 
        else: 
            pass # DO nothing  
    def map_id_CB(self, msg):
        self.map_id = msg.data
        print ("[CP] Switch to New map: " + str(self.map_id))

CP = COSTMAP_PROJECTOR()

def main(args):
    # global CP 
    rospy.init_node('projector')

    # Subscribe 
    rospy.Subscriber('/multirobot/position', PublicFootprint, CP.multi_footprint_CB)
    rospy.Subscriber('/currentFloor', String , CP.map_id_CB)
    rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, CP.self_footprint_CB)
    # Call process at 10Hz
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        # pos_arr = PoseArray()
        '''
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
        '''
        pos_arr = PoseArray()
        #----- Publish Projection on costmap----#
        for i in CP.object_to_project:
            # print ("DeltaT : " + str(time.time() - i.header.stamp))
            #if time.time() - i.header.stamp > 3 :TODO TODO TODO nned to fix the bug  # Exceed 3 sec without update, Robot Vanish, clear it from costmap
            #    pass
            if False: 
                pass  
            else: 
                CP.expand_footprint(pos_arr , CP.object_to_project[i].polygon)
        CP.projectBlockPub.publish(pos_arr)
        r.sleep()
    # Done
if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
