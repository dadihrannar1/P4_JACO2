#!/usr/bin/env python3
#license removed for brevity


from pickle import FALSE, TRUE
import rospy
from std_msgs.msg import String, Header
from itounge.msg import RAWItongueOut
from kinova_msgs.msg import PoseVelocityWithFingerVelocity, SetFingersPositionAction, SetFingersPositionGoal, ArmPoseAction, ArmPoseGoal
import actionlib
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import time


class JACO(object):
    activate = FALSE
    desired_pose = [0.4, 0.3, 0.5, 0.2, 0.1, 0.3, 0.3]



    
    
    def cartesian_pose_client():
    
        action_address = '/j2n6s200_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, ArmPoseAction)
        client.wait_for_server()
        
        goal = ArmPoseGoal()
        goal.pose.header = Header(frame_id=('j2n6s200_link_base'))
        goal.pose.pose.position = Point(
            x=0.009590843319892878, y=-0.3046131730079651, z=0.5225686430931091)
        goal.pose.pose.orientation = Quaternion(
            x=0.6971788727385513, y=0.16368353501991328, z=0.5053636054298092, w=0.4814114104145974)

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None

    def __init__(self):
            self.action = PoseVelocityWithFingerVelocity()
            
            rospy.Subscriber("RAWItongueOut", RAWItongueOut, self.callback)
            rospy.Subscriber("/j2n6s200_driver/out/tool_pose", PoseStamped, self.second_callback)
            pub = rospy.Publisher("/j2n6s200_driver/in/cartesian_velocity_with_finger_velocity", PoseVelocityWithFingerVelocity, queue_size=10)
            
            r = rospy.Rate(100)

            while not rospy.is_shutdown():
                JACO.cartesian_pose_client()
                pub.publish(self.action)
                r.sleep()
    
    def second_callback(self, info):

         JACO.second_callback.cur_pose = [info.pose.position.x, info.pose.position.y, info.pose.position.z, info.pose.orientation.x, info.pose.orientation.y, info.pose.orientation.z]

   
    def callback(self, data):
    
        

        inputdata = data.Sensor
    
        print(JACO.second_callback.cur_pose)
        print(JACO.activate)
        print(JACO.desired_pose)
    
        if inputdata == 0:
            self.action.twist_angular_x = 0.0
            self.action.twist_angular_y = 0.0
            self.action.twist_angular_z = 0.0
            self.action.twist_linear_y = 0.0
            self.action.twist_linear_x = 0.0
            self.action.twist_linear_z = 0.0
            self.action.finger1 = 0.0
            self.action.finger2 = 0.0
        elif inputdata == 1:
            self.action.twist_angular_y = 2.0
        elif inputdata == 2:
            self.action.twist_angular_x = -2.0
        elif inputdata == 3:
            self.action.twist_angular_y = -2.0
        elif inputdata == 4:
            self.action.twist_angular_z = -1.5
        elif inputdata == 5:
            self.action.twist_angular_x = 2.0
        elif inputdata == 6:
            self.action.twist_angular_z = 0.7
        elif inputdata == 7:
            JACO.activate = TRUE
        elif inputdata == 8:
            JACO.activate = FALSE
        elif inputdata == 9:
            print("ERROR NR 9 NOT ASSIGNED")
        elif inputdata == 10:
            print("ERROR NR 10 NOT ASSIGNED")
        elif inputdata == 11:
            self.action.twist_linear_z = -2.0
        elif inputdata == 12:
            self.action.twist_linear_y = 2.0
        elif inputdata == 13:
            self.action.finger1 = -2000.0
            self.action.finger2 = -2000.0
        elif inputdata == 14:
            self.action.twist_linear_x = 2.0
        elif inputdata == 15:
            self.action.twist_linear_x = -2.0
        elif inputdata == 16:
            self.action.twist_linear_z = 2.0
        elif inputdata == 17:
            self.action.twist_linear_y = -2.0
        elif inputdata == 18:
            self.action.finger1 = 2000.0
            self.action.finger2 = 2000.0

        if JACO.activate == TRUE:
            if not JACO.second_callback.cur_pose[0] < JACO.desired_pose[0] - 0.01 and not JACO.second_callback.cur_pose[0] > JACO.desired_pose[0] + 0.01:
                if JACO.second_callback.cur_pose[0] < JACO.desired_pose[0]:
                    self.action.twist_linear_x = 1.0
                elif JACO.second_callback.cur_pose[0] > JACO.desired_pose[0]:
                    self.action.twist_linear_x = -1.0
            if not JACO.second_callback.cur_pose[1] < JACO.desired_pose[1] -0.01 and not JACO.second_callback.cur_pose[1] > JACO.desired_pose[1] + 0.01:
                if JACO.second_callback.cur_pose[1] < JACO.desired_pose[1]:
                    self.action.twist_linear_y = 1.0
                elif JACO.second_callback.cur_pose[1] > JACO.desired_pose[1]:
                    self.action.twist_linear_y = -1.0
            if JACO.second_callback.cur_pose[2] < JACO.desired_pose[2] -0.01 and not JACO.second_callback.cur_pose[2] > JACO.desired_pose[2] + 0.01:
                if JACO.second_callback.cur_pose[2] < JACO.desired_pose[2]:
                    self.action.twist_linear_z = 1.0
                elif JACO.second_callback.cur_pose[2] > JACO.desired_pose[1]:
                    self.action.twist_linear_z = -1.0
            if JACO.second_callback.cur_pose[3] < JACO.desired_pose[3] - 0.01 and not JACO.second_callback.cur_pose[2] > JACO.desired_pose[2] + 0.01:
                if JACO.second_callback.cur_pose[3] < JACO.desired_pose[3]:
                    self.action.twist_angular_x = 1.0 
                elif JACO.second_callback.cur_pose[3] > JACO.desired_pose[3]:
                    self.action.twist_angular_x = -1.0
            if JACO.second_callback.cur_pose[4] < JACO.desired_pose[4] and not JACO.second_callback.cur_pose[4] > JACO.desired_pose[4] + 0.01:
                if JACO.second_callback.cur_pose[4] < JACO.desired_pose[4]:
                    self.action.twist_angular_y = 1.0
                elif JACO.second_callback.cur_pose[4] > JACO.desired_pose[4]:
                    self.action.twist_angular_y = -1.0
            if JACO.second_callback.cur_pose[5] < JACO.desired_pose[5] and not JACO.second_callback.cur_pose[5] > JACO.desired_pose[5] + 0.01:
                if JACO.second_callback.cur_pose[5] < JACO.desired_pose[5]:
                    self.action.twist_angular_z = 1.0
                if JACO.second_callback.cur_pose[5] < JACO.desired_pose[5]:
                    self.action.twist_angular_z = -1.0
        if JACO.disired.pose == JACO.second_callback.cur_pose:
            JACO.activate = FALSE
        print(inputdata)
    
        
        
        
    
    

        
 
    
if __name__ == '__main__':
    rospy.init_node('beboo', anonymous=True)
    
    
    try:
        
        #JACO.gripper_client(2)
        JACO()
        
    except rospy.ROSInterruptException:
           pass

    
        
    


    