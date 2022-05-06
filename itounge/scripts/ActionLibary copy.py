#!/usr/bin/env python3
#license removed for brevity


import rospy
from std_msgs.msg import String
from itounge.msg import RAWItongueOut
from kinova_msgs.msg import PoseVelocityWithFingerVelocity, SetFingersPositionAction
import actionlib

import time


class JACO(object):
    def __init__(self):
        self.action = PoseVelocityWithFingerVelocity()
        
        rospy.Subscriber("RAWItongueOut", RAWItongueOut, self.callback)

        pub = rospy.Publisher("/j2n6s200_driver/in/cartesian_velocity_with_finger_velocity", PoseVelocityWithFingerVelocity, queue_size=10)
        

        r = rospy.Rate(100)

        while not rospy.is_shutdown():
             pub.publish(self.action)
             r.sleep()

   
    def callback(self, data):
    
        

        inputdata = data.Sensor
    
    
        print(data.Sensor)
    
        if inputdata == 0:
            self.action.twist_angular_x = 0
            self.action.twist_angular_y = 0
            self.action.twist_angular_z = 0
            self.action.twist_linear_y = 0
            self.action.twist_linear_x = 0
            self.action.twist_linear_z = 0
        elif inputdata == 1:
            self.action.twist_angular_y = 2.0
        elif inputdata == 2:
            self.action.twist_angular_x = -2.0
        elif inputdata == 3:
            self.action.twist_angular_y = -2.0
        elif inputdata == 4:
            self.action.twist_angular_z = 1.5
        elif inputdata == 5:
            self.action.twist_angular_x = 2.0
        elif inputdata == 6:
            self.action.twist_angular_z = 0.7
        """ if inputdata == 7:
            self.action = "7"
        if inputdata == 8:
            self.action = "8"
        if inputdata == 9:
            self.action = "9"
        if inputdata == 10:
            self.action = "10"
        """
        if inputdata == 11:
            self.action.twist_linear_z = -2.0
        if inputdata == 12:
            self.action.twist_linear_y = 2.0
        if inputdata == 13:
            self.action.finger1 = -2000.0
            self.action.finger2 = -2000.0
        if inputdata == 14:
            self.action.twist_linear_x = -2.0
        if inputdata == 15:
            self.action.twist_linear_x = 2.0
        if inputdata == 16:
            self.action.twist_linear_z = 2.0
        if inputdata == 17:
            self.action.twist_linear_y = -2.0
        if inputdata == 18:
            self.action.finger1 = 2000.0
            self.action.finger2 = 2000.0
            
       # if inputdata > 18:
      #      print(inputdata)
     #   
    #    print(inputdata)
    
        
        
        
    
    

        
 
    
if __name__ == '__main__':
    rospy.init_node('actionlib', anonymous=True)
    
    
    try:
        JACO()
    except rospy.ROSInterruptException:
           pass

    
        
    


    