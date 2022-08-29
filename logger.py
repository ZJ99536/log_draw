#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32
import numpy as np
from tf.transformations import euler_from_quaternion
import time


class Logger:
    def __init__(self):
        self.f = open("/home/zhoujin/log_ws/src/UAV_logger/src/text.txt", 'w')
        self.local_pos = []
        self.target_pos = [str(1),str(1),str(1.5)]
        self.local_vel = []
        self.target_vel = [str(0),str(0),str(0)]
        self.local_att = []
        self.target_att = []
        self.local_bodyrate = []
        self.target_bodyrate = []
        self.count = 0

        self.current_time = Float32()
        self.current_time.data = .0
        self.plan_euler = []
        self.plan_pos = []
        self.plan_vel = []
        self.plan_bodyrate = []
        self.current_position = []
        self.current_velocity = []
        self.current_attitude = []
        self.current_bodyrate = []

        self.fb_euler = []
        self.fb_bodyrate = []

        self.hover_euler = []
        self.hover_eulersp = []
        self.hover_pos = []
        self.hover_vel = []

        self.Kp1x = 1
        self.Kp1y = 1
        self.Kp1z = 1

        self.imu = [str(0),str(0),str(0)]
        self.gyro = []
        self.plan_thrust = []
        self.plan_att = []

        
        
        self.start_time = 0 
        self.cur_time = 0
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.poseCb)
        rospy.Subscriber("/mavros/local_position/velocity_local",TwistStamped,self.velCb)
        rospy.Subscriber("/mavros/setpoint_raw/target_local", PositionTarget,self.tarposCb)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget,self.tarattCb)
        rospy.Subscriber("/mavfast/planning/position", PoseStamped,self.planposCb)
        rospy.Subscriber("/mavfast/planning/velocity", TwistStamped,self.planvelCb)
        rospy.Subscriber("/mavfast/planning/euler", Vector3Stamped,self.planeulCb)
        rospy.Subscriber("mavfast/planning/bodyrate", AttitudeTarget, self.planrateCb)
        rospy.Subscriber("/mavfast/feedback/euler", Vector3Stamped,self.fbeulCb)
        rospy.Subscriber("mavfast/feedback/bodyrate", AttitudeTarget, self.fbrateCb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry,self.odometryCb)
        # rospy.Subscriber("/mavfast/planning/time", Float32, self.timeCb)
        rospy.Subscriber("mavros/imu/data",Imu,self.imuCb)

        rospy.Subscriber("mavgnc/position_setpoint", PoseStamped,self.hoverposCb)
        rospy.Subscriber("mavgnc/velocity_setpoint", TwistStamped,self.hovervelCb)
        rospy.Subscriber("mavgnc/att_sp_euler", Vector3Stamped,self.hovereulspCb)
        rospy.Subscriber("mavgnc/att_euler", Vector3Stamped,self.hovereulCb)
        rospy.Subscriber('mavgnc/time', Float32, self.timeCb)
        


        

    def write_data(self):
        if self.count != 0:
            # self.cur_time = time.clock()
            self.f.write(str(self.current_time.data))
            self.f.write(',')
            # self.f.write(str(self.current_time.data))
            # self.f.write(',')
            # self.f.write(','.join(self.local_pos))
            # self.f.write(',')
            # self.f.write(','.join(self.target_pos))
            # self.f.write(',')
            # self.f.write(','.join(self.local_vel))
            # self.f.write(',')
            # self.f.write(','.join(self.target_vel))
            # self.f.write(',')
            # self.f.write(','.join(self.local_att))
            # self.f.write(',')
            # self.f.write(','.join(self.target_att))
            # self.f.write(',')
            # self.f.write(','.join(self.local_bodyrate))
            # self.f.write(',')
            # self.f.write(','.join(self.target_bodyrate))
            # self.f.write(',')
            self.f.write(','.join(self.current_position))
            self.f.write(',')
            self.f.write(','.join(self.hover_pos))
            self.f.write(',')
            # self.f.write(','.join(self.plan_pos))
            # self.f.write(',')
            self.f.write(','.join(self.current_velocity))
            self.f.write(',')
            self.f.write(','.join(self.hover_vel))
            self.f.write(',')
            # self.f.write(','.join(self.plan_vel))
            # self.f.write(',')
            self.f.write(','.join(self.current_attitude))
            self.f.write(',') 
            self.f.write(','.join(self.hover_eulersp))
            self.f.write(',') 
            self.f.write(','.join(self.hover_euler))
            self.f.write(',') 
            # self.f.write(','.join(self.plan_euler))
            # self.f.write(',') 
            # self.f.write(','.join(self.fb_euler))
            # self.f.write(',') 
            self.f.write(','.join(self.current_bodyrate))
            self.f.write(',')
            self.f.write(','.join(self.hover_bodyrate)) 
            # self.f.write(','.join(self.plan_bodyrate)) 
            # self.f.write(',') 
            # self.f.write(','.join(self.fb_bodyrate)) 
            # self.f.write(',') 
            # self.f.write(','.join(self.imu)) 
            # self.f.write(',') 
            # self.f.write(','.join(self.plan_thrust))                        
            self.f.write('\r\n')
        self.count += 1

    def write_title(self):
        self.f.write("time,x,y,z,target_x,target_y,target_z,vel_x,vel_y,vel_z,target_vel_x,target_vel_y,target_vel_z,att_x,att_y,att_z,target_att_x,target_att_y,target_att_z,rate_x,rate_y,rate_z,target_rate_x,target_rate_y,target_rate_z,current_position_x,current_position_y,current_position_z,plan_pos_x,plan_pos_y,plan_pos_z,current_velocity_x,current_velocity_y,current_velocity_z,plan_vel_x,plan_vel_y,plan_vel_z,current_attitude_x,current_attitude_y,current_attitude_z,plan_euler_x,plan_euler_y,plan_euler_z,current_bodyrate_x,current_bodyrate_y,current_bodyrate_z")
        self.f.write('\r\n')

    def poseCb(self,msg):
        self.local_pos = [str(msg.pose.position.x), str(msg.pose.position.y), str(msg.pose.position.z)]
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        self.local_att = euler_from_quaternion(quaternion)
        self.local_att = list(map(str,self.local_att))
        self.target_vel = [str(self.Kp1x*(float(self.target_pos[0])-float(self.local_pos[0]))), str(self.Kp1y*(float(self.target_pos[1])-float(self.local_pos[1]))), str(self.Kp1z*(float(self.target_pos[2])-float(self.local_pos[2])))]
        # rospy.loginfo("target_vel:%s", self.target_vel[0])
        
    def velCb(self,msg):
        self.local_vel = [str(msg.twist.linear.x), str(msg.twist.linear.y), str(msg.twist.linear.z)]
        self.local_bodyrate = [str(msg.twist.angular.x), str(msg.twist.angular.y), str(msg.twist.angular.z)]
    
    def tarposCb(self,msg):
        self.target_pos = [str(msg.position.x), str(msg.position.y), str(msg.position.z)]
        # self.target_vel = [str(msg.velocity.x), str(msg.velocity.y), str(msg.velocity.z)]
        # self.target_vel = [str(self.Kp1x*(float(self.target_pos[0])-float(self.local_pos[0]))), str(self.Kp1y*(float(self.target_pos[1])-float(self.local_pos[1]))), str(self.Kp1z*(float(self.target_pos[2])-float(self.local_pos[2])))]
        # rospy.loginfo("target_vel:", self.target_vel[0])

    def tarattCb(self,msg):
        quaternion = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        self.target_att = euler_from_quaternion(quaternion)
        self.target_att = list(map(str,self.target_att))
        self.hover_thrust = [str(msg.thrust * 9.8/0.7)]  
        self.hover_bodyrate = [str(msg.body_rate.x),str(msg.body_rate.y),str(msg.body_rate.z)]
        

    def planeulCb(self,msg):
        # self.plan_euler = [str(msg.vector.x),str(msg.vector.y),str(msg.vector.z)]
        self.plan_euler = [str(msg.vector.x /3.14*180),str(msg.vector.y /3.14*180),str(msg.vector.z /3.14*180)]


    def planposCb(self,msg):
        self.plan_pos = [str(msg.pose.position.x), str(msg.pose.position.y), str(msg.pose.position.z)]

    def planvelCb(self,msg):
        self.plan_vel = [str(msg.twist.linear.x), str(msg.twist.linear.y), str(msg.twist.linear.z)]
       
    def planrateCb(self,msg):
        self.plan_bodyrate = [str(msg.body_rate.x), str(msg.body_rate.y), str(msg.body_rate.z)]
        self.plan_thrust = [str(msg.thrust * 9.8/0.7)]  

    def fbeulCb(self,msg):
        self.fb_euler = [str(msg.vector.x /3.14*180),str(msg.vector.y /3.14*180),str(msg.vector.z /3.14*180)]

    def fbrateCb(self,msg):
        self.fb_bodyrate = [str(msg.body_rate.x), str(msg.body_rate.y), str(msg.body_rate.z)]
      

    def hovereulCb(self,msg):
        self.hover_euler = [str(msg.vector.x),str(msg.vector.y),str(msg.vector.z)]
        # print(self.hover_euler)

    def hovereulspCb(self,msg):
        self.hover_eulersp = [str(msg.vector.x),str(msg.vector.y),str(msg.vector.z)]

    def hoverposCb(self,msg):
        self.hover_pos = [str(msg.pose.position.x), str(msg.pose.position.y), str(msg.pose.position.z)]

    def hovervelCb(self,msg):
        self.hover_vel = [str(msg.twist.linear.x), str(msg.twist.linear.y), str(msg.twist.linear.z)]
  

    def odometryCb(self,msg):
        self.current_position = [str(msg.pose.pose.position.x),str(msg.pose.pose.position.y),str(msg.pose.pose.position.z)]
        self.current_velocity = [str(msg.twist.twist.linear.x),str(msg.twist.twist.linear.y),str(msg.twist.twist.linear.z)]
        self.current_bodyrate = [str(msg.twist.twist.angular.x),str(msg.twist.twist.angular.y),str(msg.twist.twist.angular.z)]

        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.current_attitude = list(euler_from_quaternion(quaternion))
        for i in range(3):
            self.current_attitude[i] = self.current_attitude[i] /3.14*180
            # self.current_attitude[i] = self.current_attitude[i]

        self.current_attitude = list(map(str,self.current_attitude))

  
    def timeCb(self,msg):
        self.current_time.data = (msg.data)

    def imuCb(self,msg):
        self.imu = [str(msg.linear_acceleration.x), str(msg.linear_acceleration.y), str(msg.linear_acceleration.z)] 
        self.gyro = [str(msg.angular_velocity.x), str(msg.angular_velocity.y), str(msg.angular_velocity.z)] 
        # print(self.imu[2])


def main():
    print("start log!")
    rospy.init_node('logger_node', anonymous=True)
    logger = Logger()
    logger.write_title()
    rate = rospy.Rate(200)
    # logger.start_time = time.clock()

    while not rospy.is_shutdown():
        logger.write_data()
        rate.sleep()
    logger.f.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass