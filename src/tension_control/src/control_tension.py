#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from motor_rmd.srv import SetVelocity
from simple_pid import PID

class PIDController:
    def __init__(self):
        # PID initialize
        self.kp = 0.1
        self.ki = 0
        self.kd = 0
        
        self.setpoint = 0.0
        self.pid = PID(self.kp,self.ki,self.kd,setpoint=self.setpoint)
        self.motor_ready = True
        self.max_speed = 400*0.5
        self.min_speed = -400*0.5
        self.pid.sample_time = 0.1
        self.pid.output_limits = (self.min_speed,self.max_speed)

        # Feedback value tension sensor  (tension force)
        self.feedback_sub = rospy.Subscriber('/spool/tension', Float64, self.tension_callback)

        # Pid out (motor velocity)
        rospy.wait_for_service('/rmd_motor/cmd_vel')
        try:
           self.set_speed_client = rospy.ServiceProxy('/rmd_motor/cmd_vel', SetVelocity)
           rospy.loginfo("SetVelocity service client connected successfully")
        except rospy.ServiceException as e:
           rospy.logerr(f"Error connecting to the service: {e}") 
        
        self.error_pub = rospy.Publisher('/control_spool/error', Float64, queue_size=1)
        self.tension_setpoint_sub = rospy.Subscriber('robot/tension_value', Float64, self.setpoint_callback)

    def tension_callback(self, msg):
        if not self.motor_ready:
           return
        current_value = msg.data
        rospy.logerr(f"*****tension data: {current_value}")
        error = self.setpoint - current_value
        vel = self.pid(current_value)
        vel = int(vel*-1)
        if vel>-10 and vel<10:
           vel = 0
        # send vel motor
        try:
           response = self.set_speed_client(vel)
           rospy.loginfo(f"velocit send: {vel}")
           if response.success:
              self.motor_ready = True
           else:
              self.motor_ready = False
        except rospy.ServiceException as e:
           rospy.logerr(f"Failed to call set_speed_client service: {e}")
        # publish error
        self.error_pub.publish(error)
        

    def setpoint_callback(self, msg):
        self.setpoint = msg.data
        rospy.logerr(f"----Setpoint is: {self.setpoint}")
        self.pid.setpoint = self.setpoint

if __name__ == '__main__':
    rospy.init_node('pid_controller')
    controller = PIDController()
    rospy.spin()

