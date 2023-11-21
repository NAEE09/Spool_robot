#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from motor_rmd.srv import SetVelocity
from simple_pid import PID

class PIDController:
    def __init__(self):
        # PID initialize
        self.kp = 0.08
        self.ki = 0
        self.kd = 0
        
        self.setpoint = 0.0
        self.pid = PID(self.kp,self.ki,self.kd,setpoint=self.setpoint)
        self.motor_ready = True
        self.max_speed = 400*0.5
        self.min_speed = -400*0.5
        self.pid.sample_time = 0.1
        self.pid.output_limits = (self.min_speed,self.max_speed)

        #Sample tension control
        self.last_tension = None
        self.last_tension_time = None

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
        self.vel_pub = rospy.Publisher('/control_spool/vel', Float64, queue_size=1)
        self.tension_setpoint_sub = rospy.Subscriber('robot/tension_value', Float64, self.setpoint_callback)

    def tension_callback(self, msg):
        if not self.motor_ready:
           return
        self.last_tension = msg.data
        self.last_tension_time = rospy.get_time()

    def setpoint_callback(self, msg):
        self.setpoint = msg.data
        self.pid.setpoint = self.setpoint

    def run(self):
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
          self.control_loop()
          rate.sleep()

    def control_loop(self):
       if self.last_tension is None:
          return
       time_samp = rospy.get_time() - self.last_tension_time
       if time_samp > 0.5:
          rospy.logwarn("Tension buffer acumulated")
          return
       current_value = self.last_tension
       error = self.setpoint - current_value
       vel = int(-1 * self.pid(current_value))
       if -10 < vel < 10:
          vel = 0
       try:
           self.vel_pub.publish(vel) 
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


if __name__ == '__main__':
    rospy.init_node('pid_controller')
    controller = PIDController()
    controller.run()
