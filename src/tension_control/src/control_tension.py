#!/usr/bin/env python3
'''
unwind velocity positive [dps] and tether linear velocity positive [m/s]
wind velocity negative [dps] and tether linear velocity negative [m/s]
'''



import rospy
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Twist
from motor_rmd.srv import SetVelocity
from simple_pid import PID


class PIDController:

    def __init__(self):

        # PID PARAMETERS
        self.kp = 40.0 #before 0.5
        self.ki = 0.0
        self.kd = 0.2 #before 0

        self.setpoint = 0.0

        self.pid = PID(
            self.kp,
            self.ki,
            self.kd,
            setpoint=self.setpoint
        )

        self.max_speed = 400
        self.min_speed = -400
        self.pid.sample_time = 0.1
        self.pid.output_limits = (self.min_speed,self.max_speed)


        # FEEDFORWARD GAIN
        # Converts robot linear velocity [m/s] into motor velocity command
        self.Kff = 800.0

        # VARIABLES
        self.last_tension = None
        self.last_tension_time = None

        self.walle_vel = 0.0
        self.spool_vel = 0.0

        self.motor_ready = True

        # SUBSCRIBERS
        rospy.Subscriber('/spool/tension', Float64, self.tension_callback)
        rospy.Subscriber('/walle/cmd_vel', Twist, self.vel_callback)
        rospy.Subscriber('/spool/vel_tether', Float32, self.spool_vel_callback)
        rospy.Subscriber('/robot/tension_value', Float64, self.setpoint_callback)

        # MOTOR SERVICE
        rospy.wait_for_service('/rmd_motor/cmd_vel')
        try:
            self.set_speed_client = rospy.ServiceProxy('/rmd_motor/cmd_vel', SetVelocity)
            rospy.loginfo("Motor service connected")

        except rospy.ServiceException as e:

            rospy.logerr(f"Service connection failed: {e}")


        # PUBLISHERS
        self.error_pub = rospy.Publisher('/control_spool/error', Float64, queue_size=1)
        self.vel_pub = rospy.Publisher('/control_spool/cmd_vel', Float64, queue_size=1)
        self.ff_pub = rospy.Publisher('/control_spool/feedforward', Float64, queue_size=1)

    # CALLBACKS
    def tension_callback(self, msg):
        self.last_tension = msg.data #[kg]
        self.last_tension_time = rospy.get_time()

    def vel_callback(self, msg):
        self.walle_vel = msg.linear.x #[m/s]

    def spool_vel_callback(self, msg):
        self.spool_vel = msg.data #[m/s]

    def setpoint_callback(self, msg):
        self.setpoint = msg.data #[kg]
        self.pid.setpoint = self.setpoint

    # MAIN CONTROL LOOP
    def control_loop(self):
        
        

        # no sensor data yet
        if self.last_tension is None:
            return

        # timeout protection
        dt = rospy.get_time() - self.last_tension_time

        if dt > 0.5:
            rospy.logwarn("Tension sensor timeout")
            return

        current_tension = self.last_tension
        pid_raw = self.pid(current_tension)
        
        error = self.setpoint - current_tension
        
        # Rversed according the system
        #winding -> negative speed
        pid_vel = -pid_raw
        
        # FEEDFORWARD velocity
        # walle moves forward -> unwind -> positive speed
        ff_term =  self.Kff * self.walle_vel
        vel_cmd = ff_term + pid_vel

        # saturation
        vel_cmd = max(
            min(vel_cmd, self.max_speed),
            self.min_speed
        )

        if abs(error) < 0.30:
            if -10 < vel_cmd < 10:
                vel_cmd = 0
        else:
            if 0 < abs(vel_cmd) < 15:
                vel_cmd = 15 if vel_cmd > 0 else -15

        vel_cmd = int(vel_cmd)

        # SEND COMMAND TO MOTOR
        try:
            response = self.set_speed_client(vel_cmd)
            rospy.loginfo(f"velocity motor send: {vel_cmd}")
            if response.success:
                self.motor_ready = True
            else:
                self.motor_ready = False

        except rospy.ServiceException as e:
            rospy.logerr(f"Motor service error: {e}")


        # DEBUG TOPICS
        self.error_pub.publish(error)
        self.vel_pub.publish(vel_cmd)
        self.ff_pub.publish(ff_term)


    # RUN
    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.control_loop()
            rate.sleep()


if __name__ == '__main__':

    rospy.init_node('pid_controller')
    controller = PIDController()
    controller.run()
