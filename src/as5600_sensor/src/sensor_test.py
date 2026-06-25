#!/usr/bin/env python3
# Conexion sensor VCC 3.3V Rojo Primer pin hacia adentro (lado contrario usb)
#                 GND GND cafe Tercer pin columna hacia afuera
#                 SDA amarillo 2do pin primera columna (misma fila vcc)
#                 SCL blanco 3er pin primera columna
#
#
#
#
#
#
#
#
#
#
#
#
#


import rospy
import smbus2
from std_msgs.msg import Float32
import math

class AS5600Node:
   def __init__(self):
      self.I2C_ADDR = 0x36
      self.bus = smbus2.SMBus(1)

      rospy.init_node("as5600_node",anonymous=True)
      self.radio = rospy.get_param('~radio',5.41)
      
      #Publisher topics 
      self.angle_pub = rospy.Publisher("spool/angle", Float32, queue_size=1)
      self.acc_angle_pub = rospy.Publisher("spool/acc_angle", Float32, queue_size=1)
      self.long_pub = rospy.Publisher('spool/long_tether', Float32, queue_size=1)
      self.vel_pub = rospy.Publisher('spool/vel_tether', Float32, queue_size=1)
      self.vel_ang_pub = rospy.Publisher('spool/vel_ang_tether', Float32, queue_size=1)
      
      #Variable Init 
      self.last_angle = None
      self.acc_angle = 0
      self.long = 0.0
      self.offset = self.get_offset()
      self.last_time = None 
      self.vel = 0.0 
      self.vel_ang = 0.0 
      print(self.offset)
      #self.rate = rospy.Rate(5)
      
   def get_offset(self):
      try:
         read_bytes = self.bus.read_i2c_block_data(self.I2C_ADDR, 0x0C,2)
         value = (read_bytes[0]<<8)|read_bytes[1]
         value = value*2*math.pi/4096
         return value
      except Exception as e:
         rospy.logerr(f"Error AS5600 read ofset: {e}")
         return -1
      
   def read_status(self):
      status = self.bus.read_i2c_block_data(self.I2C_ADDR, 0x0B,2)
      return status

   def read_angle(self):
      try:
         read_bytes = self.bus.read_i2c_block_data(self.I2C_ADDR, 0x0C,2)
         angle = (read_bytes[0]<<8)|read_bytes[1]
         if self.offset >=0:
            angle_rad =(angle*2*math.pi/4096 - self.offset) % (2*math.pi)
         else:
            angle_rad = angle*2*math.pi/4096
            return angle_rad
         
      except Exception as e:
         rospy.logerr(f"Error AS5600 reading: {e}")
         return None

   def read_magnitude(self):
         read_bytes = self.bus.read_i2c_block_data(self.I2C_ADDR, 0x1B,2)
         mag = (read_bytes[0]<<8)|read_bytes[1]
         return mag

   def read_acc_angle(self,current_angle):
      current_time = rospy.Time.now()
      
      if self.last_angle is not None and self.last_time is not None:
         delta = current_angle - self.last_angle
         
         if delta > math.pi:
            delta -= 2*math.pi
         elif delta < -math.pi:
            delta += 2*math.pi
            
         dt = (current_time - self.last_time).to_sec()
         
         if dt > 0: 
            self.vel_ang = delta / dt #rad/s 
            self.vel = self.vel_ang * self.radio / 1000 #m/s
         
         self.acc_angle += delta
         self.long = self.acc_angle*self.radio
      
      self.last_angle = current_angle
      self.last_time = current_time 

   def run(self):
      while not rospy.is_shutdown():
         start_time = rospy.Time.now()
         angle = self.read_angle()
         magnitude = self.read_magnitude()
         status = self.read_status()
         if angle is not None:
            self.read_acc_angle(angle)
            self.angle_pub.publish(angle)
            self.acc_angle_pub.publish(self.acc_angle)
            self.long_pub.publish(self.long)
            self.vel_ang_pub.publish(self.vel_ang) 
            self.vel_pub.publish(self.vel)
            
         elapsed_time = (rospy.Time.now() - start_time).to_sec()
         sleep_time = 0.01 #max(0,(1/5) - elapsed_time)
         rospy.sleep(sleep_time)

if __name__ == '__main__':
   try:
      node = AS5600Node()
      node.run()
   except rospy.ROSInterruptException:
      pass

