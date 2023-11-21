import smbus2
import time
import math

I2C_ADDR = 0x36
bus = smbus2.SMBus(1)
last_angle = None
radio = 5.41
acc_angle = 0
long = 0

def raw_angle():
   read_bytes = bus.read_i2c_block_data(0x36, 0x0C, 2)
   angle = (read_bytes[0]<<8)|read_bytes[1]
   return angle


def read_acc_angle(current_angle):
   global last_angle, acc_angle, long
   if last_angle is not None:
      delta = current_angle - last_angle
      if delta > math.pi:
         delta -= 2*math.pi
      elif delta < -math.pi:
         delta += 2*math.pi
      acc_angle += delta
      long = acc_angle*radio
   last_angle = current_angle
   return acc_angle,long

def read_angle():
   read_bytes = bus.read_i2c_block_data(0x36, 0x0C, 2)
   angle = (read_bytes[0]<<8)|read_bytes[1]
   angle_degree = angle*360/4096
   return angle_degree

while True:
   angle = read_angle()
   raw_ang = raw_angle()
   acc_angle,long = read_acc_angle(angle)
   print(f'angulo: {angle:.2f}   acc angulo: {acc_angle:.2f}   raw: {raw_ang:.2f}   long: {long:.2f}')
   time.sleep(0.5)
