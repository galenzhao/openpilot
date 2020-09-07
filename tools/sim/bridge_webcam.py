#!/usr/bin/env python3
# type: ignore
import socket
import time
import math
import atexit
import numpy as np
import threading
import random
import cereal.messaging as messaging
import argparse
from common.params import Params
from common.realtime import Ratekeeper
from lib.can import can_function, sendcan_function
from lib.helpers import FakeSteeringWheel
from selfdrive.car.honda.values import CruiseButtons

parser = argparse.ArgumentParser(description='Bridge between CARLA and openpilot.')
parser.add_argument('--autopilot', action='store_true')
parser.add_argument('--joystick', action='store_true')
parser.add_argument('--realmonitoring', action='store_true')
args = parser.parse_args()

pm = messaging.PubMaster(['frame', 'sensorEvents', 'can'])

W, H = 1164, 874
car_speed = 0

UDP_IP = "0.0.0.0"
UDP_PORT = 2562

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

ax, ay, az = 0, 0, 0
gx, gy, gz = 0, 0, 0
speed_obd = 0.01
speed_gps = 0.01

def imu_callback(imu):
  #print(imu, imu.accelerometer)

  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [ax,ay,az]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [gx,gy,gz]
  pm.send('sensorEvents', dat)
  #print(imu)

def update_car():
  global car_speed
  #car_speed = car_speed + 1
  if speed_obd > 0:
    car_speed = speed_obd
  elif speed_gps > 0:
    car_speed = speed_gps
  else:
    car_speed = 0
  #imu_callback(1)

def thread_udp_recv(name):
  while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print("received message: %s" % data)
    data = str(data, 'utf-8')
    #print(data)
    datas = data.split(",")
    if "PID: 0d" == datas[0]:
      global speed_obd
      try:
        speed_obd = float(datas[2])
      except ValueError:
        speed_obd = 0
      update_car()

    elif "PID: ff1001" == datas[0]:
      global speed_gps
      try:
        speed_gps = float(datas[2])
      except ValueError:
        speed_gps = 0
      update_car()

    elif "TYPE_GYROSCOPE_UNCALIBRATED" == datas[0]:
      global gx
      gx = float(datas[1])
      global gy
      gy = float(datas[2])
      global gz
      gz = float(datas[3])
      imu_callback(1)

    elif "TYPE_ACCELEROMETER" == datas[0]:
      global ax
      ax = float(datas[1])
      global ay
      ay = float(datas[2])
      global az
      az = float(datas[3])
      imu_callback(1)

def health_function():
  pm = messaging.PubMaster(['health'])
  rk = Ratekeeper(1.0)
  while 1:
    dat = messaging.new_message('health')
    dat.valid = True
    dat.health = {
      'ignitionLine': True,
      'hwType': "greyPanda",
      'controlsAllowed': True
    }
    pm.send('health', dat)
    rk.keep_time()

def fake_driver_monitoring():
  if args.realmonitoring:
    return
  pm = messaging.PubMaster(['driverState'])
  while 1:
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)
    time.sleep(0.1)

def go(q):
  threading.Thread(target=health_function).start()
  threading.Thread(target=fake_driver_monitoring).start()


  def destroy():
    print("clean exit")
    imu.destroy()
    camera.destroy()
    vehicle.destroy()
    print("done")
  atexit.register(destroy)

  # can loop
  sendcan = messaging.sub_sock('sendcan')
  rk = Ratekeeper(100, print_delay_threshold=0.05)

  # init
  A_throttle = 2.
  A_brake = 2.
  A_steer_torque = 1.
  fake_wheel = FakeSteeringWheel()
  is_openpilot_engaged = False
  in_reverse = False

  throttle_out = 0
  brake_out = 0
  steer_angle_out = 0

  while 1:
    cruise_button = 0
    #update_car()

    # check for a input message, this will not block
    if not q.empty():
      print("here")
      message = q.get()

      m = message.split('_')
      if m[0] == "steer":
        steer_angle_out = float(m[1])
        fake_wheel.set_angle(steer_angle_out)  # touching the wheel overrides fake wheel angle
        # print(" === steering overriden === ")
      if m[0] == "throttle":
        throttle_out = float(m[1]) / 100.
        if throttle_out > 0.3:
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False
      if m[0] == "brake":
        brake_out = float(m[1]) / 100.
        if brake_out > 0.3:
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False
      if m[0] == "reverse":
        in_reverse = not in_reverse
        cruise_button = CruiseButtons.CANCEL
        is_openpilot_engaged = False
      if m[0] == "cruise":
        if m[1] == "down":
          cruise_button = CruiseButtons.DECEL_SET
          is_openpilot_engaged = True
        if m[1] == "up":
          cruise_button = CruiseButtons.RES_ACCEL
          is_openpilot_engaged = True
        if m[1] == "cancel":
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False

    #vel = vehicle.get_velocity()
    speed = car_speed#math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
    #can_function(pm, speed, fake_wheel.angle, rk.frame, cruise_button=cruise_button, is_engaged=is_openpilot_engaged)
    can_function(pm, speed, fake_wheel.angle, rk.frame, cruise_button=CruiseButtons.CANCEL, is_engaged=False)

    if rk.frame % 1 == 0:  # 20Hz?
      throttle_op, brake_op, steer_torque_op = sendcan_function(sendcan)
      # print(" === torq, ",steer_torque_op, " ===")
      if is_openpilot_engaged:
        fake_wheel.response(steer_torque_op * A_steer_torque, speed)
        throttle_out = throttle_op * A_throttle
        brake_out = brake_op * A_brake
        steer_angle_out = fake_wheel.angle
        print(steer_torque_op)
      # print(steer_angle_out)
      #vc = carla.VehicleControl(throttle=throttle_out, steer=steer_angle_out / 3.14, brake=brake_out, reverse=in_reverse)
      #vehicle.apply_control(vc)

    rk.keep_time()

if __name__ == "__main__":
  x = threading.Thread(target=thread_udp_recv, args=(1,))
  x.start()
  
  params = Params()
  params.delete("Offroad_ConnectivityNeeded")
  from selfdrive.version import terms_version, training_version
  params.put("HasAcceptedTerms", terms_version)
  params.put("CompletedTrainingVersion", training_version)
  params.put("CommunityFeaturesToggle", "1")
  params.put("CalibrationParams", '{"vanishing_point": [582.06, 442.78], "valid_blocks": 20}')

  # no carla, still run

  from multiprocessing import Process, Queue
  q = Queue()
  p = Process(target=go, args=(q,))
  p.daemon = True
  p.start()

  if args.joystick:
    # start input poll for joystick
    from lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(q)
  else:
    # start input poll for keyboard
    from lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(q)
