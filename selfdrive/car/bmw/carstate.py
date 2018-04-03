import os
from selfdrive.can.parser import CANParser
from selfdrive.config import Conversions as CV
from common.kalman.simple_kalman import KF1D
import numpy as np


def parse_gear_shifter(can_gear, car_fingerprint):
  # # TODO: Use values from DBC to parse this field
  # if car_fingerprint == CAR.PRIUS:
  #   if can_gear == 0x0:
  #     return "park"
  #   elif can_gear == 0x1:
  #     return "reverse"
  #   elif can_gear == 0x2:
  #     return "neutral"
  #   elif can_gear == 0x3:
  #     return "drive"
  #   elif can_gear == 0x4:
  #     return "brake"
  # elif car_fingerprint in [CAR.RAV4, CAR.RAV4H,
  #                          CAR.LEXUS_RXH, CAR.COROLLA]:
  #   if can_gear == 0x20:
  #     return "park"
  #   elif can_gear == 0x10:
  #     return "reverse"
  #   elif can_gear == 0x8:
  #     return "neutral"
  #   elif can_gear == 0x0:
  #     return "drive"
  #   elif can_gear == 0x1:
  #     return "sport"
  #
  # return "unknown"
  return "drive"


def get_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  dbc_f = "bmw_i3.dbc"

  signals = [
    # sig_name, sig_address, default
    ("SPEED", "CAR_SPEED",  0),
    ("STEERING_ANGLE", "NEW_MSG_11", 0),
    ("Accelerator_pressed", "NEW_MSG_2", 0),
    # ("GEAR", "GEAR_PACKET", 0),
    ("BRAKE_FORCE", "NEW_MSG_1", 0),
    ("RATE", "STEER_ANGLE_RATE", 0),
    ("TORQUE", "STEER_TORQUE", 0),
    # ("GAS_PEDAL", "GAS_PEDAL", 0),
    # ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
    # ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
    # ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
    # ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
    # ("DOOR_OPEN_FL", "SEATS_DOORS", 1),
    # ("DOOR_OPEN_FR", "SEATS_DOORS", 1),
    # ("DOOR_OPEN_RL", "SEATS_DOORS", 1),
    # ("DOOR_OPEN_RR", "SEATS_DOORS", 1),
    # ("SEATBELT_DRIVER_UNLATCHED", "SEATS_DOORS", 1),
    # ("TC_DISABLED", "ESP_CONTROL", 1),
    # ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
    # ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
    # ("GAS_RELEASED", "PCM_CRUISE", 0),
    # ("CRUISE_STATE", "PCM_CRUISE", 0),
    # ("MAIN_ON", "PCM_CRUISE_2", 0),
    # ("SET_SPEED", "PCM_CRUISE_2", 0),
    # ("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0),
    # ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
    # ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
    # ("TURN_SIGNALS", "STEERING_LEVERS", 3),   # 3 is no blinkers
    # ("LKA_STATE", "EPS_STATUS", 0),
    # ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
    # ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
  ]

  checks = [
    # ("BRAKE_MODULE", 40),
    # ("GAS_PEDAL", 33),
    # ("WHEEL_SPEEDS", 80),
    # ("STEER_ANGLE_SENSOR", 80),
    # ("PCM_CRUISE", 33),
    # ("PCM_CRUISE_2", 33),
    # ("STEER_TORQUE_SENSOR", 50),
    # ("EPS_STATUS", 25),
  ]

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 0)


class CarState(object):
  def __init__(self, CP):

    self.CP = CP
    self.left_blinker_on = 0
    self.right_blinker_on = 0

    # initialize can parser
    self.car_fingerprint = CP.carFingerprint

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=np.matrix([[0.0], [0.0]]),
                         A=np.matrix([[1.0, dt], [0.0, 1.0]]),
                         C=np.matrix([1.0, 0.0]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.0

  def update(self, cp):
    # copy can_valid
    self.can_valid = cp.can_valid

    # update prevs, update must run once per loop
    self.prev_left_blinker_on = False #self.left_blinker_on
    self.prev_right_blinker_on = False #self.right_blinker_on

    self.door_all_closed = True
    self.seatbelt = True

    self.steer_error = False
    self.brake_error = 0
    self.steering_angle = cp.vl["NEW_MSG_11"]["STEERING_ANGLE"]

    self.brake_pressed = False # cp.vl["NEW_MSG_1"]["Brake_Force"] > 0
    self.pedal_gas = cp.vl["NEW_MSG_2"]['Accelerator_pressed'] - 192
    self.car_gas = self.pedal_gas
    self.esp_disabled = False # cp.vl["ESP_CONTROL"]['TC_DISABLED']

    # calc best v_ego estimate, by averaging two opposite corners
    #i#self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    #self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    #self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    #self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    self.v_wheel = cp.vl["CAR_SPEED"]["SPEED"] * CV.KPH_TO_MS #(self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.

    # Kalman filter
    if abs(self.v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_x = np.matrix([[self.v_wheel], [0.0]])

    self.v_ego_raw = self.v_wheel
    v_ego_x = self.v_ego_kf.update(self.v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = not self.v_wheel > 0.001

    can_gear = 0

    self.angle_steers_rate = cp.vl["STEER_ANGLE_RATE"]["RATE"]
    self.gear_shifter = parse_gear_shifter(can_gear, self.car_fingerprint)
    self.main_on = True # cp.vl["PCM_CRUISE_2"]['MAIN_ON']
    self.left_blinker_on = False # cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    self.right_blinker_on = False #cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2

    # we could use the override bit from dbc, but it's triggered at too high torque values
    self.steer_override = 0 # abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']) > 100
    self.steer_error = 0 # cp.vl["EPS_STATUS"]['LKA_STATE'] == 50
    self.steer_torque_driver = cp.vl["STEER_TORQUE"]["TORQUE"]
    self.steer_torque_motor = 0 #cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']

    self.user_brake = 0
    self.v_cruise_pcm = False #cp.vl["PCM_CRUISE_2"]['SET_SPEED']
    self.pcm_acc_status = False #cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    self.gas_pressed = self.pedal_gas #not cp.vl["PCM_CRUISE"]['GAS_RELEASED']
    self.low_speed_lockout = False #cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
    self.brake_lights = False #bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or self.brake_pressed)
    self.generic_toggle = False #bool(cp.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
