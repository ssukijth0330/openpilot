import math
from collections import deque

from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, CarControllerParams, ChryslerFlags

LongCtrlState = car.CarControl.Actuators.LongControlState

TIRE_SIZE = [275, 55, 20] # 275/55R20
# https://x-engineer.org/calculate-wheel-radius/
WHEEL_RADIUS = 0.95 * ((TIRE_SIZE[2] * 25.4 / 2) + (TIRE_SIZE[0] * TIRE_SIZE[1] / 100)) / 1000
AXLE_RATIO = 3.21 # or 3.92
FINAL_DRIVE_RATIOS = [x * AXLE_RATIO for x in [4.71, 3.14, 2.10, 1.67, 1.29, 1.00, 0.84, 0.67]]
# https://web.archive.org/web/20180116135154/https://www.ramtrucks.com/2019/ram-1500.html
CdA = 13.0 / 10.764 # CdA = frontal drag coefficient x area (ft^2 converted to m^2)
# https://www.epa.gov/compliance-and-fuel-economy-data/data-cars-used-testing-fuel-economy
ROLLING_RESISTANCE_COEFF = 46.31 / 5500 # Target Coef A (lbf) / Equivalent Test Weight (lbs.)
VEHICLE_MASS = 2495 # kg
GRAVITY = 9.81 # m/s^2
AIR_DENSITY = 1.225 # kg/m3 (sea level air density of dry air @ 15° C)

def calc_motion_force(aEgo, road_pitch):
  force_parallel = VEHICLE_MASS * aEgo
  force_perpendicular = VEHICLE_MASS * GRAVITY * math.sin(road_pitch)
  return force_parallel + force_perpendicular

def calc_drag_force(engine_torque, transmision_gear, road_pitch, aEgo, vEgo, wind=0):
  if engine_torque <= 0 or vEgo < 2 or transmision_gear == 0:
    # https://x-engineer.org/rolling-resistance/
    force_rolling = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY
    # https://x-engineer.org/aerodynamic-drag/
    force_drag = 0.5 * CdA * AIR_DENSITY * ((vEgo - wind)**2)
    return force_rolling + force_drag

  total_force = engine_torque * FINAL_DRIVE_RATIOS[transmision_gear-1] / WHEEL_RADIUS
  return total_force - calc_motion_force(aEgo, road_pitch)

def calc_engine_torque(accel, pitch, transmission_gear, drag_force):
  force_total = calc_motion_force(accel, pitch) + drag_force
  # https://x-engineer.org/calculate-wheel-torque-engine/
  wheel_torque = force_total * WHEEL_RADIUS
  engine_torque = wheel_torque / FINAL_DRIVE_RATIOS[int(transmission_gear)-1]
  return engine_torque

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    self.gas_history = deque(maxlen=15)

  def update(self, CC, CS, now_nanos):
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if not self.CP.openpilotLongitudinalControl and (self.frame - self.last_button_frame)*DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1
        if self.CP.openpilotLongitudinalControl:
          can_sends.append(chryslercan.create_acc_hud(self.packer, 4 if CC.longActive else 0, CC.hudControl.setSpeed))

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit:
        apply_steer = 0
      self.apply_steer_last = apply_steer

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    # longitudinal
    if self.CP.openpilotLongitudinalControl and (self.frame % self.params.ACC_CONTROL_STEP) == 0:
      long_active = CC.longActive
      accel = clip(CC.actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      starting = CS.out.vEgo < 0.25 and accel > 0.0 # TODO: use LongCtrlState.starting with disabled startAccel?
      stopping = CC.actuators.longControlState == LongCtrlState.stopping

      gas = self.params.INACTIVE_GAS
      brakes = self.params.INACTIVE_ACCEL
      if long_active:
        pitch = CC.orientationNED[1] if len(CC.orientationNED) > 1 else 0
        drag_force = calc_drag_force(CS.engine_torque, CS.transmission_gear, pitch, CS.out.aEgo, CS.out.vEgo)
        gas_next = clip(calc_engine_torque(accel, pitch, CS.transmission_gear, drag_force), self.params.GAS_MIN, self.params.GAS_MAX)
        self.gas_history.appendleft(gas_next)
        gas = sum(self.gas_history) / self.gas_history.maxlen
        if gas <= self.params.GAS_MIN+1 or accel < 0.1:
          brakes = min(accel, 0)
      else:
        self.gas_history.clear()

      can_sends.extend(chryslercan.create_acc_commands(self.packer, long_active, gas, brakes, starting, stopping))

    self.frame += 1

    new_actuators = CC.actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends
