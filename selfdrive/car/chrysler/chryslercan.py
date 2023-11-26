from cereal import car
from openpilot.selfdrive.car.chrysler.values import RAM_CARS
from openpilot.common.conversions import Conversions as CV

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  color = 2 if lkas_active else 1
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  if CP.carFingerprint in RAM_CARS:
    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, cancel=False, resume=False):
  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def create_acc_commands(packer, long_active, gas, brakes, starting, stopping):
  commands = []

  das_3_values = {
    'ACC_AVAILABLE': 1,
    'ACC_ACTIVE': long_active,
    'ACC_DECEL_REQ': brakes < 0.0,
    'ACC_DECEL': brakes,
    'ENGINE_TORQUE_REQUEST_MAX': brakes >= 0.0,
    'ENGINE_TORQUE_REQUEST': gas,
    'ACC_STANDSTILL': stopping,
    'ACC_GO': starting,
    # TODO: does this improve fuel economy?
    'DISABLE_FUEL_SHUTOFF': gas > 0.0,
    # TODO: does this have any impact on ACC braking responsiveness?
    'ACC_BRK_PREP': brakes < 0.0,
    # TODO: does this have any impact on ACC braking responsiveness?
    #'COLLISION_BRK_PREP': ?,
  }
  commands.append(packer.make_can_msg("DAS_3", 0, das_3_values))

  das_5_values = {
    "FCW_STATE": 0x1,
    "FCW_DISTANCE": 0x2,
  }
  commands.append(packer.make_can_msg("DAS_5", 0, das_5_values))

  return commands

def create_acc_hud(packer, long_active, set_speed):
  values = {
    "SPEED_DIGITAL": 197, # TODO: rename, this is actually distance to lead, check if pacifica is same
    "ACC_STATE": 4 if long_active else 3,
    "ACC_SET_SPEED_KPH": round(set_speed * CV.MS_TO_KPH),
    "ACC_SET_SPEED_MPH": round(set_speed * CV.MS_TO_MPH),
    "ACC_DISTANCE_CONFIG_1": 0,
    "ACC_DISTANCE_CONFIG_2": 3 if long_active else 1,
  }
  return packer.make_can_msg("DAS_4", 0, values)
