import json
import argparse
from cereal import messaging

def main(pc_addr):
  pm = messaging.PubMaster(['testJoystick'])
  sm = messaging.SubMaster(['bodyReserved0'], addr=pc_addr)

  while True:
      sm.update(0)

      if sm.updated['bodyReserved0']:
          controls = json.loads(sm['bodyReserved0'])
          # yolodat = json.loads(sm['bodyReserved0'])
          # controls = f(yolodat)
          msg = messaging.new_message('testJoystick')
          msg.testJoystick.axes = [controls['x'], controls['y']]
          msg.testJoystick.buttons = [False]
          pm.send('testJoystick', msg)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='')
  parser.add_argument("addr", help="Address of pc")
  args = parser.parse_args()
  main(args.addr)
