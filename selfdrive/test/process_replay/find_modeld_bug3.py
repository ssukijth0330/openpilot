#!/usr/bin/env python3
import time
from selfdrive.manager.process_config import managed_processes

def model_replay():
  while True:
    managed_processes['modeld'].start()
    time.sleep(5)
    managed_processes['modeld'].stop()


  return log_msgs


if __name__ == "__main__":
    model_replay()
