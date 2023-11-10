#!/usr/bin/env python3
import os
os.environ['SEND_RAW_PRED'] = '1'

import sys
import time
from collections import defaultdict
from typing import Any
import numpy as np
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.system.hardware import PC
from openpilot.selfdrive.manager.process_config import managed_processes
from openpilot.selfdrive.test.openpilotci import BASE_URL, get_url
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs, format_diff
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config, replay_process
from openpilot.system.version import get_commit
from openpilot.tools.lib.framereader import FrameReader
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.helpers import save_log

TEST_ROUTE = "2f4452b03ccb98f0|2022-12-03--13-45-30"
SEGMENT = 6
MAX_FRAMES = 600

NO_NAV = "NO_NAV" in os.environ
SEND_EXTRA_INPUTS = bool(int(os.getenv("SEND_EXTRA_INPUTS", "0")))


def get_log_fn(ref_commit, test_route):
  return f"{test_route}_model_tici_{ref_commit}.bz2"


def trim_logs_to_max_frames(logs, max_frames, frs_types, include_all_types):
  all_msgs = []
  cam_state_counts = defaultdict(int)
  # keep adding messages until cam states are equal to MAX_FRAMES
  for msg in sorted(logs, key=lambda m: m.logMonoTime):
    all_msgs.append(msg)
    if msg.which() in frs_types:
      cam_state_counts[msg.which()] += 1

    if all(cam_state_counts[state] == max_frames for state in frs_types):
      break

  if len(include_all_types) != 0:
    other_msgs = [m for m in logs if m.which() in include_all_types]
    all_msgs.extend(other_msgs)

  return all_msgs

def model_replay(lr, frs):
  # modeld is using frame pairs
  modeld_logs = trim_logs_to_max_frames(lr, MAX_FRAMES, {"roadCameraState", "wideRoadCameraState"}, {"roadEncodeIdx", "wideRoadEncodeIdx"})
  if not SEND_EXTRA_INPUTS:
    modeld_logs = [msg for msg in modeld_logs if msg.which() not in ["liveCalibration", "lateralPlan"]]
  # initial calibration
  cal_msg = next(msg for msg in lr if msg.which() == "liveCalibration").as_builder()
  cal_msg.logMonoTime = lr[0].logMonoTime
  modeld_logs.insert(0, cal_msg.as_reader())

  modeld = get_process_config("modeld")

  modeld_msgs = replay_process(modeld, modeld_logs, frs)
  return modeld_msgs


if __name__ == "__main__":
  update = "--update" in sys.argv
  replay_dir = os.path.dirname(os.path.abspath(__file__))
  ref_commit_fn = os.path.join(replay_dir, "model_replay_ref_commit")

  # load logs
  lr = list(LogReader(get_url(TEST_ROUTE, SEGMENT)))
  frs = {
    'roadCameraState': FrameReader(get_url(TEST_ROUTE, SEGMENT, log_type="fcamera"), readahead=True),
    'wideRoadCameraState': FrameReader(get_url(TEST_ROUTE, SEGMENT, log_type="ecamera"), readahead=True)
  }

  # run replays
  err_cnt = 0
  cnt = 0
  while True:
    log_msgs1 = model_replay(lr, frs)
