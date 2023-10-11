#!/usr/bin/env python3
import sys
import numpy as np
from pathlib import Path
from typing import Dict, Optional
from setproctitle import setproctitle
from openpilot.system.swaglog import cloudlog
from openpilot.common.realtime import config_realtime_process
from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.models.commonmodel_pyx import ModelFrame, CLContext
from openpilot.selfdrive.modeld.models.driving_pyx import (FEATURE_LEN, HISTORY_BUFFER_LEN,
                                                           DESIRE_LEN, TRAFFIC_CONVENTION_LEN, NAV_FEATURE_LEN, NAV_INSTRUCTION_LEN,
                                                           NET_OUTPUT_SIZE)

MODEL_PATHS = {
  ModelRunner.THNEED: Path(__file__).parent.parent.parent / 'modeld/models/supercombo.thneed',
  ModelRunner.ONNX: Path(__file__).parent.parent.parent / 'modeld/models/supercombo.onnx'}

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

class ModelState:
  frame: ModelFrame
  wide_frame: ModelFrame
  inputs: Dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse
  model: ModelRunner

  def __init__(self, context: CLContext):
    self.frame = ModelFrame(context)
    self.wide_frame = ModelFrame(context)
    self.prev_desire = np.zeros(DESIRE_LEN, dtype=np.float32)
    self.output = np.zeros(NET_OUTPUT_SIZE, dtype=np.float32)
    self.inputs = {
      'desire': np.zeros(DESIRE_LEN * (HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'traffic_convention': np.zeros(TRAFFIC_CONVENTION_LEN, dtype=np.float32),
      'nav_features': np.zeros(NAV_FEATURE_LEN, dtype=np.float32),
      'nav_instructions': np.zeros(NAV_INSTRUCTION_LEN, dtype=np.float32),
      'features_buffer': np.zeros(HISTORY_BUFFER_LEN * FEATURE_LEN, dtype=np.float32),
    }

    self.model = ModelRunner(MODEL_PATHS, self.output, Runtime.GPU, False, context)
    self.model.addInput("input_imgs", None)
    self.model.addInput("big_input_imgs", None)
    for k,v in self.inputs.items():
      self.model.addInput(k, v)
    self.cnt = 0

  def run(self) -> Optional[np.ndarray]:

    # if getCLBuffer is not None, frame will be None
    self.cnt += 1
    if self.cnt %2 == 0:
      img_val = 100.0
    else:
      img_val = 0.0
    self.model.setInputBuffer("input_imgs", img_val * np.ones((128 * 256 * 12), dtype=np.float32))
    self.model.setInputBuffer("big_input_imgs", img_val * np.ones((128 * 256 * 12), dtype=np.float32))


    self.model.execute()
    return self.output


def main():
  cloudlog.bind(daemon="selfdrive.modeld.modeld")
  setproctitle("selfdrive.modeld.modeld")
  config_realtime_process(7, 54)

  cl_context = CLContext()
  model = ModelState(cl_context)
  cloudlog.warning("models loaded, modeld starting")
  raw_preds_prev = None
  cnt = 0
  err_cnt = 0
  while True:
    raw_preds = []
    model.inputs['features_buffer'][:] = 0

    for _ in range(10):
      raw_preds.append(np.copy(np.frombuffer(model.run(), dtype=np.int8)))


    #raw_preds = [msg.modelV2.rawPredictions for msg in log_msgs if msg.which() == "modelV2"]
    if raw_preds_prev is not None:
      for i in range(len(raw_preds)):
        try:
          assert len(raw_preds[i]) > 0
          a = raw_preds[i]
          b = raw_preds_prev[i]
          equal = a == b
          assert np.all(equal)
          assert max(a-b) == 0
        except Exception as e:
          unequal_idxs = np.where(0 == equal)[0]
          print(f'ERROR: {e}')
          print(f'UNEQUAL IDXS: {unequal_idxs}')
          err_cnt += 1
    cnt += 1
    raw_preds_prev = raw_preds
    print(f'DID {cnt} ITERATIONS with {err_cnt} errors')



if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    sys.exit()
