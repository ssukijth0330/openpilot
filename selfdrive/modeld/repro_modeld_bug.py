#!/usr/bin/env python3
import sys
import pickle
import numpy as np
from pathlib import Path
from typing import Dict
from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.models.commonmodel_pyx import ModelFrame, CLContext
import logging
logging.basicConfig(filename='/tmp/myapp.log', level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(name)s %(message)s')
logger=logging.getLogger(__name__)

SEND_RAW_PRED = True #os.getenv('SEND_RAW_PRED')

MODEL_PATHS = {
  ModelRunner.THNEED: Path(__file__).parent / 'models/supercombo.thneed',
  ModelRunner.ONNX: Path(__file__).parent / 'models/supercombo.onnx'}

METADATA_PATH = Path(__file__).parent / 'models/supercombo_metadata.pkl'

class ModelState:
  inputs: Dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse
  model: ModelRunner

  def __init__(self, context: CLContext):
    self.cnt = 0
    self.inputs = {
      'desire': np.zeros(ModelConstants.DESIRE_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'traffic_convention': np.zeros(ModelConstants.TRAFFIC_CONVENTION_LEN, dtype=np.float32),
      'nav_features': np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32),
      'nav_instructions': np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32),
      'features_buffer': np.zeros(ModelConstants.HISTORY_BUFFER_LEN * ModelConstants.FEATURE_LEN, dtype=np.float32),
    }

    with open(METADATA_PATH, 'rb') as f:
      model_metadata = pickle.load(f)
    net_output_size = model_metadata['output_shapes']['outputs'][1]
    self.output = np.zeros(net_output_size, dtype=np.float32)

    self.model = ModelRunner(MODEL_PATHS, self.output, Runtime.GPU, False, context)
    self.model.addInput("input_imgs", None)
    self.model.addInput("big_input_imgs", None)
    for k,v in self.inputs.items():
      self.model.addInput(k, v)

  def run(self,):
    self.cnt += 1
    if self.cnt %2 == 0:
      img_val = 100.0
    else:
      img_val = 0.0
    self.model.setInputBuffer("input_imgs", img_val * np.ones((128 * 256 * 12), dtype=np.float32))
    self.model.setInputBuffer("big_input_imgs", img_val * np.ones((128 * 256 * 12), dtype=np.float32))
    self.model.execute()


def main():
  raw_preds = []
  raw_preds_prev = []
  total_cnt = 0
  total_err_cnt = 0
  cl_context = CLContext()
  model = ModelState(cl_context)

  while True:
    model.run()
    raw_preds.append(np.copy(model.output))
    if len(raw_preds) == 10:
      if len(raw_preds_prev) == len(raw_preds):
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
            total_err_cnt += 1
      raw_preds_prev = raw_preds
      raw_preds = []
      total_cnt += 1
      print(f'DID {total_cnt} ITERATIONS with {total_err_cnt} errors')
      # COMMENTING THIS LINE WILL FIX ERRORS
      ModelFrame(cl_context)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    sys.exit()
