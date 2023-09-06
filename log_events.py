#!/usr/bin/env python3
import time
import pickle
from collections import defaultdict

from cereal import messaging

if __name__ == '__main__':
  sm = messaging.SubMaster(['carEvents', 'modelV2'])

  levents = defaultdict(list)
  lframes = []

  prev_frame_id: int | None = None
  prev_frame_id_extra: int | None = None
  t = 0

  while True:
    sm.update(0)
    if sm.updated['carEvents']:
      for ev in sm['carEvents']:
        if ev.overrideLateral or ev.overrideLongitudinal or ev.name == 'noGps':
          continue
        d = {
          'logMonoTime': sm.logMonoTime['carEvents'],
          'event': ev.__repr__(),
        }
        print('event', d)

        levents[str(ev.name)].append(d)

    if sm.updated['modelV2']:
      if prev_frame_id is not None:
        if prev_frame_id != (sm['modelV2'].frameId - 1):
          lframes.append(('skipped frame', sm.logMonoTime['modelV2'], prev_frame_id, sm['modelV2'].frameId))
          print(lframes[-1])
        if prev_frame_id_extra != (sm['modelV2'].frameIdExtra - 1):
          lframes.append(('skipped extra frame', sm.logMonoTime['modelV2'], prev_frame_id_extra, sm['modelV2'].frameIdExtra))
          print(lframes[-1])

      prev_frame_id = sm['modelV2'].frameId
      prev_frame_id_extra = sm['modelV2'].frameIdExtra

    if time.monotonic() - t > 60:
      t = time.monotonic()
      with open('/data/python-modeld-dump', 'wb') as f:
        print('dump')
        pickle.dump((dict(levents), lframes), f)
