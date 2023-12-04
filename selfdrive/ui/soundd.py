import math
import numpy as np
import os
import wave

from typing import Dict, Optional, Tuple

from cereal import car, messaging
from openpilot.common.basedir import BASEDIR

SAMPLE_RATE = 48000
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1

AMBIENT_DB = 30 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


sound_list: Dict[int, Tuple[str, Optional[int], float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),
}


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MAX_VOLUME
    self.current_sound_frame = 0

    os.system("pactl set-sink-volume @DEFAULT_SINK@ 0.9") # set pulse to max volume, so volume can be controlled within soundd

  def load_sounds(self):
    self.loaded_sounds: Dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      wavefile = wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r')

      assert wavefile.getnchannels() == 1
      assert wavefile.getsampwidth() == 2
      assert wavefile.getframerate() == SAMPLE_RATE

      length = wavefile.getnframes()
      self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / 32767

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    data_out[:frames, 0] = self.get_sound_data(frames)

  def new_alert(self, alert):
    if self.current_alert != alert:
      self.current_alert = alert
      self.current_sound_frame = 0

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(10, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    sm = messaging.SubMaster(['controlsState', 'microphone'])

    with sd.OutputStream(device="pulse", channels=1, samplerate=SAMPLE_RATE, callback=self.callback):
      while True:
        sm.update(timeout=1000)

        if sm.updated['controlsState']:
          new_alert = sm['controlsState'].alertSound.raw
          self.new_alert(new_alert)

        if sm.updated['microphone']:
          self.current_volume = self.calculate_volume(sm["microphone"].filteredSoundPressureWeightedDb)


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()