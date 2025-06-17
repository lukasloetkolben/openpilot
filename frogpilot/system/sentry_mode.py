#!/usr/bin/env python3
import math
import numpy as np
import time

import cereal.messaging as messaging

from openpilot.common.filter_simple import FirstOrderFilter

from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

ALARM_TIME = 30
CRASH_THRESHOLD = 0.1 * ACCELERATION_DUE_TO_GRAVITY
FIRE_LEVEL = 0.8
SENSITIVITY_THRESHOLD = 0.001 * ACCELERATION_DUE_TO_GRAVITY
TIME_INTERVAL = 1

class SentryMode:
  def __init__(self):
    self.noise_filter = FirstOrderFilter(0., -ALARM_TIME / math.log(1. - FIRE_LEVEL), TIME_INTERVAL)

    self.trigger_alarm = False

    self.accel_magnitude_baseline = None

    self.sm = messaging.SubMaster(["accelerometer"])

  def update(self):
    self.sm.update()

    if not self.sm.alive["accelerometer"]:
      return

    accel_magnitude = np.linalg.norm(self.sm["accelerometer"].acceleration.v)
    print(f"accel_magnitude: {accel_magnitude}")

    if self.accel_magnitude_baseline is None:
      self.accel_magnitude_baseline = accel_magnitude

    delta = abs(accel_magnitude - self.accel_magnitude_baseline)
    print(f"delta: {delta:.4f}")

    level = self.noise_filter.update(delta > SENSITIVITY_THRESHOLD)
    print(f"level: {level}")

    self.trigger_alarm = (delta >= CRASH_THRESHOLD) or (level >= FIRE_LEVEL)
    print(f"trigger_alarm: {self.trigger_alarm}")

    time.sleep(TIME_INTERVAL)
