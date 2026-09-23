"""Integration test for teslalatplannerd: real CAN frames in, real teslaLanePlan out.

Exercises the daemon's own wiring (CAN parsing, bus selection, publishing), which the
TeslaLanePlanner unit tests do not touch.
"""
import time

import pytest

from opendbc.can import CANPacker
from opendbc.car.structs import car
from opendbc.car.tesla.values import CANBUS

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.controls.teslalatplannerd import TeslaLatPlannerD
from openpilot.selfdrive.pandad import can_list_to_can_capnp

V_EGO = 25.0


def das_lanes(packer, counter, c0=0.0, c1=0.0, c2=0.0, view_range=45.0, usage=2):
  dat = packer.pack(0x239, {
    "DAS_virtualLaneC0": c0,
    "DAS_virtualLaneC1": c1,
    "DAS_virtualLaneC2": c2,
    "DAS_virtualLaneC3": 0.0,
    "DAS_virtualLaneViewRange": view_range,
    "DAS_virtualLaneWidth": 3.7,
    "DAS_leftLineUsage": usage,
    "DAS_rightLineUsage": usage,
    "DAS_leftLaneExists": 1,
    "DAS_rightLaneExists": 1,
    "DAS_lanesCounter": counter % 16,
  })
  return bytes(dat)


class Harness:
  def __init__(self):
    CP = car.CarParams.new_message(carFingerprint="TESLA_MODEL_Y", brand="tesla")
    self.packer = CANPacker("tesla_model3_party")
    self.pm = messaging.PubMaster(['can', 'carState', 'modelV2', 'selfdriveState'])
    self.sm = messaging.SubMaster(['teslaLanePlan'])
    self.planner = TeslaLatPlannerD(CP)
    # this test drives the daemon step by step rather than in real time, so the
    # SubMaster average frequency check can never pass. It is not what we are testing.
    self.planner.sm.ignore_average_freq = ['carState', 'modelV2', 'selfdriveState']
    self.counter = 0
    # let every socket attach before the first publish, otherwise the daemon misses
    # the opening frames and the test is racing the transport rather than the logic
    time.sleep(0.3)

  def step(self, model_curvature=0.0, v_ego=V_EGO, send_lanes=True, bus=CANBUS.autopilot_party, **kw):
    if send_lanes:
      dat = das_lanes(self.packer, self.counter, **kw)
      self.pm.send('can', can_list_to_can_capnp([(0x239, dat, bus)]))
      self.counter += 1

    cs = messaging.new_message('carState')
    cs.valid = True
    cs.carState.vEgo = v_ego
    self.pm.send('carState', cs)

    md = messaging.new_message('modelV2')
    md.valid = True
    md.modelV2.action.desiredCurvature = model_curvature
    self.pm.send('modelV2', md)

    ss = messaging.new_message('selfdriveState')
    ss.valid = True
    ss.selfdriveState.active = True
    self.pm.send('selfdriveState', ss)

    time.sleep(0.01)
    self.planner.update()
    self.sm.update(100)
    return self.sm['teslaLanePlan']

  def run(self, n, **kw):
    plan = None
    for _ in range(n):
      plan = self.step(**kw)
    return plan


@pytest.fixture
def harness():
  return Harness()


class TestTeslaLatPlannerD:
  def test_publishes_and_engages_on_real_can(self, harness):
    """A packed DAS_lanes frame must make it all the way to a published plan."""
    plan = harness.run(60, c2=0.001, view_range=45.0)
    assert plan.valid, plan.invalidReason
    assert plan.invalidReason == 'none'
    assert plan.blend == 1.0, plan.blend
    assert plan.leftLineUsage == 'fused'
    assert plan.rightLineUsage == 'fused'
    assert abs(plan.viewRange - 45.0) < 1.0
    assert abs(plan.laneWidth - 3.7) < 0.35
    # 2*c2 with c2=0.001 is a 0.002 1/m right hand curve
    assert 0.0015 < plan.desiredCurvature < 0.0025, plan.desiredCurvature

  def test_decodes_polynomial_off_the_wire(self, harness):
    plan = harness.run(60, c0=0.35, c1=0.02, c2=0.0005)
    # 8 bit quantization, so only check we land near the packed values
    assert abs(plan.c0 - 0.35) < 0.04
    assert abs(plan.c1 - 0.02) < 0.002
    assert abs(plan.c2 - 0.0005) < 3e-5
    # c3's zero point does not land exactly on 0.0 in float, it never carries signal
    assert abs(plan.c3) < 1e-12

  def test_falls_back_when_can_stops(self, harness):
    harness.run(60, c2=0.001)
    assert harness.sm['teslaLanePlan'].valid
    plan = harness.run(60, send_lanes=False, model_curvature=0.01)
    assert not plan.valid
    assert plan.invalidReason == 'noData'
    assert abs(plan.desiredCurvature - 0.01) < 1e-6
    assert plan.blend == 0.0

  def test_reports_lines_not_fused(self, harness):
    plan = harness.run(30, usage=0)
    assert not plan.valid
    assert plan.invalidReason == 'linesNotFused'
    assert plan.leftLineUsage == 'rejectedUnavailable'

  def test_works_on_the_party_bus_too(self, harness):
    """DAS_lanes is mirrored onto bus 0 while the relay forwards; either must work."""
    plan = harness.run(60, c2=0.001, bus=CANBUS.party)
    assert plan.valid, plan.invalidReason
    assert 0.0015 < plan.desiredCurvature < 0.0025
