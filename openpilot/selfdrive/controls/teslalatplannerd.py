#!/usr/bin/env python3
"""Publishes teslaLanePlan: a desired curvature derived from the stock Tesla
Autopilot lane model (DAS_lanes) instead of from the comma driving model."""
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import car
from opendbc.car.tesla.values import CANBUS, DBC

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL, Priority, Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.tesla_lane_planner import LINE_USAGE, TeslaLanePlanner
from openpilot.selfdrive.pandad import can_capnp_to_list

LaneChangeState = log.LaneChangeState

# DAS_lanes is sent by the Autopilot computer, so it arrives on the autopilot side of
# the party bus. It is also mirrored onto bus 0 while the harness relay is forwarding,
# so fall back to that bus if the autopilot side is not carrying it.
LANES_BUSES = (CANBUS.autopilot_party, CANBUS.party)


class TeslaLatPlannerD:
  def __init__(self, CP):
    self.planner = TeslaLanePlanner()
    self.parsers = {bus: CANParser(DBC[CP.carFingerprint][Bus.party], [("DAS_lanes", 10)], bus)
                    for bus in LANES_BUSES}

    self.sm = messaging.SubMaster(['carState', 'modelV2', 'selfdriveState'])
    self.pm = messaging.PubMaster(['teslaLanePlan'])
    self.can_sock = messaging.sub_sock('can', timeout=20)

  def update_can(self) -> dict | None:
    """Feed every queued CAN packet to both parsers, return the freshest DAS_lanes."""
    can_list = can_capnp_to_list(messaging.drain_sock_raw(self.can_sock))
    for cp in self.parsers.values():
      cp.update(can_list)

    # whichever bus is actually carrying a healthy DAS_lanes wins, autopilot side first
    for bus in LANES_BUSES:
      if self.parsers[bus].can_valid:
        return self.parsers[bus].vl["DAS_lanes"]
    return None

  def update(self):
    lanes = self.update_can()
    self.sm.update(0)

    CS = self.sm['carState']
    md = self.sm['modelV2']

    # never take over unless the comma model we fall back to is itself healthy
    if not self.sm.all_checks(['modelV2', 'carState']):
      lanes = None

    # start from a clean blend every time lateral control is handed to us, so we never
    # engage already leaning on a curvature computed while openpilot was not steering
    if not self.sm['selfdriveState'].active:
      self.planner.reset()

    lane_change = md.meta.laneChangeState != LaneChangeState.off
    plan = self.planner.update(lanes, CS.vEgo, float(md.action.desiredCurvature), lane_change, DT_MDL)
    self.publish(plan)

  def publish(self, plan):
    msg = messaging.new_message('teslaLanePlan')
    msg.valid = self.sm.all_checks(['carState', 'modelV2'])
    p = msg.teslaLanePlan
    p.valid = plan.valid
    p.invalidReason = plan.invalid_reason
    p.desiredCurvature = plan.desired_curvature
    p.lookahead = plan.lookahead
    p.lateralOffset = plan.lateral_offset
    p.c0, p.c1, p.c2, p.c3 = plan.c0, plan.c1, plan.c2, plan.c3
    p.viewRange = plan.view_range
    p.laneWidth = plan.lane_width
    p.leftLineUsage = LINE_USAGE[plan.left_line_usage]
    p.rightLineUsage = LINE_USAGE[plan.right_line_usage]
    self.pm.send('teslaLanePlan', msg)

  def run(self):
    rk = Ratekeeper(int(1 / DT_MDL), print_delay_threshold=None)
    while True:
      self.update()
      rk.keep_time()


def main():
  # imported here so TeslaLatPlannerD stays importable without the compiled params lib
  from openpilot.common.params import Params

  config_realtime_process(5, Priority.CTRL_LOW)

  params = Params()
  cloudlog.info("teslalatplannerd is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  if CP.brand != 'tesla':
    cloudlog.info(f"teslalatplannerd: not a Tesla ({CP.brand}), exiting")
    return

  TeslaLatPlannerD(CP).run()


if __name__ == "__main__":
  main()
