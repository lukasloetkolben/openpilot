import random
import struct

from opendbc.can import CANPacker, CANParser
from opendbc.can.parser import PyCANParser
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp

NO_TRAVERSAL_LIMIT = 2**64 - 1


def pycapnp_to_list(strings, msgtype='can'):
  # reference: the pycapnp implementation can_capnp_to_list replaced
  result = []
  for s in strings:
    with log.Event.from_bytes(s, traversal_limit_in_words=NO_TRAVERSAL_LIMIT) as event:
      frames = [(f.address, f.dat, f.src) for f in getattr(event, msgtype)]
      result.append((event.logMonoTime, frames))
  return result


def random_frames(rng, n):
  return [(rng.getrandbits(29), rng.randbytes(rng.choice([0, 1, 8, 12, 32, 48, 64])), rng.randrange(256)) for _ in range(n)]


def segment_count(b):
  return struct.unpack_from('<I', b)[0] + 1


class TestCanCapnp(OpenpilotTestCase):
  def test_matches_pycapnp(self):
    rng = random.Random(0)
    saw_multi_segment = False
    for msgtype in ('can', 'sendcan'):
      for n in (0, 1, 2, 10, 200, 1000, 3000):
        strings = [can_list_to_can_capnp(random_frames(rng, n), msgtype=msgtype) for _ in range(3)]
        saw_multi_segment |= any(segment_count(s) > 1 for s in strings)
        self.assertEqual(list(can_capnp_to_list(strings, msgtype)), pycapnp_to_list(strings, msgtype))
    # big messages must spill into more segments to exercise far pointers
    self.assertTrue(saw_multi_segment)

  def test_list_interface(self):
    strings = [can_list_to_can_capnp([(0x123, b'\x01\x02', 1)]), can_list_to_can_capnp([])]
    frames = can_capnp_to_list(strings)
    ref = pycapnp_to_list(strings)
    self.assertEqual(len(frames), 2)
    self.assertEqual(frames[0], ref[0])
    self.assertEqual(frames[-1], ref[-1])
    self.assertEqual([m[1] for m in frames], [m[1] for m in ref])
    with self.assertRaises(IndexError):
      frames[2]
    self.assertEqual(len(can_capnp_to_list([])), 0)

  def test_wrong_msgtype(self):
    with self.assertRaises(ValueError):
      can_capnp_to_list([can_list_to_can_capnp([(1, b'', 0)], msgtype='sendcan')], 'can')
    with self.assertRaises(ValueError):
      can_capnp_to_list([log.Event.new_message(clocks={}).to_bytes()])

  def test_corrupt_input(self):
    # truncated or bit-flipped messages must raise ValueError or decode, never crash
    rng = random.Random(1)
    good = [can_list_to_can_capnp(random_frames(rng, n)) for n in (5, 1500)]
    for _ in range(3000):
      b = bytearray(rng.choice(good))
      if rng.random() < 0.3:
        b = b[:rng.randrange(len(b))]
      for _ in range(rng.randint(1, 4)):
        if b:
          b[rng.randrange(len(b))] ^= 1 << rng.randrange(8)
      try:
        can_capnp_to_list([bytes(b)])
      except ValueError:
        pass

  def test_parser_fast_path(self):
    # CANParser reads CanFrames natively, PyCANParser goes through the list interface
    dbc = "toyota_nodsu_pt_generated"
    packer = CANPacker(dbc)
    msgs = [("STEER_ANGLE_SENSOR", 80), ("WHEEL_SPEEDS", 80)]
    rs, py = CANParser(dbc, msgs, 0), PyCANParser(dbc, msgs, 0)
    for i in range(100):
      frames = [packer.make_can_msg("STEER_ANGLE_SENSOR", 0, {"STEER_ANGLE": i}),
                packer.make_can_msg("WHEEL_SPEEDS", i % 2, {"WHEEL_SPEED_FL": i})]
      cf = can_capnp_to_list([can_list_to_can_capnp(frames)])
      self.assertEqual(rs.update(cf), py.update(cf))
      self.assertEqual(rs.vl, py.vl)
      self.assertEqual(rs.ts_nanos, py.ts_nanos)
      self.assertEqual(rs.can_valid, py.can_valid)
