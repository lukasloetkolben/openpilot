import time
from openpilot.cereal import log
from opendbc.can._parser_rs import decode_can_events

# Cache schema fields for faster access (avoids string lookup on each field access)
_can_layouts = {}  # msgtype -> field offsets for decode_can_events
_cached_writer_fields = None  # (address_field, dat_field, src_field) for writing


def _get_writer_fields(schema):
  """Get cached schema field objects for writing."""
  global _cached_writer_fields
  if _cached_writer_fields is None:
    fields = schema.fields
    _cached_writer_fields = (fields['address'], fields['dat'], fields['src'])
  return _cached_writer_fields


def can_list_to_can_capnp(can_msgs, msgtype='can', valid=True):
  """Convert list of CAN messages to Cap'n Proto serialized bytes.

  Args:
    can_msgs: List of tuples [(address, data_bytes, src), ...]
    msgtype: 'can' or 'sendcan'
    valid: Whether the event is valid

  Returns:
    Cap'n Proto serialized bytes
  """
  global _cached_writer_fields

  dat = log.Event.new_message(valid=valid, logMonoTime=int(time.monotonic() * 1e9))
  can_data = dat.init(msgtype, len(can_msgs))

  # Cache schema fields on first call
  if _cached_writer_fields is None and len(can_msgs) > 0:
    _cached_writer_fields = _get_writer_fields(can_data[0].schema)

  if _cached_writer_fields is not None:
    addr_f, dat_f, src_f = _cached_writer_fields
    for i, msg in enumerate(can_msgs):
      f = can_data[i]
      f._set_by_field(addr_f, msg[0])
      f._set_by_field(dat_f, msg[1])
      f._set_by_field(src_f, msg[2])

  return dat.to_bytes()


def _get_can_layout(msgtype):
  """Byte offsets of the Event and CanData fields read by decode_can_events."""
  if msgtype not in _can_layouts:
    event = log.Event.schema.node.struct
    union_field = next(f for f in event.fields if f.name == msgtype)
    event_fields = {f.name: f for f in event.fields}
    can_fields = {f.name: f for f in log.CanData.schema.node.struct.fields}
    _can_layouts[msgtype] = {
      'log_mono_time': event_fields['logMonoTime'].slot.offset * 8,  # UInt64
      'discriminant': event.discriminantOffset * 2,  # UInt16
      'discriminant_value': union_field.discriminantValue,
      'list_ptr': union_field.slot.offset,
      'address': can_fields['address'].slot.offset * 4,  # UInt32
      'src': can_fields['src'].slot.offset,  # UInt8
      'dat_ptr': can_fields['dat'].slot.offset,
    }
  return _can_layouts[msgtype]


def can_capnp_to_list(strings, msgtype='can'):
  """Convert Cap'n Proto serialized bytes to list of CAN messages.

  Args:
    strings: Tuple/list of serialized Cap'n Proto bytes
    msgtype: 'can' or 'sendcan'

  Returns:
    CanFrames, decoded in Rust. CANParser.update reads it directly, otherwise it
    behaves like a list of tuples [(nanos, [(address, data, src), ...]), ...]
  """
  return decode_can_events(strings, _get_can_layout(msgtype))
