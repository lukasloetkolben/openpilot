# distutils: language = c++
# cython: language_level = 3
import datetime
import json
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector

cdef extern from "common/params.h":
  cpdef enum ParamKeyFlag:
    PERSISTENT
    CLEAR_ON_MANAGER_START
    CLEAR_ON_ONROAD_TRANSITION
    CLEAR_ON_OFFROAD_TRANSITION
    DEVELOPMENT_ONLY
    CLEAR_ON_IGNITION_ON
    ALL

  cpdef enum ParamKeyType:
    STRING
    BOOL
    INT
    FLOAT
    TIME
    JSON
    BYTES

  cdef cppclass c_Params "Params":
    c_Params(string) except + nogil
    string get(string, bool) nogil
    bool getBool(string, bool) nogil
    int remove(string) nogil
    int put(string, string) nogil
    void putNonBlocking(string, string) nogil
    void putBoolNonBlocking(string, bool) nogil
    int putBool(string, bool) nogil
    bool checkKey(string) nogil
    ParamKeyType getKeyType(string) nogil
    string getKeyDefaultValue(string) nogil
    string getParamPath(string) nogil
    void clearAll(ParamKeyFlag)
    vector[string] allKeys()


def ensure_bytes(v):
  return v.encode() if isinstance(v, str) else v

class UnknownKeyName(Exception):
  pass

cdef class Params:
  cdef c_Params* p
  cdef str d

  def __cinit__(self, d=""):
    cdef string path = <string>d.encode()
    with nogil:
      self.p = new c_Params(path)
    self.d = d

  def __reduce__(self):
    return (type(self), (self.d,))

  def __dealloc__(self):
    del self.p

  def clear_all(self, tx_flag=ParamKeyFlag.ALL):
    self.p.clearAll(tx_flag)

  def check_key(self, key):
    key = ensure_bytes(key)
    if not self.p.checkKey(key):
      raise UnknownKeyName(key)
    return key

  def get(self, key, bool block=False, default=None):
    cdef string k = self.check_key(key)
    cdef ParamKeyType t = self.p.getKeyType(ensure_bytes(key))
    cdef string val
    with nogil:
      val = self.p.get(k, block)

    if val == b"":
      if block:
        # If we got no value while running in blocked mode
        # it means we got an interrupt while waiting
        raise KeyboardInterrupt
      else:
        return default

    try:
      if t == STRING:
        return val.decode("utf-8")
      elif t == BOOL:
        return val == b"1"
      elif t == INT:
        return int(val)
      elif t == FLOAT:
        return float(val)
      elif t == TIME:
        return datetime.datetime.fromisoformat(val.decode("utf-8"))
      elif t == JSON:
        return json.loads(val)
      elif t == BYTES:
        return val
      else:
        return default
    except (TypeError, ValueError):
      return default

  def get_bool(self, key, bool block=False):
    cdef string k = self.check_key(key)
    cdef bool r
    with nogil:
      r = self.p.getBool(k, block)
    return r

  def put(self, key, dat):
    """
    Warning: This function blocks until the param is written to disk!
    In very rare cases this can take over a second, and your code will hang.
    Use the put_nonblocking, put_bool_nonblocking in time sensitive code, but
    in general try to avoid writing params as much as possible.
    """
    cdef string k = self.check_key(key)
    cdef string dat_bytes = ensure_bytes(dat)
    with nogil:
      self.p.put(k, dat_bytes)

  def put_bool(self, key, bool val):
    cdef string k = self.check_key(key)
    with nogil:
      self.p.putBool(k, val)

  def put_nonblocking(self, key, dat):
    cdef string k = self.check_key(key)
    cdef string dat_bytes = ensure_bytes(dat)
    with nogil:
      self.p.putNonBlocking(k, dat_bytes)

  def put_bool_nonblocking(self, key, bool val):
    cdef string k = self.check_key(key)
    with nogil:
      self.p.putBoolNonBlocking(k, val)

  def remove(self, key):
    cdef string k = self.check_key(key)
    with nogil:
      self.p.remove(k)

  def get_param_path(self, key=""):
    cdef string key_bytes = ensure_bytes(key)
    return self.p.getParamPath(key_bytes).decode("utf-8")

  def all_keys(self):
    return self.p.allKeys()

  def get_default_value(self, key):
    return self.p.getKeyDefaultValue(self.check_key(key))
