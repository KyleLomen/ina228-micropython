"""INA228 Driver for MicroPython"""

import time
from machine import I2C, Pin

# Common register addresses for INA2XX family
_CONFIG = const(0x00)
_ADCCFG = const(0x01)
_SHUNTCAL = const(0x02)
_VSHUNT = const(0x04)
_VBUS = const(0x05)
_DIETEMP = const(0x06)
_CURRENT = const(0x07)
_POWER = const(0x08)
_DIAGALRT = const(0x0B)
_MFG_UID = const(0x3E)
_DVC_UID = const(0x3F)

# INA228-specific registers
_SHUNT_TEMPCO = const(0x03)
_ENERGY = const(0x09)
_CHARGE = const(0x0A)
_SOVL = const(0x0C)
_SUVL = const(0x0D)
_BOVL = const(0x0E)
_BUVL = const(0x0F)
_TEMP_LIMIT = const(0x10)
_PWR_LIMIT = const(0x11)

# Constants
_INA2XX_DEFAULT_ADDR = const(0x40)
_TEXAS_INSTRUMENTS_ID = const(0x5449)
_INA228_DEVICE_ID = const(0x228)

def _twos_comp(val : int, bits : int) -> int:
  """compute the 2's complement of int value val"""
  if (val & (1 << (bits - 1))) != 0:
    val = val - (1 << bits) 
  return val

class INA228:
  """Driver for INA228 Current Sensor"""

  def __init__(self, i2c: I2C, address: int = _INA2XX_DEFAULT_ADDR):
    self._i2c = i2c
    self._address = address
    self._current_lsb = 10 / (2**19)

  def _read_register(self, register: int, size: int) -> int:
    tmp = self._i2c.readfrom_mem(self._address, register, size)
    return int.from_bytes(tmp, 'big')

  def _write_register16(self, register: int, register_value: int):
    register_bytes = register_value.to_bytes(2, 'big')
    self._i2c.writeto_mem(self._address, register, register_bytes)
  
  def reset_all(self):
    config = self._read_register(_CONFIG, 2)
    config = config | (1 << 15) # set Reset bit
    self._write_register16(_CONFIG, config)

  def calibrate_shunt(self, max_current: float, shunt_ohms: float):
    self._current_lsb = max_current / (2**19)
    shunt_cal = 13107.2e6 * self._current_lsb * shunt_ohms # Assumes ADCRANGE = 0
    self._write_register16(_SHUNTCAL, int(shunt_cal))

  def get_current(self) -> float:
    raw = self._read_register(_CURRENT, 3)
    current = _twos_comp(raw >> 4, 20) * self._current_lsb
    return current
  
  def get_vshunt(self) -> float:
    raw = self._read_register(_VSHUNT, 3)
    return _twos_comp(raw >> 4, 20)
  
  def get_diagnostic_flags(self) -> bytes:
    return self._i2c.readfrom_mem(self._address, _DIAGALRT, 2)
  
  def get_manufacturer_id(self) -> bytes:
    return self._i2c.readfrom_mem(self._address, _MFG_UID, 2)
  
  def get_device_id(self) -> bytes:
    return self._i2c.readfrom_mem(self._address, _DVC_UID, 2)

  