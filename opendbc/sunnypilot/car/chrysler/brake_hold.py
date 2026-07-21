"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Jeep brake hold: the stock ACC auto-cancels ~3s after reaching a standstill
(no full stop-and-go support). While stopped in a hold, inject DAS_3 brake
commands to keep the car stationary, report engaged-at-standstill upstream so
openpilot stays engaged, and press resume (paced to the steering column's
counter) when the model wants to move. Based on the jvePilot implementation.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.chrysler import chryslercan
from opendbc.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser

from opendbc.sunnypilot.car.chrysler.values_ext import ChryslerFlagsSP

GearShifter = structs.CarState.GearShifter

HOLD_DECEL = -2.0  # m/s^2, default brake decel while holding

SPEED_1_SNA = 0xFFF * 0.071028  # m/s, raw max means the ESC speed signal is not available

# resume press pattern, indexed by column counter tick (jvePilot): two presses
# at counter+1, one at counter+0, two ticks silent. The DASM debounces button
# presses; this mimics a human press held across counter periods.
RESUME_COUNTER_OFFSETS = [1, 1, 0, None, None]

# 100Hz control frames without a fresh DASM DAS_3 (50Hz) before the car bus is
# considered dead. 50 frames = 0.5s = ~25 missed DASM transmissions.
DAS_3_STALE_LIMIT = 50


def create_das_3_command(packer, counter_offset: int, go: bool | None, torque_req: bool, torque: float | None,
                         max_gear: int | None, stop: bool | None, brake: float | None, brake_prep: bool,
                         das_3: dict) -> CanData:
  values = das_3.copy()  # forward what we parsed
  values.pop("CHECKSUM", None)
  values["ACC_AVAILABLE"] = 1
  values["ACC_ACTIVE"] = 1
  values["COUNTER"] = (das_3["COUNTER"] + counter_offset) % 0x10

  if go is not None:
    values["ACC_GO"] = go

  if stop is not None:
    values["ACC_STANDSTILL"] = stop

  if brake is not None:
    values["ACC_DECEL_REQ"] = 1
    values["ACC_DECEL"] = brake
    values["ACC_BRK_PREP"] = brake_prep

  if torque is not None:
    values["ENGINE_TORQUE_REQUEST_MAX"] = torque_req
    values["ENGINE_TORQUE_REQUEST"] = torque
  else:
    # never forward the DASM's torque-request bit: the safety layer rejects
    # any DAS_3 with it set, which would silently drop the entire hold frame
    values["ENGINE_TORQUE_REQUEST_MAX"] = 0

  if max_gear is not None:
    values["GR_MAX_REQ"] = max_gear

  return packer.make_can_msg("DAS_3", 0, values)


class BrakeHoldCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.brake_hold_enabled = bool(CP_SP.flags & ChryslerFlagsSP.JEEP_BRAKE_HOLD)

    self.brake_hold = False  # brake hold currently active
    self.brake_hold_pending = False  # armed, waiting for full stop
    self.cruise_active_actual = False  # ACC state as reported by the car
    self.cruise_available_actual = False  # ACC main switch state as reported by the car
    self.forward_gear = False
    self.acc_decelerating = False
    self.vehicle_stopped = False  # ESP_8 raw speed is zero
    self.inputs_valid = False  # all brake hold inputs are fresh and plausible
    self.das_3: dict[str, float] = {}

  def update_brake_hold(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.brake_hold_enabled:
      return

    cp = can_parsers[Bus.pt]

    self.forward_gear = ret.gearShifter == GearShifter.drive

    # capture the actual cruise state for brake hold logic, independent of any
    # DAS_3 messages we inject on the bus
    self.cruise_active_actual = ret.cruiseState.enabled
    self.cruise_available_actual = ret.cruiseState.available

    # ACC_DECEL_REQ means the ACC is commanding brakes (jvePilot activation gate)
    self.acc_decelerating = cp.vl["DAS_3"]["ACC_DECEL_REQ"] == 1

    # ESP_8 Vehicle_Speed == 0 means the safety layer will permit DAS_3 hold
    self.vehicle_stopped = cp.vl["ESP_8"]["Vehicle_Speed"] == 0

    # All deactivation inputs (speed, gear, pedals) must be fresh and plausible.
    # Any parser invalidity/timeout, or an ESC speed SNA burst (raw max — the
    # safety layer reads it as "moving" while carstate holds the last value, so
    # the two sides disagree about standstill), makes the hold unsafe to run.
    speed_sna = max(cp.vl["SPEED_1"]["SPEED_LEFT"], cp.vl["SPEED_1"]["SPEED_RIGHT"]) >= SPEED_1_SNA
    self.inputs_valid = cp.can_valid and not cp.bus_timeout and not speed_sna

    # While holding after the stock ACC dropped out, report engaged-at-standstill
    # upstream (jvePilot: "stay enabled"). This keeps openpilot engaged through
    # the ACC's 3s standstill timeout, so controlsd provides cruiseControl.resume
    # when the model sees the lead depart — the same path every stop-and-go
    # capable car uses. Gated on ret.standstill so it can never assert engaged
    # while the car is still creeping, and on input validity so a frozen or
    # implausible bus can never keep openpilot fake-engaged.
    if not self.inputs_valid:
      self.brake_hold = False
      self.brake_hold_pending = False

    # preserve the raw DAS_3 fields for safe modification and retransmission
    self.das_3 = dict(cp.vl["DAS_3"])

    if self.brake_hold and not ret.cruiseState.enabled and ret.standstill and self.forward_gear:
      ret.cruiseState.enabled = ret.cruiseState.available
      ret.cruiseState.standstill = True


class BrakeHoldCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.brake_hold_enabled = bool(CP_SP.flags & ChryslerFlagsSP.JEEP_BRAKE_HOLD)

    self.hold_decel = HOLD_DECEL
    self.last_das_3_counter = -1
    self.last_button_counter = -1
    self.button_frame = 0
    self.das_3_stale_frames = 0

  def create_brake_hold(self, packer, CC: structs.CarControl, CS: CarStateBase, frame: int) -> list[CanData]:
    """DAS_3 brake hold injection. Buttons are handled by cruise_buttons()."""
    can_sends: list[CanData] = []

    if not self.brake_hold_enabled:
      return can_sends

    counter_changed = CS.das_3.get("COUNTER") != self.last_das_3_counter
    self.last_das_3_counter = CS.das_3.get("COUNTER")

    # Dead-bus detection: the DASM broadcasts DAS_3 at 50Hz, so its counter
    # must change at least every other 100Hz control frame. A frozen counter
    # means bus 0 is down (e.g. bus-off after a transmit collision). Stop
    # transmitting immediately — continuing to inject frames prevents the CAN
    # controller's bus-off recovery from ever succeeding — and drop all hold
    # state, since every deactivation condition below reads frozen data and
    # would hold forever. inputs_valid likewise covers parser invalidity, bus
    # timeout, and ESC speed SNA (carstate and the safety layer disagree about
    # standstill during SNA, so no hold frame would be accepted anyway).
    self.das_3_stale_frames = 0 if counter_changed else self.das_3_stale_frames + 1
    if self.das_3_stale_frames >= DAS_3_STALE_LIMIT or not CS.inputs_valid:
      if CS.brake_hold or CS.brake_hold_pending:
        CS.brake_hold = False
        CS.brake_hold_pending = False
        carlog.error("Jeep brake hold: inputs stale or invalid, releasing hold")
      return can_sends

    # --- two-stage activation ---
    # Stage 1: arm when the ACC is braking to its standstill (~0.8 m/s creep).
    # No cancel is sent — the stock ACC times out on its own (jvePilot).
    if (not CS.brake_hold and not CS.brake_hold_pending and
        CS.cruise_active_actual and CS.acc_decelerating and CS.das_3.get("ACC_STANDSTILL")):
      CS.brake_hold_pending = True
      self.hold_decel = HOLD_DECEL
      carlog.info("Jeep brake hold: armed")

    # Disarm if the driver intervenes before the car stops, or if it never
    # actually stops (e.g. the lead keeps rolling)
    if CS.brake_hold_pending and (CS.out.gasPressed or CS.out.brakePressed or
                                  not CS.forward_gear or not CS.cruise_available_actual or
                                  not CS.cruise_active_actual or CS.out.vEgo > 1.0):
      CS.brake_hold_pending = False
      carlog.info("Jeep brake hold: disarmed")

    # Stage 2: activate once fully stopped (ESP_8 speed == 0). The safety
    # layer permits DAS_3 TX only at this point.
    if CS.brake_hold_pending and CS.vehicle_stopped:
      CS.brake_hold_pending = False
      CS.brake_hold = True
      carlog.info("Jeep brake hold: active")

    # --- deactivation: driver intervention or the car is moving again ---
    if CS.brake_hold and (CC.cruiseControl.cancel or CS.out.gasPressed or CS.out.brakePressed or
                          not CS.cruise_available_actual or not CS.forward_gear or CS.out.vEgo > 0.5):
      CS.brake_hold = False
      carlog.info("Jeep brake hold: deactivating")
      return can_sends

    if CS.brake_hold:
      if CS.cruise_active_actual:
        # the stock ACC is still (or again) active and braking; track its decel
        self.hold_decel = min(self.hold_decel, CS.das_3.get("ACC_DECEL", HOLD_DECEL))
      elif counter_changed:
        # ACC timed out: keep the brakes applied with our own DAS_3 messages.
        # These continue uninterrupted during resume presses (jvePilot).
        # Transmit-on-receive: send only on the control frame right after a
        # fresh DASM DAS_3 arrived, so our frame lands in the quiet part of
        # its 50Hz cycle. Free-running 100Hz transmission can phase-align
        # with the DASM's own DAS_3 (same arbitration ID, different payload)
        # and drive the bus off within milliseconds — observed 2026-07-20.
        can_sends.append(create_das_3_command(packer, 2,
                                              False,            # go
                                              False,            # torque_req
                                              None,             # torque
                                              2,                # max_gear
                                              False,            # stop (standstill)
                                              self.hold_decel,  # brake
                                              False,            # brake_prep
                                              CS.das_3))

    return can_sends

  def cruise_buttons(self, packer, CC: structs.CarControl, CS: CarStateBase, das_bus: int) -> list[CanData]:
    """Single authority for CRUISE_BUTTONS during brake hold states (jvePilot
    wheel_button_control): paced to the steering column's own counter, at most
    one message per tick, cancel wins over resume."""
    can_sends: list[CanData] = []

    # a cancel request drops the hold immediately, before the counter
    # freshness check — a one-cycle cancel must never be lost to a stalled
    # column counter and leave the hold armed
    if CC.cruiseControl.cancel:
      CS.brake_hold = False
      CS.brake_hold_pending = False

    # only transmit on a fresh column counter tick
    if CS.button_counter == self.last_button_counter:
      return can_sends
    self.last_button_counter = CS.button_counter
    self.button_frame += 1

    if CC.cruiseControl.cancel:
      can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter + 1, das_bus, cancel=True))
    elif (CC.cruiseControl.resume and CS.brake_hold and not CS.out.brakePressed and
          not CS.out.gasPressed and CS.forward_gear and CS.cruise_available_actual):
      # resume press paced across counter ticks to satisfy the DASM debounce.
      # Gated on the full hold-still-valid set: a resume with the driver on
      # the gas or out of drive must never reach the DASM.
      counter_offset = RESUME_COUNTER_OFFSETS[self.button_frame % len(RESUME_COUNTER_OFFSETS)]
      if counter_offset is not None:
        can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter + counter_offset, das_bus, resume=True))

    return can_sends
