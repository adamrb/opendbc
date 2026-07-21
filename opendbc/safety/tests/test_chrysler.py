#!/usr/bin/env python3
import unittest

from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.sunnypilot.car.chrysler.values_ext import ChryslerSafetyFlagsSP


class TestChryslerSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x2D9, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6, 0x2D9)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6, 0x2D9]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_safety("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_safety("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume/accel/decel only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(accel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(decel=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # invalid: more than one button pressed
      combos = [
        # 2 buttons
        {"cancel": True, "resume": True},
        {"cancel": True, "accel": True},
        {"cancel": True, "decel": True},
        {"resume": True, "accel": True},
        {"resume": True, "decel": True},
        {"accel": True, "decel": True},

        # 3 buttons
        {"cancel": True, "resume": True, "accel": True},
        {"cancel": True, "resume": True, "decel": True},
        {"cancel": True, "accel": True, "decel": True},
        {"resume": True, "accel": True, "decel": True},

        # all 4 buttons
        {"cancel": True, "resume": True, "accel": True, "decel": True},
      ]

      for combo in combos:
        with self.subTest(combo=combo):
          self.assertFalse(self._tx(self._button_msg(**combo)))

  def _lkas_button_msg(self, enabled):
    values = {"TOGGLE_LKAS": enabled}
    return self.packer.make_can_msg_safety("TRACTION_BUTTON", 0, values)


class TestChryslerRamDTSafety(TestChryslerSafety):
  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [350]

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_ram_dt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_DT)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_safety("Center_Stack_2", 0, values)


class TestChryslerRamHDSafety(TestChryslerSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE_LOOKUP = [0], [361]
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_ram_hd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_HD)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_safety("Center_Stack_2", 0, values)


class TestChryslerJeepBrakeHoldSafety(TestChryslerSafety):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x2D9, 0], [0x1F4, 0]]

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(ChryslerSafetyFlagsSP.JEEP_BRAKE_HOLD)
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, 0)
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _das_3_msg(self, acc_go=False, torque_req=False, decel_req=True, decel=-2.0):
    values = {"ACC_AVAILABLE": 1, "ACC_ACTIVE": 1, "ACC_GO": acc_go,
              "ENGINE_TORQUE_REQUEST_MAX": torque_req,
              "ACC_DECEL_REQ": 1 if decel_req else 0, "ACC_DECEL": decel}
    return self.packer.make_can_msg_safety("DAS_3", 0, values)

  def _acc_state_rx(self, active, available=True):
    values = {"ACC_ACTIVE": active, "ACC_AVAILABLE": available}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _reset(self):
    """Full reinit: re-runs chrysler_init (epoch/credit globals) and clears
    pedal state left by a previous loop iteration. init_tests() zeroes
    current_safety_param_sp, so it must be restored before the hooks re-run."""
    self.safety.set_current_safety_param_sp(ChryslerSafetyFlagsSP.JEEP_BRAKE_HOLD)
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, 0)
    self.safety.init_tests()
    self.safety.set_current_safety_param_sp(ChryslerSafetyFlagsSP.JEEP_BRAKE_HOLD)

  def test_buttons(self):
    """Override: at standstill in brake hold mode, resume/accel/decel follow
    the hold-specific rules (see the dedicated tests below); while moving,
    the standard controls_allowed rule applies."""
    self._rx(self._speed_msg(10))
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(accel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(decel=True)))
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

  def _enter_hold(self):
    """Drive the safety state into a valid standstill hold: genuine ACC
    engagement, full stop, then the stock ACC's standstill dropout."""
    self._rx(self._user_gas_msg(0))
    self._rx(self._user_brake_msg(False))
    self._rx(self._speed_msg(0))
    self._rx(self._acc_state_rx(active=True))    # genuine engagement (epoch set)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._acc_state_rx(active=False))   # stock ACC standstill dropout
    self.assertTrue(self.safety.get_controls_allowed())  # latched

  # --- controls latch ---

  def test_controls_latched_through_standstill_acc_dropout(self):
    self._enter_hold()

    # latch releases once moving without re-engagement
    self._rx(self._speed_msg(10))
    self._rx(self._acc_state_rx(active=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_latch_when_acc_main_off(self):
    self._enter_hold()
    self._rx(self._acc_state_rx(active=False, available=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_latch_without_prior_controls(self):
    # ACC off at standstill must not grant controls out of thin air
    self.safety.set_controls_allowed(False)
    self._rx(self._speed_msg(0))
    self._rx(self._acc_state_rx(active=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_latch_without_engagement_epoch(self):
    # controls_allowed set externally (e.g. stale from a previous ignition
    # cycle) must not latch without a genuine ACC rising edge this epoch
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(0))
    self._rx(self._acc_state_rx(active=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_main_off_ends_engagement_epoch(self):
    self._enter_hold()
    # ACC main off then on again: the old engagement must not revive the latch
    self._rx(self._acc_state_rx(active=False, available=False))
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_controls_allowed(True)
    self._rx(self._acc_state_rx(active=False, available=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_latch_released_by_brake(self):
    self._enter_hold()
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    # and the latch must not re-engage afterwards
    self._rx(self._acc_state_rx(active=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_latch_blocked_by_pedals(self):
    for pedal in ('gas', 'brake'):
      self._reset()
      self._rx(self._speed_msg(0))
      self._rx(self._acc_state_rx(active=True))
      if pedal == 'gas':
        self._rx(self._user_gas_msg(1))
      else:
        self._rx(self._user_brake_msg(True))
      # with a pedal held at the dropout, the latch must not extend controls
      self._rx(self._acc_state_rx(active=False))
      self.assertFalse(self.safety.get_controls_allowed(), f"{pedal=}")

  # --- resume button ---

  def test_resume_at_standstill_requires_hold_state(self):
    # resume is only allowed inside a valid hold (controls latched, stopped,
    # ACC main on, no pedals)
    self._enter_hold()
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def test_resume_blocked_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_acc_main_on(True)
    self._rx(self._speed_msg(0))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

  def test_resume_while_moving_normal_path(self):
    # while moving, resume follows the standard controls_allowed rule
    # (not brake-hold specific)
    self._enter_hold()
    self._rx(self._speed_msg(10))
    self.assertTrue(self._tx(self._button_msg(resume=True)))
    # once the latch releases on the next DASM frame, resume is blocked
    self._rx(self._acc_state_rx(active=False))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

  def test_resume_blocked_with_pedals(self):
    for pedal in ('gas', 'brake'):
      self._reset()
      self._enter_hold()
      if pedal == 'gas':
        self._rx(self._user_gas_msg(1))
      else:
        self._rx(self._user_brake_msg(True))
      self.assertFalse(self._tx(self._button_msg(resume=True)), f"{pedal=}")

  def test_accel_decel_blocked_at_standstill(self):
    # accel/decel alongside a hold/resume is the contradictory-button
    # pattern that hard-faults the DASM
    self._enter_hold()
    self.assertFalse(self._tx(self._button_msg(accel=True)))
    self.assertFalse(self._tx(self._button_msg(decel=True)))
    # cancel is always allowed
    self.assertTrue(self._tx(self._button_msg(cancel=True)))

  # --- DAS_3 hold TX ---

  def test_das_3_requires_controls_allowed(self):
    self.safety.set_controls_allowed(False)
    self._rx(self._speed_msg(0))
    self._rx(self._acc_state_rx(active=False))  # grants tx credit, not controls
    self.assertFalse(self._tx(self._das_3_msg()))

  def test_das_3_brake_hold_only(self):
    self._enter_hold()

    # standstill: only pure braking commands allowed, one TX per rx credit
    self._rx(self._acc_state_rx(active=False))
    self.assertTrue(self._tx(self._das_3_msg()))
    for bad in (dict(acc_go=True), dict(torque_req=True), dict(decel_req=False), dict(decel=1.0)):
      self._rx(self._acc_state_rx(active=False))
      self.assertFalse(self._tx(self._das_3_msg(**bad)), f"{bad=}")

    # never while moving
    self._rx(self._speed_msg(10))
    self._rx(self._acc_state_rx(active=False))
    self._rx(self._speed_msg(10))
    self.assertFalse(self._tx(self._das_3_msg()))

  def test_das_3_blocked_with_pedals(self):
    for pedal in ('gas', 'brake'):
      self._reset()
      self._enter_hold()
      # pedal press releases the latch and blocks the frame directly
      if pedal == 'gas':
        self._rx(self._user_gas_msg(1))
      else:
        self._rx(self._user_brake_msg(True))
      self._rx(self._acc_state_rx(active=False))
      self.assertFalse(self._tx(self._das_3_msg()), f"{pedal=}")

  def test_das_3_rate_limited_one_tx_per_rx(self):
    # at most one of our DAS_3 frames per received DASM frame: an unpaced
    # stream can collide with the DASM's own DAS_3 and bus-off the CAN
    # controller (observed 2026-07-20)
    self._enter_hold()
    self._rx(self._acc_state_rx(active=False))
    self.assertTrue(self._tx(self._das_3_msg()))
    self.assertFalse(self._tx(self._das_3_msg()))  # credit consumed
    self._rx(self._acc_state_rx(active=False))
    self.assertTrue(self._tx(self._das_3_msg()))   # fresh credit


if __name__ == "__main__":
  unittest.main()
