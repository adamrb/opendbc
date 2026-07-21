#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/chrysler_common.h"

// Chrysler Pacifica/Jeep addresses
#define CHRYSLER_EPS_2            0x220  // EPS driver input torque
#define CHRYSLER_ESP_1            0x140  // Brake pedal and vehicle speed
#define CHRYSLER_ESP_8            0x11C  // Brake pedal and vehicle speed
#define CHRYSLER_ECM_5            0x22F  // Throttle position sensor
#define CHRYSLER_DAS_3            0x1F4  // ACC engagement states from DASM
#define CHRYSLER_DAS_6            0x2A6  // LKAS HUD and auto headlight control from DASM
#define CHRYSLER_LKAS_COMMAND     0x292  // LKAS controls from DASM
#define CHRYSLER_CRUISE_BUTTONS   0x23B  // Cruise control buttons
#define CHRYSLER_LKAS_HEARTBIT    0x2D9  // LKAS HEARTBIT from DASM
#define CHRYSLER_TRACTION_BUTTON  0x330  // Traction control button
#define CHRYSLER_Center_Stack_2   0x000  // Placeholder, does not exist

// RAM DT addresses
#define CHRYSLER_RAM_DT_EPS_2            0x31
#define CHRYSLER_RAM_DT_ESP_1            0x83
#define CHRYSLER_RAM_DT_ESP_8            0x79
#define CHRYSLER_RAM_DT_ECM_5            0x9D
#define CHRYSLER_RAM_DT_DAS_3            0x99
#define CHRYSLER_RAM_DT_DAS_6            0xFA
#define CHRYSLER_RAM_DT_LKAS_COMMAND     0xA6
#define CHRYSLER_RAM_DT_CRUISE_BUTTONS   0xB1
#define CHRYSLER_RAM_DT_LKAS_HEARTBIT    0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_DT_TRACTION_BUTTON  0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_DT_Center_Stack_2   0x28A

// RAM HD addresses
#define CHRYSLER_RAM_HD_EPS_2            0x220
#define CHRYSLER_RAM_HD_ESP_1            0x140
#define CHRYSLER_RAM_HD_ESP_8            0x11C
#define CHRYSLER_RAM_HD_ECM_5            0x22F
#define CHRYSLER_RAM_HD_DAS_3            0x1F4
#define CHRYSLER_RAM_HD_DAS_6            0x275
#define CHRYSLER_RAM_HD_LKAS_COMMAND     0x276
#define CHRYSLER_RAM_HD_CRUISE_BUTTONS   0x23A
#define CHRYSLER_RAM_HD_LKAS_HEARTBIT    0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_HD_TRACTION_BUTTON  0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_HD_Center_Stack_2   0x28A

typedef enum {
  CHRYSLER_RAM_DT,
  CHRYSLER_RAM_HD,
  CHRYSLER_PACIFICA,  // plus Jeep
} ChryslerPlatform;
static ChryslerPlatform chrysler_platform;
static bool chrysler_jeep_brake_hold = false;
// one brake-hold DAS_3 TX permitted per DASM DAS_3 received: same-ID transmit
// collision with the DASM's own frame drives the bus off (observed 2026-07-20)
static bool chrysler_das_3_tx_credit = false;
// set on a genuine ACC engagement (rising edge from the DASM), cleared when
// ACC main drops or the safety mode re-initializes: the standstill controls
// latch must never extend an engagement from a previous ignition cycle
static bool chrysler_acc_engaged_this_cycle = false;

#define CHRYSLER_ADDR(name) ((uint32_t)((chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_##name : \
                                        ((chrysler_platform == CHRYSLER_RAM_HD) ? CHRYSLER_RAM_HD_##name : CHRYSLER_##name)))


static uint8_t chrysler_get_counter(const CANPacket_t *msg) {
  return (uint8_t)(msg->data[6] >> 4);
}

static void chrysler_rx_hook(const CANPacket_t *msg) {
  // Measured EPS torque
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(EPS_2))) {
    int torque_meas_new = ((msg->data[4] & 0x7U) << 8) + msg->data[5] - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  const unsigned int das_3_bus = (chrysler_platform == CHRYSLER_PACIFICA) ? 0U : 2U;
  if ((msg->bus == das_3_bus) && (msg->addr == CHRYSLER_ADDR(DAS_3))) {
    bool cruise_engaged = GET_BIT(msg, 21U);
    acc_main_on = GET_BIT(msg, 20U);
    if (cruise_engaged) {
      chrysler_acc_engaged_this_cycle = true;
    }
    if (!acc_main_on) {
      // ACC main off (driver switch or DASM re-init after an ignition cycle)
      // ends the engagement epoch
      chrysler_acc_engaged_this_cycle = false;
    }
    // Jeep brake hold: stock ACC auto-cancels ~3s after standstill. Latch
    // controls through that dropout while stopped with ACC main on, so the
    // brake hold and standstill resume can function. Driver brake/gas press
    // and movement without re-engagement still exit controls via the
    // standard checks. The latch requires a genuine ACC engagement in the
    // current epoch — it can extend an engagement, never originate one.
    if (chrysler_jeep_brake_hold && !cruise_engaged && controls_allowed && !vehicle_moving && acc_main_on &&
        !gas_pressed && !brake_pressed && chrysler_acc_engaged_this_cycle) {
      cruise_engaged = true;
    }
    pcm_cruise_check(cruise_engaged);
    // each received DASM frame grants one brake-hold DAS_3 transmission
    chrysler_das_3_tx_credit = true;
  }

  // TODO: use the same message for both
  // update vehicle moving
  if ((chrysler_platform != CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ESP_8))) {
    vehicle_moving = ((msg->data[4] << 8) + msg->data[5]) != 0U;
  }
  if ((chrysler_platform == CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == 514U)) {
    int speed_l = (msg->data[0] << 4) + (msg->data[1] >> 4);
    int speed_r = (msg->data[2] << 4) + (msg->data[3] >> 4);
    vehicle_moving = (speed_l != 0) || (speed_r != 0);
  }

  // exit controls on rising edge of gas press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ECM_5))) {
    gas_pressed = msg->data[0U] != 0U;
  }

  // exit controls on rising edge of brake press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ESP_1))) {
    brake_pressed = ((msg->data[0U] & 0xFU) >> 2U) == 1U;
  }

  if ((chrysler_platform == CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(TRACTION_BUTTON))) {
    mads_button_press = GET_BIT(msg, 53U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
  }

  if ((chrysler_platform != CHRYSLER_PACIFICA) && (msg->bus == 0U)) {
    if (msg->addr == CHRYSLER_ADDR(Center_Stack_2)) {
      mads_button_press = GET_BIT(msg, 57U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
}

static bool chrysler_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits CHRYSLER_STEERING_LIMITS = {
    .max_torque = 261,
    .max_rt_delta = 112,
    .max_rate_up = 3,
    .max_rate_down = 3,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_DT_STEERING_LIMITS = {
    .max_torque = 350,
    .max_rt_delta = 112,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_HD_STEERING_LIMITS = {
    .max_torque = 361,
    .max_rt_delta = 182,
    .max_rate_up = 14,
    .max_rate_down = 14,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  // STEERING
  if (msg->addr == CHRYSLER_ADDR(LKAS_COMMAND)) {
    int start_byte = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 1;
    int desired_torque = ((msg->data[start_byte] & 0x7U) << 8) | msg->data[start_byte + 1];
    desired_torque -= 1024;

    const TorqueSteeringLimits limits = (chrysler_platform == CHRYSLER_PACIFICA) ? CHRYSLER_STEERING_LIMITS :
                                        (chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_STEERING_LIMITS : CHRYSLER_RAM_HD_STEERING_LIMITS;

    bool steer_req = (chrysler_platform == CHRYSLER_PACIFICA) ? GET_BIT(msg, 4U) : (msg->data[3] & 0x7U) == 2U;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (msg->addr == CHRYSLER_ADDR(CRUISE_BUTTONS)) {
    const bool is_cancel = msg->data[0] == 1U;
    const bool is_accel = msg->data[0] == 0x04U;
    const bool is_decel = msg->data[0] == 0x08U;
    const bool is_resume = msg->data[0] == 0x10U;
    // Jeep brake hold: allow RESUME at standstill with ACC main on. With the
    // controls latch in the rx hook, controls_allowed now survives the stock
    // ACC's standstill dropout, so resume is also gated on it — plus no
    // pedals, matching the conditions under which the hold itself is valid.
    const bool resume_standstill_allowed = chrysler_jeep_brake_hold && is_resume && acc_main_on &&
                                           !vehicle_moving && controls_allowed && !gas_pressed && !brake_pressed;
    // at standstill with the brake hold flag, only cancel and the gated
    // resume are allowed: accel/decel presses alongside a concurrent
    // hold/resume are the contradictory-button pattern that hard-faults
    // the DASM
    const bool standstill_hold = chrysler_jeep_brake_hold && !vehicle_moving;
    const bool allowed = is_cancel ||
                         ((is_accel || is_decel) && controls_allowed && !standstill_hold) ||
                         (is_resume && controls_allowed && !standstill_hold) ||
                         resume_standstill_allowed;
    if (!allowed) {
      tx = false;
    }
  }

  // Jeep brake hold: DAS_3 may only hold the brakes at a standstill, never
  // command a launch (ACC_GO), engine torque, or acceleration. Gated on
  // controls_allowed (rx-timeout or driver disengage stops the hold), on
  // pedals (driver input overrides the hold), and on a per-received-frame
  // TX credit (at most one of our frames per DASM frame — an unpaced
  // stream can collide with the DASM's own DAS_3 and kill the bus).
  if (chrysler_jeep_brake_hold && (msg->addr == CHRYSLER_ADDR(DAS_3))) {
    const bool acc_go = GET_BIT(msg, 6U);
    const bool torque_req = GET_BIT(msg, 7U);
    // ACC_DECEL: 12 bits, factor 0.004885, offset -16 m/s^2. raw <= 3275 <=> decel <= 0
    const int acc_decel_raw = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
    const bool decel_req = (msg->data[4] & 0x70U) != 0U;  // ACC_DECEL_REQ
    const bool decelerating = decel_req && (acc_decel_raw <= 3275);
    if (vehicle_moving || acc_go || torque_req || !decelerating ||
        !controls_allowed || gas_pressed || brake_pressed || !chrysler_das_3_tx_credit) {
      tx = false;
    } else {
      chrysler_das_3_tx_credit = false;
    }
  }

  return tx;
}

static safety_config chrysler_init(uint16_t param) {
  const uint32_t CHRYSLER_PARAM_RAM_DT = 1U;  // set for Ram DT platform

  static RxCheck chrysler_ram_dt_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_DT_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_rx_checks[] = {
    {.msg = {{CHRYSLER_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{514, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_DAS_3, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_TRACTION_BUTTON, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_TX_MSGS[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_LKAS_HEARTBIT, 0, 5, .check_relay = true},
  };

  // Jeep brake hold: DAS_3 is sent alongside the stock DASM's message, so the relay check must not apply
  static const CanMsg chrysler_jeep_brake_hold_tx_msgs[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_LKAS_HEARTBIT, 0, 5, .check_relay = true},
    {CHRYSLER_DAS_3, 0, 8, .check_relay = false},
  };

  static const CanMsg CHRYSLER_RAM_DT_TX_MSGS[] = {
    {CHRYSLER_RAM_DT_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_DT_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_6, 0, 8, .check_relay = true},
  };

#ifdef ALLOW_DEBUG
  static RxCheck chrysler_ram_hd_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_HD_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
    {CHRYSLER_RAM_HD_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_HD_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_HD_DAS_6, 0, 8, .check_relay = true},
  };

  const uint32_t CHRYSLER_PARAM_RAM_HD = 2U;  // set for Ram HD platform
  bool enable_ram_hd = GET_FLAG(param, CHRYSLER_PARAM_RAM_HD);
#endif

  safety_config ret;

  bool enable_ram_dt = GET_FLAG(param, CHRYSLER_PARAM_RAM_DT);

  const uint16_t CHRYSLER_PARAM_SP_JEEP_BRAKE_HOLD = 1;
  chrysler_jeep_brake_hold = GET_FLAG(current_safety_param_sp, CHRYSLER_PARAM_SP_JEEP_BRAKE_HOLD);
  chrysler_das_3_tx_credit = false;
  chrysler_acc_engaged_this_cycle = false;

  if (enable_ram_dt) {
    chrysler_platform = CHRYSLER_RAM_DT;
    chrysler_jeep_brake_hold = false;
    ret = BUILD_SAFETY_CFG(chrysler_ram_dt_rx_checks, CHRYSLER_RAM_DT_TX_MSGS);
#ifdef ALLOW_DEBUG
  } else if (enable_ram_hd) {
    chrysler_platform = CHRYSLER_RAM_HD;
    chrysler_jeep_brake_hold = false;
    ret = BUILD_SAFETY_CFG(chrysler_ram_hd_rx_checks, CHRYSLER_RAM_HD_TX_MSGS);
#endif
  } else {
    chrysler_platform = CHRYSLER_PACIFICA;
    ret = chrysler_jeep_brake_hold ? BUILD_SAFETY_CFG(chrysler_rx_checks, chrysler_jeep_brake_hold_tx_msgs) : \
                                     BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_TX_MSGS);
  }
  return ret;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .get_counter = chrysler_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};
