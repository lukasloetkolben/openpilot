#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"
#include "opendbc/safety/modes/volkswagen_meb.h"

// MQB Evo V1: MQB RX signals (ESP_19, TSK_06, Motor_20, ESP_05) + MEB TX steering (HCA_03 curvature)
// For cars like Tiguan MK3 that use MQB message IDs but MQBevo steering interface

static safety_config volkswagen_mqb_evo_v1_init(uint16_t param) {
  // TX: HCA_03 curvature steering + MQB-style ACC + KLR_01 capacitive wheel
  static const CanMsg VOLKSWAGEN_MQB_EVO_V1_STOCK_TX_MSGS[] = {
    {MSG_HCA_03, 0, 24, .check_relay = true},
    {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
    {MSG_GRA_ACC_01, 2, 8, .check_relay = false},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_LH_EPS_03, 2, 8, .check_relay = true},
    {MSG_KLR_01, 0, 8, .check_relay = false},
    {MSG_KLR_01, 2, 8, .check_relay = true},
  };

  // RX: MQB messages but without Motor_14 (not present on Tiguan MK3)
  // Brake detection uses ESP_05 only
  static RxCheck volkswagen_mqb_evo_v1_rx_checks[] = {
    {.msg = {{MSG_ESP_19, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_TSK_06, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_20, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, 33U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  return BUILD_SAFETY_CFG(volkswagen_mqb_evo_v1_rx_checks, VOLKSWAGEN_MQB_EVO_V1_STOCK_TX_MSGS);
}

static void volkswagen_mqb_evo_v1_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state from ESP_19 wheel speeds (MQB style)
    if (msg->addr == MSG_ESP_19) {
      int speed = 0;
      for (uint8_t i = 0U; i < 8U; i += 2U) {
        int wheel_speed = msg->data[i] | (msg->data[i + 1U] << 8);
        speed += wheel_speed;
      }
      vehicle_moving = speed > 0;
    }

    // Update driver input torque from LH_EPS_03 (shared MQB/MEB)
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Cruise state from TSK_06 (MQB style)
    if (msg->addr == MSG_TSK_06) {
      int acc_status = (msg->data[3] & 0x7U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }

      if (!acc_main_on) {
        controls_allowed = false;
      }
    }

    // ACC buttons from GRA_ACC_01 (shared MQB/MEB)
    if (msg->addr == MSG_GRA_ACC_01) {
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // Gas pedal from Motor_20 (MQB style)
    if (msg->addr == MSG_MOTOR_20) {
      gas_pressed = ((GET_BYTES(msg, 0, 4) >> 12) & 0xFFU) != 0U;
    }

    // Brake from ESP_05 only (Motor_14 not present on this car)
    if (msg->addr == MSG_ESP_05) {
      volkswagen_brake_pressure_detected = GET_BIT(msg, 26U);
    }

    brake_pressed = volkswagen_brake_pressure_detected;
  }
}

static bool volkswagen_mqb_evo_v1_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Safety check for HCA_03 curvature steering (MEB style)
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_raw = GET_BYTES(msg, 3, 2) & 0x7FFFU;

    bool desired_curvature_sign = GET_BIT(msg, 39U);
    if (!desired_curvature_sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = (((msg->data[1] >> 4) & 0x0FU) == 4U);
    int steer_power = msg->data[2];

    if (steer_power_cmd_checks(steer_power, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }

    if (steer_curvature_cmd_checks_average(desired_curvature_raw, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    if ((msg->data[2] & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks volkswagen_mqb_evo_v1_hooks = {
  .init = volkswagen_mqb_evo_v1_init,
  .rx = volkswagen_mqb_evo_v1_rx_hook,
  .tx = volkswagen_mqb_evo_v1_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
