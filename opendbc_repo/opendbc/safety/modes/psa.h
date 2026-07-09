#pragma once

#include "opendbc/safety/declarations.h"

#define PSA_STEERING              757U  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773U  // RX from EPS, steering angle
#define PSA_DYN_CMM               520U  // RX from CMM, gas pedal
#define PSA_DYN4_FRE              781U  // RX from ABS, wheel speeds
#define PSA_HS2_DYN_ABR_38D       909U  // RX from UC_FREIN, speed
#define PSA_DAT_BSI               1042U // RX from BSI, brake
#define PSA_LANE_KEEP_ASSIST      1010U // angle request, computed by car-side LKA ECU from the lane lines
#define PSA_LKAS_CAM_LANE_LEFT    1067U // TX by OP, left lane line to LKA ECU
#define PSA_LKAS_CAM_LANE_RIGHT   1099U // TX by OP, right lane line to LKA ECU

// CAN bus
#define PSA_MAIN_BUS 0U
#define PSA_ADAS_BUS 1U
#define PSA_CAM_BUS  2U

static uint8_t psa_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    cnt = (msg->data[5] >> 4) & 0xFU;
  } else {
  }
  return cnt;
}

static uint32_t psa_get_checksum(const CANPacket_t *msg) {
  return msg->data[5] & 0xFU;
}

static uint8_t _psa_compute_checksum(const CANPacket_t *msg, uint8_t chk_ini, int chk_pos) {
  int len = GET_LEN(msg);

  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    uint8_t b = msg->data[i];

    if (i == chk_pos) {
      // set checksum in low nibble to 0
      b &= 0xF0U;
    }
    sum += (b >> 4) + (b & 0xFU);
  }
  return (chk_ini - sum) & 0xFU;
}

static uint32_t psa_compute_checksum(const CANPacket_t *msg) {
  uint8_t chk = 0;
  if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    chk = _psa_compute_checksum(msg, 0x7, 5);
  } else {
  }
  return chk;
}

static void psa_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == PSA_MAIN_BUS) {
    if (msg->addr == PSA_DYN_CMM) {
      gas_pressed = msg->data[3] > 0U; // P002_Com_rAPP
    }
    if (msg->addr == PSA_STEERING_ALT) {
      int angle_meas_new = to_signed((msg->data[0] << 8) | msg->data[1], 16); // ANGLE, 0.1 deg
      update_sample(&angle_meas, angle_meas_new);
      // measured curvature in LINE_CURVATURE CAN units: deg2rad(0.1 * angle / (SR * WB)) * curvature_to_can
      // SR * WB ~= 47.7 (fit from stock LKA operation): 0.1 * (pi / 180) / 47.7 * 32787 ~= 1.2
      update_sample(&curvature_state.meas, ROUND((float)angle_meas_new * 1.2f));
    }
    if (msg->addr == PSA_DYN4_FRE) {
      // front wheel speed average as second speed source (0.01 km/h)
      int fl = (msg->data[0] << 8) | msg->data[1];
      int fr = (msg->data[2] << 8) | msg->data[3];
      UPDATE_VEHICLE_SPEED_2((fl + fr) * 0.005 * KPH_TO_MS);
    }
    if (msg->addr == PSA_HS2_DYN_ABR_38D) {
      int speed = (msg->data[0] << 8) | msg->data[1];
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS); // VITESSE_VEHICULE_ROUES
    }
    if (msg->addr == PSA_DAT_BSI) {
      brake_pressed = (msg->data[0U] >> 5U) & 1U; // P013_MainBrake
    }
    if (msg->addr == PSA_LANE_KEEP_ASSIST) {
      unsigned int status = (msg->data[4] >> 2) & 0x7U;
      pcm_cruise_check((status == 3U) || (status == 4U)); // 3: AUTHORIZED, 4: ACTIVE
    }
  }
}

static bool psa_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  static const AngleSteeringLimits PSA_STEERING_LIMITS = {
    .max_angle = 3900,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, .2},
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2., .3},
    },
  };

  // Safety check for LKA
  if (msg->addr == PSA_LANE_KEEP_ASSIST) {
    // SET_ANGLE
    int desired_angle = to_signed((msg->data[6] << 6) | ((msg->data[7] & 0xFCU) >> 2), 14);
    // TORQUE_FACTOR
    bool lka_active = ((msg->data[5] & 0xFEU) >> 1) == 100U;

    if (steer_angle_cmd_checks(desired_angle, lka_active, PSA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for injected lane lines. Only the curvature may steer: heading and curvature
  // rate must be zero and the lateral position fixed, so the LKA ECU centering terms stay neutral.
  // LEFT runs the full checks, RIGHT must be identical so the pair advances rate/jerk state once per cycle.
  static const CurvatureSteeringLimits PSA_CURVATURE_LIMITS = {
    .max_curvature = 656,               // 0.02 1/m * curvature_to_can
    .curvature_to_can = 32787,          // 1 / 3.05e-5 (LINE_CURVATURE factor)
    .frequency = 20,                    // Hz
    .max_curvature_error = 66,          // 0.002 1/m * curvature_to_can
    .curvature_error_min_speed = 10.0,  // m/s
    .max_steer_power = 0,
    .inactive_curvature_is_zero = true,
  };
  static int psa_left_curvature = 0;
  static bool psa_left_tracked = false;
  static bool psa_left_ok = false;

  if ((msg->addr == PSA_LKAS_CAM_LANE_LEFT) || (msg->addr == PSA_LKAS_CAM_LANE_RIGHT)) {
    // LINE_CURVATURE, camera sign convention is opposite of the steering angle
    int desired_curvature = -to_signed(((msg->data[4] << 8) | msg->data[5]) >> 4, 12);
    bool steer_control_enabled = ((msg->data[7] >> 5) & 1U) != 0U;  // LINE_TRACKED
    unsigned int lat_position = (msg->data[6] << 2) | (msg->data[7] >> 6);  // LINE_LATERAL_POSITION

    bool violation = false;
    violation |= ((msg->data[0] << 8) | msg->data[1]) != 0;  // LINE_HEADING
    violation |= ((msg->data[2] << 8) | msg->data[3]) != 0;  // LINE_CURVATURE_RATE

    if (msg->addr == PSA_LKAS_CAM_LANE_LEFT) {
      violation |= lat_position != 112U;  // +1.75 m
      violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, PSA_CURVATURE_LIMITS);
      psa_left_curvature = desired_curvature;
      psa_left_tracked = steer_control_enabled;
      psa_left_ok = !violation;
    } else {
      violation |= lat_position != 912U;  // -1.75 m (10-bit two's complement)
      violation |= (desired_curvature != psa_left_curvature) || (steer_control_enabled != psa_left_tracked) || !psa_left_ok;
    }

    if (violation) {
      tx = false;
    }
  }
  return tx;
}

static safety_config psa_init(uint16_t param) {
  SAFETY_UNUSED(param);
  static const CanMsg PSA_TX_MSGS[] = {
    // {PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, .check_relay = false}, // EPS steering
    {PSA_LKAS_CAM_LANE_LEFT, PSA_MAIN_BUS, 8, .check_relay = true},   // lane line injection
    {PSA_LKAS_CAM_LANE_RIGHT, PSA_MAIN_BUS, 8, .check_relay = true},  // lane line injection
  };

  static RxCheck psa_rx_checks[] = {
    {.msg = {{PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // LKA status (cruise state)
    {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            // speed
    {.msg = {{PSA_DYN4_FRE, PSA_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},      // wheel speeds
    {.msg = {{PSA_STEERING_ALT, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // steering angle
    {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},     // driver torque
    {.msg = {{PSA_DYN_CMM, PSA_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},      // gas pedal
    {.msg = {{PSA_DAT_BSI, PSA_MAIN_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},       // brake
  };

  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .get_counter = psa_get_counter,
  .get_checksum = psa_get_checksum,
  .compute_checksum = psa_compute_checksum,
};
