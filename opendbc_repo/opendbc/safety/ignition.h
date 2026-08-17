#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "opendbc/safety/can.h"

bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

void ignition_can_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    int len = GET_LEN(msg);

    // GM exception
    if ((msg->addr == 0x1F1U) && (len == 8)) {
      // SystemPowerMode (2=Run, 3=Crank Request)
      ignition_can = (msg->data[0] & 0x2U) != 0U;
      ignition_can_cnt = 0U;
    }

    // Rivian R1S/T GEN1 exception
    if ((msg->addr == 0x152U) && (len == 8)) {
      // 0x152 overlaps with Subaru pre-global which has this bit as the high beam
      int counter = msg->data[1] & 0xFU;  // max is only 14

      static int prev_counter_rivian = -1;
      if ((counter == ((prev_counter_rivian + 1) % 15)) && (prev_counter_rivian != -1)) {
        // VDM_OutputSignals->VDM_EpasPowerMode
        ignition_can = ((msg->data[7] >> 4U) & 0x3U) == 1U;  // VDM_EpasPowerMode_Drive_On=1
        ignition_can_cnt = 0U;
      }
      prev_counter_rivian = counter;
    }

    // Tesla Model 3/Y exception
    if ((msg->addr == 0x118U) && (len == 8)) {
      int counter = msg->data[1] & 0x0FU;  // DI_systemStatusCounter

      static int prev_counter_tesla = -1;
      if ((counter == ((prev_counter_tesla + 1) % 16)) && (prev_counter_tesla != -1)) {
        // DI_systemStatus->DI_gear
        int gear = (msg->data[2] >> 5U) & 0x7U;
        // ignition on in any gear out of park
        ignition_can = gear != 0x1;  // DI_GEAR_P=1
        ignition_can_cnt = 0U;
      }
      prev_counter_tesla = counter;
    }

    // Mazda exception
    if ((msg->addr == 0x9EU) && (len == 8)) {
      ignition_can = (msg->data[0] >> 5) == 0x6U;
      ignition_can_cnt = 0U;
    }

    // Volkswagen MEB exception
    if ((msg->addr == 0x3C0U) && (len == 4)) {
      int counter = msg->data[1] & 0xFU;

      static int prev_counter_vw_meb = -1;
      if ((counter == ((prev_counter_vw_meb + 1) % 16)) && (prev_counter_vw_meb != -1)) {
        // Klemmen_Status_01->ZAS_Kl_15
        ignition_can = ((msg->data[2] >> 1) & 1U) != 0U;
        ignition_can_cnt = 0U;
      }
      prev_counter_vw_meb = counter;
    }
  }

  // TODO: this is too loose, Teslas have 0x222
  // body v2 exception
  // if (((msg->bus == 0U) || (msg->bus == 2U)) && (msg->addr == 0x222U)) {
  //   ignition_can = true;
  //   ignition_can_cnt = 0U;
  // }
}
