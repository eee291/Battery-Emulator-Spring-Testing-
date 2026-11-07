#include "CMFA-EV-BATTERY.h"
#include <Arduino.h>
#include <cstring>
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/common_functions.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

uint32_t CmfaEvBattery::rescale_raw_SOC(uint32_t raw_SOC) {
  if (raw_SOC > 9500) {
    raw_SOC = 9500;
  }
  if (raw_SOC < 500) {
    raw_SOC = 500;
  }
  return ((raw_SOC - 500) * 10) / 9;
}

void CmfaEvBattery::update_values() {
  datalayer.battery.status.soh_pptt = soh_average;
  datalayer.battery.status.real_soc = soc_u;
  datalayer.battery.status.voltage_dV = (pack_voltage / 10);
  datalayer.battery.status.current_dA = pack_current_dA();

  datalayer.battery.status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer.battery.status.real_soc) / 10000) * datalayer.battery.info.total_capacity_Wh);

  if (use_uds_power_limits) {
    datalayer.battery.status.max_discharge_power_W = max_discharge_power;
    datalayer.battery.status.max_charge_power_W = max_charge_power;
  } else {
    datalayer.battery.status.max_discharge_power_W =
        datalayer.battery.status.override_discharge_power_W
            ? datalayer.battery.status.override_discharge_power_W
            : estimate_max_discharge_power_W(datalayer.battery.status.real_soc, minimum_temperature,
                                             datalayer.battery.status.voltage_dV, max_cell_voltage);
    datalayer.battery.status.max_charge_power_W =
        datalayer.battery.status.override_charge_power_W
            ? datalayer.battery.status.override_charge_power_W
            : estimate_max_charge_power_W(datalayer.battery.status.real_soc, minimum_temperature,
                                          datalayer.battery.status.voltage_dV, max_cell_voltage);
  }

  memcpy(datalayer.battery.status.cell_voltages_mV, cell_voltages, NUMBER_OF_CELLS * sizeof(uint16_t));
  datalayer.battery.status.cell_min_voltage_mV = min_cell_voltage;
  datalayer.battery.status.cell_max_voltage_mV = max_cell_voltage;
  datalayer.battery.status.temperature_min_dC = minimum_temperature;
  datalayer.battery.status.temperature_max_dC = maximum_temperature;
  datalayer.battery.status.total_charged_battery_Wh = cumulative_energy_when_charging;
  datalayer.battery.status.total_discharged_battery_Wh = cumulative_energy_when_discharging;

  if (lv_bat_mv < 11000) {
    set_event(EVENT_12V_LOW, lv_bat_mv);
  }
}

static uint32_t bal_mAh(uint16_t raw) {
  return (uint32_t)raw * 25 / 18;
}
static uint32_t bal_s(uint16_t raw) {
  return (uint32_t)raw * 10;
}

String CmfaEvBattery::get_uds_info_html() {
  String content;
  content.reserve(3072);
  auto fmt_bal = [](const char* label, uint16_t cur, uint16_t init, uint32_t (*conv)(uint16_t), const char* unit) {
    String s = "<h4>";
    s += label;
    s += ": ";
    if (cur == BAL_UNSET) {
      s += "not read yet";
    } else {
      s += String(conv(cur)) + unit + " (was " + String(conv(init)) + unit + ")";
    }
    s += "</h4>";
    return s;
  };
  content += "<h4>SOC U: " + String(soc_u / 100) + "percent</h4>";
  content += "<h4>SOC Z: " + String(soc_z / 100) + "percent</h4>";
  content += "<h4>SOH Average: " + String(soh_average) + "pptt</h4>";
  content += "<h4>Instant current: " + String(((int32_t)instant_current_raw - 32767) * 250) + "mA</h4>";
  content += "<h4>Average current: " + String((((int32_t)averaged_current_raw - 32640) * 125) / 4) + "mA</h4>";
  content += "<h4>12V voltage: " + String(lv_bat_mv) + "mV</h4>";
  content += "<h4>Highest cell number: " + String(highest_cell_number) + "</h4>";
  content += "<h4>Lowest cell number: " + String(lowest_cell_number) + "</h4>";
  content += "<h4>Sum of cellvoltages: " + String(sum_of_cell_voltages) + "dV</h4>";
  content += "<h4>Max regen power: " + String(max_regen_power) + "W</h4>";
  content += "<h4>Max discharge power: " + String(max_discharge_power) + "W</h4>";
  content += "<h4>Max charge power: " + String(max_charge_power) + "W</h4>";
  content += "<h4>Power limit source: " +
             String(use_uds_power_limits ? "UDS (pack-reported)" : "ESTIMATED (user setting)") + "</h4>";
  content += "<h4>Broadcast charge power limit (0x3D6): " + String(broadcast_charge_power_w) + "W</h4>";
  content += "<h4>Broadcast discharge power limit (0x3D6): " + String(broadcast_discharge_power_w) + "W</h4>";
  content += "<h4>Broadcast charge power request (0x127): " + String(broadcast_127_charge_power_w) + "W</h4>";
  content += "<h4>Car state sent: " +
             String(car_mode == CAR_MODE_CHARGING ? "CHARGING (as a real car)" : "DRIVING / IDLE (active EVC)") +
             "</h4>";
  {
    char hex[8];
    snprintf(hex, sizeof(hex), "0x%02X", broadcast_3D7_mode_byte);
    const char* mode_str = "OTHER";
    if (broadcast_3D7_mode_byte == 0x24) {
      mode_str = "IDLE";
    } else if (broadcast_3D7_mode_byte == 0x36) {
      mode_str = "CHARGING";
    }
    content += "<h4>Pack mode (0x3D7 byte 0): " + String(mode_str) + " (" + String(hex) + ")</h4>";
  }
  {
    char hex[8];
    snprintf(hex, sizeof(hex), "0x%02X", broadcast_43C_byte0);
    bool idle_flag = (broadcast_43C_byte0 & 0x40) != 0;
    content +=
        "<h4>Pack idle flag (0x43C byte 0 bit 6): " + String(idle_flag ? "IDLE" : "CLEAR") + " (" + String(hex) + ")</h4>";
  }
  content += "<h4>BMS mode (DID 0x901E): " + String(bms_state) + "</h4>";
  content += "<h4>Request from CAN (DID 0x901F): " + String(request_from_can) + "</h4>";
  content += "<h4>End of charge flag (DID 0x9019): " + String(end_of_charge_flag ? "SET" : "CLEAR") + "</h4>";
  content += "<h4>SOH available power: " + String(soh_available_power) + "</h4>";
  content += "<h4>SOH generated power: " + String(soh_generated_power) + "</h4>";
  content += "<h4>Average temperature: " + String(average_temperature) + "dC</h4>";
  content += "<h4>Maximum temperature: " + String(maximum_temperature) + "dC</h4>";
  content += "<h4>Minimum temperature: " + String(minimum_temperature) + "dC</h4>";
  content += "<h4>Cumulative energy discharged: " + String(cumulative_energy_when_discharging) + "Wh</h4>";
  content += "<h4>Cumulative energy charged: " + String(cumulative_energy_when_charging) + "Wh</h4>";
  content += "<h4>Cumulative energy regen: " + String(cumulative_energy_in_regen) + "Wh</h4>";
  content += fmt_bal("Balance capacity total", bal_cap_total_raw, initial_bal_cap_total_raw, bal_mAh, "mAh");
  content += fmt_bal("Balance time total", bal_time_total_raw, initial_bal_time_total_raw, bal_s, "s");
  content += fmt_bal("Balance capacity sleep", bal_cap_sleep_raw, initial_bal_cap_sleep_raw, bal_mAh, "mAh");
  content += fmt_bal("Balance time sleep", bal_time_sleep_raw, initial_bal_time_sleep_raw, bal_s, "s");
  content += fmt_bal("Balance capacity wake", bal_cap_wake_raw, initial_bal_cap_wake_raw, bal_mAh, "mAh");
  content += fmt_bal("Balance time wake", bal_time_wake_raw, initial_bal_time_wake_raw, bal_s, "s");
  String temp_str = (temporisation_flag == 0xFF)
                        ? "UNKNOWN"
                        : (temporisation_flag == 1 ? "ENABLED (1)" : ("DISABLED (" + String(temporisation_flag) + ")"));
  content += "<h4>Temporisation: " + temp_str + "</h4>";
  content += "<h4>Pack time life: " + String(pack_time_life) + "m (was " + String((int32_t)initial_pack_time_life) +
             "m)</h4>";
  content += "<h4>Absolute time saved: " + String(absolute_time_saved) + "m (was " +
             String((int32_t)initial_absolute_time_saved) + "m)</h4>";
  content += "<h4>Vehicle time sent (0x3D4): " +
             (vehicle_time_valid ? (String(vehicle_time_sent) + "m") : String("not yet (waiting for pack times)")) +
             "</h4>";
  {
    String sleep_str;
    unsigned long now_ms = millis();
    bool bms_quiet = (now_ms - last_rx_ms) >= BMS_SILENT_THRESHOLD_MS;
    const char* bms_state_str =
        bms_quiet ? "BMS SILENT" : (bms_seen_silent_in_sleep ? "BMS WOKE SELF" : "BMS STILL TRANSMITTING");
    if (sleep_phase == SLEEP_STOPPING) {
      unsigned long elapsed_s = (now_ms - sleep_phase_started_ms) / 1000;
      sleep_str = "STOPPING CURRENT (" + String(elapsed_s) + "s, I=" + String(pack_current_dA()) + "dA)";
    } else if (sleep_phase == SLEEP_SHUTDOWN) {
      sleep_str = "SHUTTING DOWN (0.9s 0x44 sequence)";
    } else if (sleep_phase == SLEEP_SILENT) {
      unsigned long elapsed = now_ms - sleep_phase_started_ms;
      if (sleep_silent_duration_ms == 0) {
        sleep_str = "ASLEEP (Sleep button) for " + String(elapsed / 60000) + "m, " + bms_state_str;
      } else {
        unsigned long rem_s =
            (elapsed < sleep_silent_duration_ms) ? ((sleep_silent_duration_ms - elapsed) / 1000) : 0;
        sleep_str = "SLEEPING (" + String(rem_s) + "s left, " + bms_state_str + ")";
      }
    } else {
      sleep_str = "AWAKE";
    }
    content += "<h4>Sleep: " + sleep_str + "</h4>";
  }
  content += "<h4>Last NVROL/sleep sequence: " + last_nvrol_status + "</h4>";
  append_mission_html(content);
  return content;
}

void CmfaEvBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  last_rx_ms = millis();
  switch (rx_frame.ID) {
    case 0x127: {
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      uint8_t raw = (rx_frame.data.u8[0] >> 2) & 0x3F;
      broadcast_127_charge_power_w = (uint16_t)raw * 300;
      break;
    }
    case 0x3D6:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      broadcast_charge_power_w = (uint16_t)rx_frame.data.u8[2] * 500;
      broadcast_discharge_power_w = (uint16_t)rx_frame.data.u8[3] * 500;
      break;
    case 0x3D7:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      broadcast_3D7_mode_byte = rx_frame.data.u8[0];
      break;
    case 0x3D8:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x43C:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      broadcast_43C_byte0 = rx_frame.data.u8[0];
      break;
    case 0x5A9:
    case 0x5AB:
    case 0x5C8:
    case 0x5E1:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x7BB: {
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;

      if (uds_state != UDS_IDLE) {
        if (uds_state == UDS_DTC_READ) {
          if ((rx_frame.data.u8[0] & 0xF0) == 0x10) {
            transmit_can_frame(&CMFA_ACK);
          }
          if (uds_try_parse_dtc_ReadDTCByStatusMask_rx(&rx_frame, CMFA_UDS_RESP_ID)) {
            uds_state = UDS_IDLE;
          }
          break;
        }
        if (uds_state == UDS_DTC_CLEAR) {
          if (uds_check_ClearDiagnosticInformation_rx(&rx_frame, CMFA_UDS_RESP_ID)) {
            uds_dtc_clear_Complete();
            uds_state = UDS_IDLE;
          }
          break;
        }
        uint8_t pci_len = rx_frame.data.u8[0];
        uint8_t sid = rx_frame.data.u8[1];
        if (pci_len >= 2 && sid == 0x7F) {
          char tag[16];
          snprintf(tag, sizeof(tag), "NRC 0x%02X", rx_frame.data.u8[3]);
          sequence_step_rejected(tag);
          break;
        }
        on_uds_sequence_step();
        break;
      }

      if (rx_frame.data.u8[0] == 0x10) {
        transmit_can_frame(&CMFA_ACK);
      }

      if (uds_fast_ingest_response(&rx_frame, CMFA_UDS_RESP_ID)) {
        const UdsRxBuffer& buf = uds_fast_buffer();
        if (buf.len >= 3 && buf.data[0] == 0x62) {
          uint16_t pid = ((uint16_t)buf.data[1] << 8) | buf.data[2];
          handle_pid(pid, &buf.data[3], buf.len - 3);
        }
        uds_wait_done = true;
      } else if (rx_frame.data.u8[0] <= 0x07 && rx_frame.data.u8[1] == 0x7F) {
        uds_wait_done = true;
      }
      break;
    }
    default:
      break;
  }
}

static void decode_balance_word(uint16_t val, uint16_t& cur, uint16_t& init, const char* name, uint32_t (*conv)(uint16_t),
                                const char* unit) {
  if (init == 0xFFFF) {
    init = val;
    logging.printf("CMFA: %s initial = %lu%s (raw %u)\n", name, (unsigned long)conv(val), unit, val);
  } else if (val != cur) {
    logging.printf("CMFA: %s changed %lu%s -> %lu%s (delta %+ld%s, from initial %+ld%s)\n", name,
                   (unsigned long)conv(cur), unit, (unsigned long)conv(val), unit,
                   (long)conv(val) - (long)conv(cur), unit, (long)conv(val) - (long)conv(init), unit);
  }
  cur = val;
}

void CmfaEvBattery::handle_pid(uint16_t poll_pid, const uint8_t* data, uint8_t length) {
  if (length == 0) {
    return;
  }
  uint16_t value = (length >= 2) ? (((uint16_t)data[0] << 8) | data[1]) : data[0];
  uint32_t value32 = (length >= 4) ? (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                                      ((uint32_t)data[2] << 8) | (uint32_t)data[3])
                                   : value;
  switch (poll_pid) {
    case PID_POLL_SOC_Z:
      soc_z = (length == 1) ? ((uint32_t)data[0] * 50) : rescale_raw_SOC(value);
      break;
    case PID_POLL_SOC_U:
      soc_u = (length == 1) ? ((uint32_t)data[0] * 50) : rescale_raw_SOC(value);
      break;
    case PID_POLL_SOH_AVERAGE:
      soh_average = (length == 1) ? (uint16_t)((uint16_t)data[0] * 50) : value;
      break;
    case PID_POLL_PACK_VOLTAGE:
      pack_voltage = value;
      break;
    case PID_POLL_AVERAGE_VOLTAGE_OF_CELLS:
      sum_of_cell_voltages = (uint16_t)(((uint32_t)value * 10) / 128);
      break;
    case PID_POLL_MAX_CELL_VOLTAGE:
      max_cell_voltage = (value * 9765) / 10000;
      break;
    case PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE:
      highest_cell_number = (uint8_t)value;
      break;
    case PID_POLL_MIN_CELL_VOLTAGE:
      min_cell_voltage = (value * 9765) / 10000;
      break;
    case PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE:
      lowest_cell_number = (uint8_t)value;
      break;
    case PID_POLL_INSTANT_CURRENT:
      instant_current_raw = value;
      break;
    case PID_POLL_100MS_CURRENT:
      averaged_current_raw = value;
      break;
    case PID_POLL_MAXIMUM_GENERATED_POWER:
      max_regen_power = (uint32_t)value * 250;
      break;
    case PID_POLL_AVAILABLE_POWER:
      max_discharge_power = (uint32_t)value * 250;
      break;
    case PID_POLL_LV_BAT:
      lv_bat_mv = (value * 9765) / 10000;
      break;
    case PID_POLL_AVERAGE_TEMPERATURE:
      average_temperature = (length == 1) ? (int16_t)(((int16_t)data[0] - 40) * 10) : (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MIN_TEMPERATURE:
      minimum_temperature = (length == 1) ? (int16_t)(((int16_t)data[0] - 40) * 10) : (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MAX_TEMPERATURE:
      maximum_temperature = (length == 1) ? (int16_t)(((int16_t)data[0] - 40) * 10) : (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MAXIMUM_CHARGE_POWER:
      max_charge_power = (uint32_t)value * 250;
      break;
    case PID_POLL_END_OF_CHARGE_FLAG:
      end_of_charge_flag = (uint8_t)value;
      break;
    case PID_POLL_HVBATT_INTERLOCK_FLAG:
      interlock_flag = (uint8_t)value;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING:
      cumulative_energy_when_charging = value32;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING:
      cumulative_energy_when_discharging = value32;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_IN_REGEN:
      cumulative_energy_in_regen = value32;
      break;
    case PID_POLL_SOH_AVAILABLE_POWER:
      soh_available_power = value;
      break;
    case PID_POLL_SOH_GENERATED_POWER:
      soh_generated_power = value;
      break;
    case PID_POLL_BALANCE_CAPACITY_TOTAL:
      decode_balance_word(value, bal_cap_total_raw, initial_bal_cap_total_raw, "Balance capacity total", bal_mAh,
                          "mAh");
      break;
    case PID_POLL_BALANCE_TIME_TOTAL:
      decode_balance_word(value, bal_time_total_raw, initial_bal_time_total_raw, "Balance time total", bal_s, "s");
      break;
    case PID_POLL_BALANCE_CAPACITY_SLEEP:
      decode_balance_word(value, bal_cap_sleep_raw, initial_bal_cap_sleep_raw, "Balance capacity sleep", bal_mAh,
                          "mAh");
      break;
    case PID_POLL_BALANCE_TIME_SLEEP:
      decode_balance_word(value, bal_time_sleep_raw, initial_bal_time_sleep_raw, "Balance time sleep", bal_s, "s");
      break;
    case PID_POLL_BALANCE_CAPACITY_WAKE:
      decode_balance_word(value, bal_cap_wake_raw, initial_bal_cap_wake_raw, "Balance capacity wake", bal_mAh, "mAh");
      break;
    case PID_POLL_BALANCE_TIME_WAKE:
      decode_balance_word(value, bal_time_wake_raw, initial_bal_time_wake_raw, "Balance time wake", bal_s, "s");
      break;
    case PID_POLL_TEMPORISATION:
      if (temporisation_flag != (uint8_t)value) {
        logging.printf("CMFA: Temporisation flag = %u\n", (unsigned)value);
      }
      temporisation_flag = (uint8_t)value;
      break;
    case PID_POLL_PACK_TIME_LIFE: {
      uint32_t v = (length >= 3) ? (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]) : value;
      if (initial_pack_time_life == UINT32_MAX) {
        initial_pack_time_life = v;
      } else if (v != pack_time_life) {
        log_pack_time_change("Pack time life", pack_time_life, v);
      }
      pack_time_life = v;
      update_vehicle_time_from_pack();
      break;
    }
    case PID_POLL_ABSOLUTE_TIME_SAVED: {
      uint32_t v = (length >= 3) ? (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]) : value;
      if (initial_absolute_time_saved == UINT32_MAX) {
        initial_absolute_time_saved = v;
      } else if (v != absolute_time_saved) {
        log_pack_time_change("Absolute time saved", absolute_time_saved, v);
      }
      absolute_time_saved = v;
      update_vehicle_time_from_pack();
      break;
    }
    case PID_POLL_BMS_STATE:
      bms_state = (uint8_t)value;
      break;
    case PID_POLL_REQUEST_FROM_CAN:
      request_from_can = (uint8_t)value;
      break;

    case PID_POLL_CELL_1:
    case PID_POLL_CELL_2:
    case PID_POLL_CELL_3:
    case PID_POLL_CELL_4:
    case PID_POLL_CELL_5:
    case PID_POLL_CELL_6:
    case PID_POLL_CELL_7:
    case PID_POLL_CELL_8:
    case PID_POLL_CELL_9:
    case PID_POLL_CELL_10:
    case PID_POLL_CELL_11:
    case PID_POLL_CELL_12:
    case PID_POLL_CELL_13:
    case PID_POLL_CELL_14:
    case PID_POLL_CELL_15:
    case PID_POLL_CELL_16:
    case PID_POLL_CELL_17:
    case PID_POLL_CELL_18:
    case PID_POLL_CELL_19:
    case PID_POLL_CELL_20:
    case PID_POLL_CELL_21:
    case PID_POLL_CELL_22:
    case PID_POLL_CELL_23:
    case PID_POLL_CELL_24:
    case PID_POLL_CELL_25:
    case PID_POLL_CELL_26:
    case PID_POLL_CELL_27:
    case PID_POLL_CELL_28:
    case PID_POLL_CELL_29:
    case PID_POLL_CELL_30:
    case PID_POLL_CELL_31:
      cell_voltages[poll_pid - PID_POLL_CELL_1] = (value * 9765) / 10000;
      break;
    case PID_POLL_CELL_32:
    case PID_POLL_CELL_33:
    case PID_POLL_CELL_34:
    case PID_POLL_CELL_35:
    case PID_POLL_CELL_36:
    case PID_POLL_CELL_37:
    case PID_POLL_CELL_38:
    case PID_POLL_CELL_39:
    case PID_POLL_CELL_40:
    case PID_POLL_CELL_41:
    case PID_POLL_CELL_42:
    case PID_POLL_CELL_43:
    case PID_POLL_CELL_44:
    case PID_POLL_CELL_45:
    case PID_POLL_CELL_46:
    case PID_POLL_CELL_47:
    case PID_POLL_CELL_48:
    case PID_POLL_CELL_49:
    case PID_POLL_CELL_50:
    case PID_POLL_CELL_51:
    case PID_POLL_CELL_52:
    case PID_POLL_CELL_53:
    case PID_POLL_CELL_54:
    case PID_POLL_CELL_55:
    case PID_POLL_CELL_56:
    case PID_POLL_CELL_57:
    case PID_POLL_CELL_58:
    case PID_POLL_CELL_59:
    case PID_POLL_CELL_60:
    case PID_POLL_CELL_61:
    case PID_POLL_CELL_62:
      cell_voltages[poll_pid - PID_POLL_CELL_1 - 1] = (value * 9765) / 10000;
      break;
    case PID_POLL_CELL_63:
    case PID_POLL_CELL_64:
    case PID_POLL_CELL_65:
    case PID_POLL_CELL_66:
    case PID_POLL_CELL_67:
    case PID_POLL_CELL_68:
    case PID_POLL_CELL_69:
    case PID_POLL_CELL_70:
    case PID_POLL_CELL_71:
    case PID_POLL_CELL_72:
      cell_voltages[poll_pid - PID_POLL_CELL_1 - 2] = (value * 9765) / 10000;
      break;
    default:
      if (poll_pid >= PID_POLL_MISSION_FIRST && poll_pid <= PID_POLL_MISSION_LAST) {
        handle_mission_pid(poll_pid, data, length);
      }
      break;
  }
}

void CmfaEvBattery::update_vehicle_time_from_pack() {
  uint32_t base_pack_life = (initial_pack_time_life != UINT32_MAX) ? initial_pack_time_life : 0;
  uint32_t base_abs_time = (initial_absolute_time_saved != UINT32_MAX) ? initial_absolute_time_saved : 113502;

  uint32_t delta_min = (initial_pack_time_life != UINT32_MAX && pack_time_life > base_pack_life)
                           ? (pack_time_life - base_pack_life)
                           : (uint32_t)(millis() / 60000UL);

  uint32_t t1 = CMFA_VEHICLE_TIME_1_OFFSET + base_abs_time + delta_min;
  uint32_t t2 = base_abs_time + delta_min + 1;

  CMFA_3D4.data.u8[0] = (uint8_t)((t1 >> 16) & 0xFF);
  CMFA_3D4.data.u8[1] = (uint8_t)((t1 >> 8) & 0xFF);
  CMFA_3D4.data.u8[2] = (uint8_t)(t1 & 0xFF);
  CMFA_3D4.data.u8[3] = (uint8_t)((t2 >> 16) & 0xFF);
  CMFA_3D4.data.u8[4] = (uint8_t)((t2 >> 8) & 0xFF);
  CMFA_3D4.data.u8[5] = (uint8_t)(t2 & 0xFF);

  vehicle_time_sent = t2;
  vehicle_time_valid = true;
}

void CmfaEvBattery::log_pack_time_change(const char* name, uint32_t prev, uint32_t now) {
  logging.printf("CMFA: %s changed %lum -> %lum (delta %+ldm)\n", name, (unsigned long)prev, (unsigned long)now,
                 (long)now - (long)prev);
}

void CmfaEvBattery::handle_mission_pid(uint16_t poll_pid, const uint8_t* data, uint8_t length) {
  uint8_t block_idx = (uint8_t)(poll_pid - PID_POLL_MISSION_FIRST);
  if (block_idx >= MISSION_BLOCK_COUNT) {
    return;
  }
  uint8_t n_entries = length / 8;
  if (n_entries > MISSION_ENTRIES_PER_BLOCK) {
    n_entries = MISSION_ENTRIES_PER_BLOCK;
  }
  bool first_time_block = (mission_blocks_seen_mask & (1u << block_idx)) == 0;
  for (uint8_t i = 0; i < n_entries; i++) {
    const uint8_t* p = &data[i * 8];
    bool all_zero = true, all_ff = true;
    for (int b = 0; b < 8; b++) {
      if (p[b] != 0x00) all_zero = false;
      if (p[b] != 0xFF) all_ff = false;
    }
    MissionEntry e;
    memcpy(e.raw, p, 8);
    e.valid = !(all_zero || all_ff);
    e.t_end = (int8_t)p[0];
    e.t_init = (int8_t)p[1];
    e.soc_end = p[2];
    e.soc_init = p[3];
    e.mode_flags = p[4];
    e.charge_ah = p[5];
    e.discharge_ah = (uint8_t)(p[6] >> 2);
    e.duration_min = (uint16_t)(((p[6] & 0x03) << 8) | p[7]);

    uint8_t slot = block_idx * MISSION_ENTRIES_PER_BLOCK + i;
    MissionEntry& prev = mission_initial[slot];
    if (first_time_block || memcmp(prev.raw, e.raw, 8) != 0) {
      prev = e;
      if (e.valid) {
        mission_ring[mission_ring_head] = e;
        mission_ring_head = (mission_ring_head + 1) % MISSION_RING_SIZE;
        if (mission_ring_count < MISSION_RING_SIZE) {
          mission_ring_count++;
        }
      }
    }
  }
  mission_blocks_seen_mask |= (1u << block_idx);
}

void CmfaEvBattery::append_mission_html(String& content) {
  if (mission_blocks_seen_mask == 0) {
    content += "<h4>Mission log: not read yet</h4>";
    return;
  }
  content += "<h4>Mission log (" + String(mission_ring_count) + " recent entries):</h4>";
  if (mission_ring_count == 0) {
    return;
  }
  content += "<pre style='font-size:11px'>";
  content += " #  Mode  Dur(m) SOC i->f  T i->f  +Ah -Ah  Raw\n";
  for (uint8_t k = 0; k < mission_ring_count; k++) {
    uint8_t idx = (mission_ring_head + MISSION_RING_SIZE - 1 - k) % MISSION_RING_SIZE;
    const MissionEntry& e = mission_ring[idx];
    char line[96];
    snprintf(line, sizeof(line), "%2u  0x%02X  %5u  %3u->%-3u  %3d->%-3d %3u %3u  %02X%02X%02X%02X%02X%02X%02X%02X\n",
             (unsigned)k, e.mode_flags, e.duration_min, e.soc_init, e.soc_end, (int)e.t_init, (int)e.t_end,
             e.charge_ah, e.discharge_ah, e.raw[0], e.raw[1], e.raw[2], e.raw[3], e.raw[4], e.raw[5], e.raw[6],
             e.raw[7]);
    content += line;
  }
  content += "</pre>";
}

void CmfaEvBattery::update_car_state_frames() {
  if (car_mode == CAR_MODE_CHARGING) {
    charging_3D3_toggle = !charging_3D3_toggle;
    CMFA_3D3.data.u8[0] = 0x47;
    CMFA_3D3.data.u8[1] = 0x1F;
    CMFA_3D3.data.u8[2] = 0x00;
    CMFA_3D3.data.u8[3] = 0x02;
    CMFA_3D3.data.u8[4] = charging_3D3_toggle ? 0x5D : 0xB2;
    CMFA_3D3.data.u8[5] = 0x20;
    CMFA_3D3.data.u8[6] = charging_3D3_toggle ? 0xB2 : 0x5D;
    CMFA_3D3.data.u8[7] = 0xD9;
    CMFA_3D4.data.u8[6] = 0x98;
    CMFA_3D4.data.u8[7] = 0x22;
    CMFA_3D5.data.u8[3] = 0x60;
    uint8_t chg134[8] = {0x91, 0x8A, 0xD2, 0x3F, 0xC2, 0x4C, 0x80, 0x00};
    memcpy(CMFA_134.data.u8, chg134, 8);
  } else {
    charging_3D3_toggle = !charging_3D3_toggle;
    CMFA_3D3.data.u8[0] = 0x47;
    CMFA_3D3.data.u8[1] = 0x1E;
    CMFA_3D3.data.u8[2] = 0x00;
    CMFA_3D3.data.u8[3] = 0x01;
    CMFA_3D3.data.u8[4] = 0x5D;
    CMFA_3D3.data.u8[5] = 0xA0;
    CMFA_3D3.data.u8[6] = 0xB2;
    CMFA_3D3.data.u8[7] = charging_3D3_toggle ? 0xDB : 0xDA;
    CMFA_3D4.data.u8[6] = 0x80;
    CMFA_3D4.data.u8[7] = 0x04;
    CMFA_3D5.data.u8[3] = 0x00;
    uint8_t drv134[8] = {0x05, 0x4A, 0xCC, 0x41, 0x62, 0x4C, 0x80, 0x00};
    memcpy(CMFA_134.data.u8, drv134, 8);
  }
}

void CmfaEvBattery::start_uds_sequence(CmfaUdsState first_step, const char* label) {
  if (uds_state != UDS_IDLE) {
    logging.printf("CMFA: %s ignored, UDS busy (state=%u)\n", label, (unsigned)uds_state);
    return;
  }
  uds_state = first_step;
  uds_step_started_ms = millis();
  logging.printf("CMFA: %s sequence -> step 1: 10 03 (Extended Diagnostic Session)\n", label);
  transmit_can_frame(&CMFA_EXT_SESSION);
  if (first_step == UDS_NVROL_EXT_SESSION_1) {
    last_nvrol_status = "In progress: sent 10 03";
  }
}

void CmfaEvBattery::on_uds_sequence_step() {
  uds_step_started_ms = millis();
  switch (uds_state) {
    case UDS_TEMP_EXT_SESSION:
      uds_state = UDS_TEMP_WRITE_9281;
      logging.printf("CMFA: Temporisation sequence -> step 2: 2E 92 81 01\n");
      transmit_can_frame(&CMFA_WRITE_9281);
      break;
    case UDS_TEMP_WRITE_9281:
      logging.printf("CMFA: Temporisation enabled (0x9281 = 0x01)\n");
      uds_state = UDS_IDLE;
      break;
    case UDS_NVROL_EXT_SESSION_1:
      uds_state = UDS_NVROL_ROUTINE_B009;
      logging.printf("CMFA: NVROL sequence -> step 2: 31 01 B0 09\n");
      last_nvrol_status = "In progress: sent 31 01 B0 09";
      transmit_can_frame(&CMFA_ROUTINE_B009);
      break;
    case UDS_NVROL_ROUTINE_B009:
      uds_state = UDS_NVROL_EXT_SESSION_2;
      logging.printf("CMFA: NVROL sequence -> step 3: 10 03 (Extended Diagnostic Session)\n");
      last_nvrol_status = "In progress: sent second 10 03";
      transmit_can_frame(&CMFA_EXT_SESSION);
      break;
    case UDS_NVROL_EXT_SESSION_2:
      uds_state = UDS_NVROL_WRITE_9281;
      logging.printf("CMFA: NVROL sequence -> step 4: 2E 92 81 01\n");
      last_nvrol_status = "In progress: sent 2E 92 81 01";
      transmit_can_frame(&CMFA_WRITE_9281);
      break;
    case UDS_NVROL_WRITE_9281:
      logging.printf("CMFA: NVROL UDS sequence complete -> sleeping BMS for %lus\n", NVROL_SLEEP_DURATION_MS / 1000);
      uds_state = UDS_IDLE;
      start_sleep("NVROL");
      break;
    default:
      uds_state = UDS_IDLE;
      break;
  }
}

void CmfaEvBattery::on_uds_sequence_timeout() {
  abort_sequence("Timed out");
}

void CmfaEvBattery::sequence_step_rejected(const char* tag) {
  if (uds_state == UDS_NVROL_WRITE_9281 || uds_state == UDS_TEMP_WRITE_9281) {
    logging.printf("CMFA: Ignoring %s on 0x9281 (not supported on this BMS)\n", tag);
    if (uds_state == UDS_NVROL_WRITE_9281) {
      start_sleep("NVROL");
    }
    uds_state = UDS_IDLE;
    return;
  }
  abort_sequence(tag);
}

void CmfaEvBattery::abort_sequence(const char* reason) {
  bool was_nvrol = (uds_state >= UDS_NVROL_EXT_SESSION_1 && uds_state <= UDS_NVROL_WRITE_9281);
  logging.printf("CMFA: UDS sequence aborted (%s) at state=%u\n", reason, (unsigned)uds_state);
  if (was_nvrol) {
    last_nvrol_status = String(reason) + " at step " + String((int)uds_state);
  }
  uds_state = UDS_IDLE;
}

void CmfaEvBattery::begin_sleep(const char* label, unsigned long silent_duration_ms) {
  sleep_phase = SLEEP_STOPPING;
  sleep_phase_started_ms = millis();
  sleep_low_current_since_ms = 0;
  sleep_silent_duration_ms = silent_duration_ms;
  sleep_requested = true;
  bms_seen_silent_in_sleep = false;
  last_nvrol_status = String(label) + ": waiting for current <= 1A";
  logging.printf("CMFA: %s requested -> waiting for current to drop (I=%ddA)\n", label, (int)pack_current_dA());
}

void CmfaEvBattery::end_sleep() {
  if (sleep_phase == SLEEP_AWAKE) {
    return;
  }
  logging.printf("CMFA: Wake requested by user -> resuming CAN\n");
  finish_sleep();
  last_nvrol_status = "Woken by user at " + String(millis() / 1000) + "s";
}

void CmfaEvBattery::continue_sleep(unsigned long now) {
  bool bms_quiet = (now - last_rx_ms) >= BMS_SILENT_THRESHOLD_MS;
  if (bms_quiet && !bms_seen_silent_in_sleep) {
    bms_seen_silent_in_sleep = true;
    logging.printf("CMFA: BMS went silent %lus into sleep\n", (now - sleep_phase_started_ms) / 1000);
    last_nvrol_status = "BMS silent at +" + String((now - sleep_phase_started_ms) / 1000) + "s";
  }
}

void CmfaEvBattery::finish_sleep() {
  sleep_phase = SLEEP_AWAKE;
  sleep_requested = false;
  previousMillis10 = 0;
  previousMillis100 = 0;
  previousMillis200 = 0;
  previousMillis1000 = 0;
  uds_wait_done = true;
  const char* bms_note = bms_seen_silent_in_sleep ? " (BMS went silent)" : " (BMS NEVER went silent)";
  last_nvrol_status = "Completed at " + String(millis() / 1000) + "s" + bms_note;
  logging.printf("CMFA: BMS sleep window finished%s, resuming CAN\n", bms_note);
}

void CmfaEvBattery::transmit_can(unsigned long currentMillis) {
  if (sleep_phase == SLEEP_STOPPING) {
    int16_t cur_dA = pack_current_dA();
    if (std::abs(cur_dA) <= CMFA_SLEEP_MAX_CURRENT_DA) {
      if (sleep_low_current_since_ms == 0) {
        sleep_low_current_since_ms = currentMillis;
      } else if ((currentMillis - sleep_low_current_since_ms) >= CMFA_SLEEP_STOP_SETTLE_MS) {
        sleep_phase = SLEEP_SHUTDOWN;
        sleep_phase_started_ms = currentMillis;
        last_nvrol_status = "Shutting down (0.9s 0x44 sequence)";
        logging.printf("CMFA: Current settled (I=%ddA) -> sending 0.9s shutdown sequence\n", (int)cur_dA);
      }
    } else {
      sleep_low_current_since_ms = 0;
      if ((currentMillis - sleep_phase_started_ms) >= CMFA_SLEEP_MAX_WAIT_MS) {
        sleep_phase = SLEEP_AWAKE;
        sleep_requested = false;
        last_nvrol_status = "Aborted: current still " + String(cur_dA) + "dA after 30s";
        logging.printf("CMFA: Sleep aborted, current still %ddA after 30s\n", (int)cur_dA);
      }
    }
  }

  if (sleep_phase == SLEEP_SHUTDOWN) {
    if ((currentMillis - sleep_phase_started_ms) >= CMFA_SLEEP_SHUTDOWN_MS) {
      sleep_phase = SLEEP_SILENT;
      sleep_phase_started_ms = currentMillis;
      if (sleep_silent_duration_ms == 0) {
        last_nvrol_status = "Asleep (press Wake to resume)";
        logging.printf("CMFA: Shutdown sequence complete -> bus silent until Wake\n");
      } else {
        last_nvrol_status = "Sleeping (30s silent)";
        logging.printf("CMFA: Shutdown sequence complete -> bus silent for %lus\n", sleep_silent_duration_ms / 1000);
      }
      return;
    }
    if ((currentMillis - previousMillis10) >= INTERVAL_10_MS) {
      previousMillis10 = currentMillis;
      CMFA_135.data.u8[1] = ((CMFA_135.data.u8[1] + 0x50) & 0xF0) | 0x05;
      transmit_can_frame(&CMFA_135);
    }
    if ((currentMillis - previousMillis100) >= INTERVAL_100_MS) {
      previousMillis100 = currentMillis;
      shutdown_3D3_toggle = !shutdown_3D3_toggle;
      CMFA_3D3_SHUTDOWN.data.u8[4] = shutdown_3D3_toggle ? 0xB2 : 0x5D;
      CMFA_3D3_SHUTDOWN.data.u8[6] = shutdown_3D3_toggle ? 0x5D : 0xB2;
      transmit_can_frame(&CMFA_3D3_SHUTDOWN);
      update_vehicle_time_from_pack();
      transmit_can_frame(&CMFA_3D4);
      transmit_can_frame(&CMFA_3D5_SHUTDOWN);
    }
    return;
  }

  if (sleep_phase == SLEEP_SILENT) {
    if (sleep_silent_duration_ms > 0 && (currentMillis - sleep_phase_started_ms) >= sleep_silent_duration_ms) {
      finish_sleep();
    } else {
      continue_sleep(currentMillis);
      return;
    }
  }

  if ((currentMillis - previousMillis10) >= INTERVAL_10_MS) {
    previousMillis10 = currentMillis;
    CMFA_135.data.u8[1] = ((CMFA_135.data.u8[1] + 0x50) & 0xF0) | 0x05;
    CMFA_125.data.u8[3] = (CMFA_125.data.u8[3] + 5) & 0x0F;
    transmit_can_frame(&CMFA_1EA);
    transmit_can_frame(&CMFA_135);
    transmit_can_frame(&CMFA_134);
    transmit_can_frame(&CMFA_125);
  }

  if ((currentMillis - previousMillis100) >= INTERVAL_100_MS) {
    previousMillis100 = currentMillis;
    update_car_state_frames();
    update_vehicle_time_from_pack();

    transmit_can_frame(&CMFA_59B);
    transmit_can_frame(&CMFA_3D3);
    transmit_can_frame(&CMFA_3D4);
    transmit_can_frame(&CMFA_3D5);
    transmit_can_frame(&CMFA_42B);
    transmit_can_frame(&CMFA_431);
    transmit_can_frame(&CMFA_442);
    transmit_can_frame(&CMFA_443);
    transmit_can_frame(&CMFA_511);

    static uint8_t tick_1000ms = 0;
    if (++tick_1000ms >= 10) {
      tick_1000ms = 0;
      transmit_can_frame(&CMFA_69F);
    }

    if (datalayer.battery.status.bms_status != FAULT) {
      datalayer.system.status.battery_allows_contactor_closing = true;
    } else {
      datalayer.system.status.battery_allows_contactor_closing = false;
    }
  }

  if (uds_state != UDS_IDLE && (currentMillis - uds_step_started_ms) >= UDS_STEP_TIMEOUT_MS) {
    if (uds_state == UDS_DTC_READ || uds_state == UDS_DTC_CLEAR) {
      uds_state = UDS_IDLE;
    } else {
      on_uds_sequence_timeout();
    }
  }

  if (uds_state == UDS_IDLE) {
    if (uds_dtc_clear_CheckRequest()) {
      CAN_frame req{};
      if (uds_build_ClearDiagnosticInformation_tx(&req, CMFA_UDS_REQ_ID)) {
        uds_state = UDS_DTC_CLEAR;
        uds_step_started_ms = currentMillis;
        transmit_can_frame(&req);
      }
    } else if (uds_dtc_request_CheckRequest()) {
      CAN_frame req{};
      if (uds_build_ReadDTCByStatusMask_tx(&req, CMFA_UDS_REQ_ID, 0xFF)) {
        uds_state = UDS_DTC_READ;
        uds_step_started_ms = currentMillis;
        transmit_can_frame(&req);
      }
    }
  }

  if (uds_state == UDS_IDLE &&
      (uds_wait_done || (currentMillis - previousMillis200) >= INTERVAL_200_MS) &&
      (currentMillis - previousMillis200) >= INTERVAL_100_MS) {
    previousMillis200 = currentMillis;
    uds_wait_done = false;

    int pid = 0;
    if (fast_slot_counter < FAST_POLL_COUNT) {
      pid = fast_poll_pids[fast_poll_index];
      fast_poll_index = (fast_poll_index + 1) % FAST_POLL_COUNT;
      fast_slot_counter++;
    } else {
      fast_slot_counter = 0;
      if (slow_poll_pids.size() > 0) {
        pid = slow_poll_pids.get(slow_poll_cursor);
        slow_poll_cursor = (slow_poll_cursor + 1) % slow_poll_pids.size();
      }
    }
    if (pid != 0) {
      current_poll_pid = (uint16_t)pid;
      CAN_frame req{};
      if (uds_build_ReadDataByIdentifier_tx(&req, CMFA_UDS_REQ_ID, current_poll_pid)) {
        req.data.u8[4] = 0x55;
        req.data.u8[5] = 0x55;
        req.data.u8[6] = 0x55;
        req.data.u8[7] = 0x55;
        uds_fast_start_request(CMFA_UDS_RESP_ID, 0x22);
        transmit_can_frame(&req);
      }
    }
  }
}

void CmfaEvBattery::setup(void) {
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.battery.info.number_of_cells = NUMBER_OF_CELLS;
  datalayer.battery.info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer.battery.info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
  datalayer.battery.info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer.battery.info.total_capacity_Wh = 26800;
  datalayer.battery.info.chemistry = battery_chemistry_enum::NMC;

  const int initial_slow_pids[] = {
      PID_POLL_PACK_TIME_LIFE,
      PID_POLL_BMS_STATE,
      PID_POLL_REQUEST_FROM_CAN,
      PID_POLL_SOC_Z,
      PID_POLL_SOC_U,
      PID_POLL_SOH_AVERAGE,
      PID_POLL_PACK_VOLTAGE,
      PID_POLL_AVERAGE_VOLTAGE_OF_CELLS,
      PID_POLL_MAX_CELL_VOLTAGE,
      PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE,
      PID_POLL_MIN_CELL_VOLTAGE,
      PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE,
      PID_POLL_MAXIMUM_GENERATED_POWER,
      PID_POLL_LV_BAT,
      PID_POLL_AVERAGE_TEMPERATURE,
      PID_POLL_MIN_TEMPERATURE,
      PID_POLL_MAX_TEMPERATURE,
      PID_POLL_END_OF_CHARGE_FLAG,
      PID_POLL_HVBATT_INTERLOCK_FLAG,
      PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING,
      PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING,
      PID_POLL_CELL_1,  PID_POLL_CELL_2,  PID_POLL_CELL_3,  PID_POLL_CELL_4,  PID_POLL_CELL_5,  PID_POLL_CELL_6,
      PID_POLL_CELL_7,  PID_POLL_CELL_8,  PID_POLL_CELL_9,  PID_POLL_CELL_10, PID_POLL_CELL_11, PID_POLL_CELL_12,
      PID_POLL_CELL_13, PID_POLL_CELL_14, PID_POLL_CELL_15, PID_POLL_CELL_16, PID_POLL_CELL_17, PID_POLL_CELL_18,
      PID_POLL_CELL_19, PID_POLL_CELL_20, PID_POLL_CELL_21, PID_POLL_CELL_22, PID_POLL_CELL_23, PID_POLL_CELL_24,
      PID_POLL_CELL_25, PID_POLL_CELL_26, PID_POLL_CELL_27, PID_POLL_CELL_28, PID_POLL_CELL_29, PID_POLL_CELL_30,
      PID_POLL_CELL_31, PID_POLL_CELL_32, PID_POLL_CELL_33, PID_POLL_CELL_34, PID_POLL_CELL_35, PID_POLL_CELL_36,
      PID_POLL_CELL_37, PID_POLL_CELL_38, PID_POLL_CELL_39, PID_POLL_CELL_40, PID_POLL_CELL_41, PID_POLL_CELL_42,
      PID_POLL_CELL_43, PID_POLL_CELL_44, PID_POLL_CELL_45, PID_POLL_CELL_46, PID_POLL_CELL_47, PID_POLL_CELL_48,
      PID_POLL_CELL_49, PID_POLL_CELL_50, PID_POLL_CELL_51, PID_POLL_CELL_52, PID_POLL_CELL_53, PID_POLL_CELL_54,
      PID_POLL_CELL_55, PID_POLL_CELL_56, PID_POLL_CELL_57, PID_POLL_CELL_58, PID_POLL_CELL_59, PID_POLL_CELL_60,
      PID_POLL_CELL_61, PID_POLL_CELL_62, PID_POLL_CELL_63, PID_POLL_CELL_64, PID_POLL_CELL_65, PID_POLL_CELL_66,
      PID_POLL_CELL_67, PID_POLL_CELL_68, PID_POLL_CELL_69, PID_POLL_CELL_70, PID_POLL_CELL_71, PID_POLL_CELL_72,
      PID_POLL_MISSION_FIRST + 0,
      PID_POLL_MISSION_FIRST + 1,
      PID_POLL_MISSION_FIRST + 2,
      PID_POLL_MISSION_FIRST + 3,
  };
  slow_poll_pids.clear();
  for (size_t i = 0; i < sizeof(initial_slow_pids) / sizeof(initial_slow_pids[0]); i++) {
    slow_poll_pids.add(initial_slow_pids[i]);
  }
  update_vehicle_time_from_pack();
}