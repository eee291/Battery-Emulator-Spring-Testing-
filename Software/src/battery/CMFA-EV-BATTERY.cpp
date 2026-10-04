#include "CMFA-EV-BATTERY.h"
#include <algorithm>
#include <cstring>  //unit tests memcpy
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"
#include "BATTERIES.h"

/* The raw SOC (0x127, 0.01%/4) is rescaled so that 100% is reported at MAXSOC (90% raw) and 0% at MINSOC (5% raw),
keeping a margin at both ends. A real car charges further: its dash shows 100% at about 97.5% raw, and the pack keeps
charging to 100.00% raw before ending the charge itself. While emulating charging we use MAXSOC_CHARGING instead. */

uint16_t CmfaEvBattery::rescale_raw_SOC(uint32_t raw_SOC, uint32_t max_soc) {

  uint32_t calc_soc = raw_SOC / 4;
  if (calc_soc > max_soc) {  //Constrain if needed
    calc_soc = max_soc;
  }
  if (calc_soc < MINSOC) {  //Constrain if needed
    calc_soc = MINSOC;
  }
  // Perform scaling between the two points
  calc_soc = 10000 * (calc_soc - MINSOC);
  calc_soc = calc_soc / (max_soc - MINSOC);

  return (uint16_t)calc_soc;
}

void CmfaEvBattery::
    update_values() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus
  datalayer_battery->status.soh_pptt = (SOH * 100);

  const bool charging_mode = (car_mode == CAR_MODE_CHARGING);
  datalayer_battery->status.real_soc = rescale_raw_SOC(SOC_raw, charging_mode ? MAXSOC_CHARGING : MAXSOC);

  datalayer_battery->status.current_dA = (((int32_t)current_raw * 10) / 4) - 5000;

  datalayer_battery->status.voltage_dV = pack_voltage * 5;

  datalayer_battery->info.total_capacity_Wh = 27000;

  //Calculate the remaining Wh amount from SOC% and max Wh value.
  datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);

  if (user_selected_use_estimated_charge_limits) {  //Some packs are locked? and do not report allowed charge/discharge power
    datalayer_battery->status.max_charge_power_W = datalayer.battery.status.override_charge_power_W;

    datalayer_battery->status.max_discharge_power_W = datalayer.battery.status.override_discharge_power_W;
  } else if (use_broadcast_power_limits) {  //Use the limits in the 0x3D6 broadcast
    // 0x3D6 looks like the DC fast charge limit: it tapers to 0 just after
    // 100%, while a real car's AC charger carries on, following the pack's own
    // charge power request in 0x127 until the pack ends the charge. Do the
    // same while emulating charging, so the pack can finish its charge.
    datalayer_battery->status.max_charge_power_W = charging_mode ? charge_power_request_w : charge_power_w;

    datalayer_battery->status.max_discharge_power_W = discharge_power_w;
  } else {  //Use the limits read via UDS
    datalayer_battery->status.max_charge_power_W = maximum_charge_power;

    datalayer_battery->status.max_discharge_power_W = max_discharge_power;
  }

  if (manual_sleep_requested || sleep_phase != SLEEP_AWAKE) {
    // Asleep (or about to be): the pack has opened its contactors.
    datalayer_battery->status.max_charge_power_W = 0;
    datalayer_battery->status.max_discharge_power_W = 0;
  }

  datalayer_battery->status.temperature_min_dC = (lowest_cell_temperature * 10);

  datalayer_battery->status.temperature_max_dC = (highest_cell_temperature * 10);

  datalayer_battery->status.cell_min_voltage_mV = lowest_cell_voltage_mv;

  datalayer_battery->status.cell_max_voltage_mV = highest_cell_voltage_mv;

  if (lead_acid_voltage < 11000) {  //11.000V
    set_event(EVENT_12V_LOW, lead_acid_voltage, battery_index);
  }
}

template <typename T>
inline String& operator<<(String& str, const T& value) {
  str += value;
  return str;
}

// Balancing stats are stored in 1/1024 Ah or 1/1024 h units.
// Treat the INT32_MIN sentinel as zero.
static int32_t bal_mAh(int32_t v) {
  return (v == INT32_MIN) ? 0 : (v * 125) / 128;
}
static int32_t bal_s(int32_t v) {
  return (v == INT32_MIN) ? 0 : (v * 225) / 64;
}

String CmfaEvBattery::get_uds_info_html() {
  String content;
  content.reserve(900);

  // clang-format off
  content << "<h4>SOC U: " << soc_u << "percent</h4>"
             "<h4>SOC Z: " << soc_z << "percent</h4>"
             "<h4>SOH Average: " << soh_average << "pptt</h4>"
             "<h4>Instant current: " << (((int32_t)instant_current_raw - 48000) * 25) << "mA</h4>"
             "<h4>Average current: " << ((((int32_t)instant_current_raw - 32640) * 125) / 4) << "mA</h4>"
             "<h4>12V voltage: " << lead_acid_voltage << "mV</h4>"
             "<h4>Highest cell number: " << highest_cell_voltage_number << "</h4>"
             "<h4>Lowest cell number: " << lowest_cell_voltage_number << "</h4>"
             "<h4>Sum of cellvoltages: " << average_voltage_of_cells << "</h4>"
             "<h4>Max regen power: " << max_regen_power << "</h4>"
             "<h4>Max discharge power: " << max_discharge_power << "</h4>"
             "<h4>Max charge power: " << maximum_charge_power << "</h4>"
             "<h4>Power limit source: " << (
                user_selected_use_estimated_charge_limits ? "ESTIMATED (user setting)" :
                use_broadcast_power_limits ? (car_mode == CAR_MODE_CHARGING ? "BROADCAST (charge: 0x127 request, discharge: 0x3D6)" : "BROADCAST (0x3D6)") : "UDS (DIDs)"
             ) << "</h4>"
             "<h4>Broadcast charge power limit (0x3D6): " << charge_power_w << "W</h4>"
             "<h4>Broadcast discharge power limit (0x3D6): " << discharge_power_w << "W</h4>"
             "<h4>Broadcast charge power request (0x127): " << charge_power_request_w << "W</h4>"
             "<h4>Car state sent: " << (car_mode == CAR_MODE_CHARGING ? "CHARGING (as a real car)" : "DEFAULT (emulator)") << "</h4>"
             "<h4>Pack mode (0x3D7 byte 0): " << (
                status_3d7==0x0A ? "CHARGING" :
                status_3d7==0x24 ? "IDLE" :
                status_3d7==0x2A ? "READY" : "UNKNOWN"
             ) << " (0x" << String(status_3d7, HEX) << ")</h4>"
             "<h4>Pack idle flag (0x43C byte 0 bit 6): " << (
                status_43c==0xFF ? "UNKNOWN" :
                (status_43c & 0x40) ? "SET" : "CLEAR"
             ) << " (0x" << String(status_43c, HEX) << ")</h4>"
             "<h4>BMS mode (DID 0x9259): " << bms_state << "</h4>"
             "<h4>End of charge flag (DID 0x9019): " << (end_of_charge ? "SET" : "CLEAR") << "</h4>"
             "<h4>SOH available power: " << SOH_available_power << "</h4>"
             "<h4>SOH generated power: " << SOH_generated_power << "</h4>"
             "<h4>Average temperature: " << average_temperature << "dC</h4>"
             "<h4>Maximum temperature: " << maximum_temperature << "dC</h4>"
             "<h4>Minimum temperature: " << minimum_temperature << "dC</h4>"
             "<h4>Cumulative energy discharged: " << cumulative_energy_when_discharging << "Wh</h4>"
             "<h4>Cumulative energy charged: " << cumulative_energy_when_charging << "Wh</h4>"
             "<h4>Cumulative energy regen: " << cumulative_energy_in_regen << "Wh</h4>"
             "<h4>Balance capacity total: " << bal_mAh(balance_capacity_total) << "mAh (was " << bal_mAh(initial_balance_capacity_total) << "mAh)</h4>"
             "<h4>Balance time total: " << bal_s(balance_time_total) << "s (was " << bal_s(initial_balance_time_total) << "s)</h4>"
             "<h4>Balance capacity sleep: " << bal_mAh(balance_capacity_sleep) << "mAh (was " << bal_mAh(initial_balance_capacity_sleep) << "mAh)</h4>"
             "<h4>Balance time sleep: " << bal_s(balance_time_sleep) << "s (was " << bal_s(initial_balance_time_sleep) << "s)</h4>"
             "<h4>Balance capacity wake: " << bal_mAh(balance_capacity_wake) << "mAh (was " << bal_mAh(initial_balance_capacity_wake) << "mAh)</h4>"
             "<h4>Balance time wake: " << bal_s(balance_time_wake) << "s (was " << bal_s(initial_balance_time_wake) << "s)</h4>"
             "<h4>Temporisation: " << (
                temporisation==0xFF ? "UNKNOWN" : 
                temporisation==1 ? "ACTIVE" : "INACTIVE"
             ) << "</h4>"
             "<h4>Pack time life: " << pack_time_life << "m (was " << int32_t(initial_pack_time_life) << "m)</h4>"
             "<h4>Absolute time saved: " << absolute_time_saved << "m (was " << int32_t(initial_absolute_time_saved) << "m)</h4>"
             "<h4>Vehicle time sent (0x3D4): ";
  if (vehicle_time_valid) {
    // Real minutes elapsed since we started sending, vs how far the vehicle
    // time has moved. Any excess is a forward jump.
    const uint32_t real_min = (millis() - vehicle_time_start_ms) / 60000;
    const uint32_t advanced = vehicle_time_min - vehicle_time_start;
    content << vehicle_time_min << "m</h4>"
            << "<h4>Vehicle time started at: " << vehicle_time_start << "m, advanced " << advanced << "m in " << real_min
            << "m real time, " << vehicle_time_jumps << " forward jumps (" << vehicle_time_jumped_min << "m)"
            << (advanced > real_min + 1 ? " - AHEAD OF REAL TIME" : "") << "</h4>"
            // If an offset stays constant, that clock follows our 0x3D4. If it
            // falls by 1 per minute, that clock isn't moving.
            << "<h4>Pack time life - vehicle time: " << (int32_t)(pack_time_life - vehicle_time_min) << "m (was "
            << initial_offset_pack_time_life << "m)</h4>"
            << "<h4>Absolute time saved - vehicle time: " << (int32_t)(absolute_time_saved - vehicle_time_min)
            << "m (was " << initial_offset_time_saved << "m)</h4>";
  } else {
    content << "not yet (waiting for pack times)</h4>";
  }
  content << "<h4>Sleep: ";
  if (sleep_phase == SLEEP_AWAKE) {
    content << "AWAKE";
  } else {
    content << (manual_sleep ? "ASLEEP (Sleep button)" : "ASLEEP (sleep sequence)") << " for "
            << (millis() - sleep_request_ms) / 60000 << "m, ";
    if (bms_quiet) {
      content << "BMS went quiet " << bms_quiet_after_ms << "ms after the request";
    } else {
      content << "BMS STILL TRANSMITTING";
    }
    if (bms_wakes) {
      content << ", BMS woke up " << bms_wakes << " times";
    }
  }
  content << "</h4>";
  content << "<h4>Last NVROL/sleep sequence: " << sequence_status << "</h4>";
  append_mission_html(content);
  // clang-format on

  return content;
}

void CmfaEvBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  // UDS frames (0x7BB PID/DTC replies) are handled by the superclass.
  if (handle_incoming_uds_can_frame(rx_frame)) {
    return;
  }

  bool from_bms = true;
  switch (rx_frame.ID) {  //These frames are transmitted by the battery
    case 0x127:           //10ms , Same structure as old Zoe 0x155 message!
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      charge_power_request_w = rx_frame.data.u8[0] * 300;
      current_raw = ((rx_frame.data.u8[1] & 0x0F) << 8) | rx_frame.data.u8[2];
      SOC_raw = ((rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5]);
      pack_voltage = (((rx_frame.data.u8[6] & 0x03) << 8) | rx_frame.data.u8[7]);
      break;
    case 0x3D6:  //100ms, Same structure as old Zoe 0x424 message!
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      charge_power_w = rx_frame.data.u8[2] * 500;
      discharge_power_w = rx_frame.data.u8[3] * 500;
      lowest_cell_temperature = (rx_frame.data.u8[4] - 40);
      SOH = rx_frame.data.u8[5];
      heartbeat = rx_frame.data.u8[6];
      highest_cell_temperature = (rx_frame.data.u8[7] - 40);
      break;
    case 0x3D7:  //100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      status_3d7 = rx_frame.data.u8[0];
      break;
    case 0x3D8:  //100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //counter_3D8 = rx_frame.data.u8[3]; //?
      //CRC_3D8 = rx_frame.data.u8[4]; //?
      break;
    case 0x43C:  //100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      status_43c = rx_frame.data.u8[0];
      heartbeat2 = rx_frame.data.u8[2];  //Alternates between 0x55 and 0xAA every 5th frame
      break;
    case 0x431:  //100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //byte0 9C always
      //byte1 40 always
      break;
    case 0x5A9:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5AB:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5C8:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5E1:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    default:
      from_bms = false;
      break;
  }

  if (from_bms) {
    last_bms_frame_ms = millis();
    if (sleep_phase != SLEEP_AWAKE && bms_quiet) {
      // The BMS has started talking again while it should be asleep.
      bms_quiet = false;
      bms_wakes = bms_wakes + 1;
      logging.printf("[CMFA] BMS started transmitting again during sleep (0x%03X)\n", (unsigned)rx_frame.ID);
    }
  }
}

// The balancing data has a offset that needs stripping
static int32_t decode_balance_word(uint32_t raw) {
  return (int32_t)(raw - 0x80000000u);
}

uint16_t CmfaEvBattery::handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) {
  // Called by the UDS superclass for every successful PID response. `value` is
  // the big-endian PID value (up to 4 bytes), `data` points at the raw value
  // bytes (without the SID/DID header). Return 0 to continue the scan list.
  switch (pid) {
    case PID_POLL_SOCZ:
      soc_z = (uint16_t)value;
      break;
    case PID_POLL_USOC:
      soc_u = (uint16_t)value;
      break;
    case PID_POLL_SOH_AVERAGE:
      soh_average = (uint16_t)value;
      break;
    case PID_POLL_AVERAGE_VOLTAGE_OF_CELLS:
      average_voltage_of_cells = value;
      break;
    case PID_POLL_HIGHEST_CELL_VOLTAGE:
      highest_cell_voltage_mv = (uint16_t)(value * 0.976563f);
      break;
    case PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE:
      highest_cell_voltage_number = data[0];
      break;
    case PID_POLL_LOWEST_CELL_VOLTAGE:
      lowest_cell_voltage_mv = (uint16_t)(value * 0.976563f);
      break;
    case PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE:
      lowest_cell_voltage_number = data[0];
      break;
    case PID_POLL_CURRENT_OFFSET:
      // Not used
      break;
    case PID_POLL_INSTANT_CURRENT:
      instant_current_raw = value;
      break;
    case PID_POLL_100MS_CURRENT:
      averaged_current_raw = value;
      break;
    case PID_POLL_MAX_REGEN:
      max_regen_power = value * 10;
      break;
    case PID_POLL_MAX_DISCHARGE_POWER:
      max_discharge_power = value * 10;
      break;
    case PID_POLL_12V_BATTERY:
      lead_acid_voltage = (uint16_t)value;
      break;
    case PID_POLL_AVERAGE_TEMPERATURE:
      average_temperature = (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MIN_TEMPERATURE:
      minimum_temperature = (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MAX_TEMPERATURE:
      maximum_temperature = (int16_t)(((int32_t)value - 400) / 2);
      break;
    case PID_POLL_MAX_CHARGE_POWER:
      maximum_charge_power = value * 10;
      break;
    case PID_POLL_END_OF_CHARGE_FLAG:
      end_of_charge = data[0];
      break;
    case PID_POLL_INTERLOCK_FLAG:
      interlock_flag = data[0];
      break;
    case PID_POLL_SOH_AVAILABLE_POWER_CALCULATION:
      SOH_available_power = (uint16_t)value;
      break;
    case PID_POLL_SOH_GENERATED_POWER_CALCULATION:
      SOH_generated_power = (uint16_t)value;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING:
      cumulative_energy_when_discharging = value;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING:
      cumulative_energy_when_charging = value;
      break;
    case PID_POLL_CUMULATIVE_ENERGY_IN_REGEN:
      cumulative_energy_in_regen = value;
      break;
    case PID_POLL_BALANCE_CAPACITY_TOTAL:
      balance_capacity_total = decode_balance_word(value);
      if (initial_balance_capacity_total == INT32_MIN)
        initial_balance_capacity_total = balance_capacity_total;
      break;
    case PID_POLL_BALANCE_TIME_TOTAL:
      balance_time_total = decode_balance_word(value);
      if (initial_balance_time_total == INT32_MIN)
        initial_balance_time_total = balance_time_total;
      break;
    case PID_POLL_BALANCE_CAPACITY_SLEEP:
      balance_capacity_sleep = decode_balance_word(value);
      if (initial_balance_capacity_sleep == INT32_MIN)
        initial_balance_capacity_sleep = balance_capacity_sleep;
      break;
    case PID_POLL_BALANCE_TIME_SLEEP:
      balance_time_sleep = decode_balance_word(value);
      if (initial_balance_time_sleep == INT32_MIN)
        initial_balance_time_sleep = balance_time_sleep;
      break;
    case PID_POLL_BALANCE_CAPACITY_WAKE:
      balance_capacity_wake = decode_balance_word(value);
      if (initial_balance_capacity_wake == INT32_MIN)
        initial_balance_capacity_wake = balance_capacity_wake;
      break;
    case PID_POLL_BALANCE_TIME_WAKE:
      balance_time_wake = decode_balance_word(value);
      if (initial_balance_time_wake == INT32_MIN)
        initial_balance_time_wake = balance_time_wake;
      break;
    case PID_POLL_BMS_STATE:
      bms_state = (uint8_t)value;
      break;
    case PID_POLL_TEMPORISATION:
      temporisation = value;
      break;
    case PID_POLL_PACK_TIME_LIFE:
      log_pack_time_change("pack time life (0x91C1)", pack_time_life, value);
      pack_time_life = value;
      if (initial_pack_time_life == UINT32_MAX)
        initial_pack_time_life = pack_time_life;
      update_vehicle_time_from_pack();
      break;
    case PID_POLL_ABSOLUTE_TIME_SAVED:
      log_pack_time_change("absolute time saved (0x9261)", absolute_time_saved, value);
      absolute_time_saved = value;
      if (initial_absolute_time_saved == UINT32_MAX)
        initial_absolute_time_saved = absolute_time_saved;
      update_vehicle_time_from_pack();
      break;
    case PID_POLL_MISSION_MODE:
    case PID_POLL_MISSION_TEMP_BEG:
    case PID_POLL_MISSION_TEMP_END:
    case PID_POLL_MISSION_USOC_BEG:
    case PID_POLL_MISSION_USOC_END:
    case PID_POLL_MISSION_TIME_BEG:
    case PID_POLL_MISSION_TIME_END:
    case PID_POLL_MISSION_DIST_END:
    case PID_POLL_MISSION_INDEX:
      handle_mission_pid(pid, data, length);
      break;
    case PID_POLL_BALANCE_SWITCHES: {
      // Assumed to use the same encoding as RENAULT-ZOE-2-BATTERY, with the
      // bitmap at the end of the payload, cell 1 in bit 0 of the last byte.
      // May not be correct!
      uint8_t cells = datalayer_battery->info.number_of_cells;
      if (length < (cells + 7) / 8) {
        break;
      }
      for (uint8_t i = 0; i < cells; i++) {
        datalayer_battery->status.cell_balancing_status[i] = (data[length - 1 - (i >> 3)] >> (i & 7)) & 0x01;
      }
      break;
    }
    default:  //Unknown pid, or a cellvoltage
      uint8_t cellnumber = 0;
      if (pid >= PID_POLL_CELL_1 && pid <= PID_POLL_CELL_31) {  //Cellvoltage PID reply
        cellnumber = (pid - PID_POLL_CELL_1);
      } else if (pid >= PID_POLL_CELL_32 && pid <= PID_POLL_CELL_62) {
        cellnumber = (pid - PID_POLL_CELL_1) - 1;
      } else if (pid >= PID_POLL_CELL_63 && pid <= PID_POLL_CELL_72) {
        cellnumber = (pid - PID_POLL_CELL_1) - 2;
      } else {  //Unknown pid
        break;
      }

      if (cellnumber < MAX_AMOUNT_CELLS) {
        uint16_t cellvoltage_reading = (uint16_t)value;
        if (cellvoltage_reading == 0) {
          //Blown fuse/celltap. Force value to 10mV so user sees this in cellmonitor page. Also fire event
          cellvoltage_reading = 10;
          set_event(EVENT_BATTERY_FUSE, cellnumber, battery_index);
        }
        datalayer_battery->status.cell_voltages_mV[cellnumber] = (uint16_t)(cellvoltage_reading * 0.976563f);
      }

      break;
  }

  // Do current and power much more often than the regular scans
  if (pid == PID_POLL_INSTANT_CURRENT) {
    return PID_POLL_MAX_CHARGE_POWER;
  } else if (pid == PID_POLL_MAX_CHARGE_POWER) {
    return PID_POLL_MAX_DISCHARGE_POWER;
  } else if (pid == PID_POLL_MAX_DISCHARGE_POWER) {
    // Scan the next regular PID in the sequence
    return 0;
  } else {
    // Current follows every regular PID
    return PID_POLL_INSTANT_CURRENT;
  }
}

void CmfaEvBattery::update_vehicle_time_from_pack() {
  // Wait until both of the pack's own times have been read at least once.
  if (initial_pack_time_life == UINT32_MAX || initial_absolute_time_saved == UINT32_MAX) {
    return;
  }

  if (!vehicle_time_valid) {
    // Start just above anything the pack has seen, so the time never goes backwards.
    const uint32_t pack_time = std::max(pack_time_life, absolute_time_saved);
    vehicle_time_min = std::min(pack_time + 1, VEHICLE_TIME_MAX);
    vehicle_time_last_tick_ms = millis();
    vehicle_time_valid = true;
    vehicle_time_start = vehicle_time_min;
    vehicle_time_start_ms = vehicle_time_last_tick_ms;
    initial_offset_pack_time_life = (int32_t)(pack_time_life - vehicle_time_min);
    initial_offset_time_saved = (int32_t)(absolute_time_saved - vehicle_time_min);
    logging.printf("[CMFA] Vehicle time starting at %lu min (pack time life %lu, absolute time saved %lu)\n",
                   (unsigned long)vehicle_time_min, (unsigned long)pack_time_life, (unsigned long)absolute_time_saved);
  } else if (absolute_time_saved > vehicle_time_min) {
    // The pack's saved vehicle time is ahead of us: catch up (forwards only).
    const uint32_t target = std::min(absolute_time_saved, VEHICLE_TIME_MAX);
    vehicle_time_jumps++;
    vehicle_time_jumped_min += target - vehicle_time_min;
    logging.printf("[CMFA] Absolute time saved %lu min is ahead of vehicle time %lu min, jumping forwards\n",
                   (unsigned long)absolute_time_saved, (unsigned long)vehicle_time_min);
    vehicle_time_min = target;
  }
}

void CmfaEvBattery::log_pack_time_change(const char* name, uint32_t old_value, uint32_t new_value) {
  // Log every change to the pack's clocks next to what we're sending, so a
  // log shows whether they follow our 0x3D4 (and whether anything runs ahead).
  if (vehicle_time_valid && old_value != new_value) {
    logging.printf("[CMFA] %s %lu -> %lu min (%+ld), vehicle time sent %lu min\n", name, (unsigned long)old_value,
                   (unsigned long)new_value, (long)(new_value - old_value), (unsigned long)vehicle_time_min);
  }
}

void CmfaEvBattery::handle_mission_pid(uint16_t pid, const uint8_t* data, uint16_t length) {
  // Each table holds up to 32 entries; some packs return fewer bytes, in
  // which case the missing entries are left as they were.
  auto bytes = [&](uint8_t* out) {
    for (uint16_t i = 0; i < length && i < MISSION_ENTRIES; i++)
      out[i] = data[i];
  };
  auto words = [&](uint32_t* out, uint8_t width) {
    for (uint16_t i = 0; (i + 1) * width <= length && i < MISSION_ENTRIES; i++) {
      uint32_t v = 0;
      for (uint8_t b = 0; b < width; b++)
        v = (v << 8) | data[i * width + b];
      out[i] = v;
    }
  };

  switch (pid) {
    case PID_POLL_MISSION_MODE:
      bytes(missions.mode);
      break;
    case PID_POLL_MISSION_TEMP_BEG:
      bytes(missions.temp_beg);
      break;
    case PID_POLL_MISSION_TEMP_END:
      bytes(missions.temp_end);
      break;
    case PID_POLL_MISSION_USOC_BEG:
      bytes(missions.usoc_beg);
      break;
    case PID_POLL_MISSION_USOC_END:
      bytes(missions.usoc_end);
      break;
    case PID_POLL_MISSION_TIME_BEG:
      words(missions.time_beg, 3);
      missions.read = true;
      break;
    case PID_POLL_MISSION_TIME_END:
      words(missions.time_end, 3);
      break;
    case PID_POLL_MISSION_DIST_END:
      words(missions.dist_end, 4);
      break;
    case PID_POLL_MISSION_INDEX:
      if (length >= 1)
        missions.index = data[0];
      break;
  }
}

static const char* mission_mode_name(uint8_t mode) {
  switch (mode) {
    case 1:
      return "Init";
    case 2:
      return "Wait";
    case 3:
      return "Isolated charge";
    case 4:
      return "Non isolated charge";
    case 5:
      return "External charge";
    case 6:
      return "Driving";
    case 7:
      return "Sleep transient";
    case 8:
      return "DC charge w/o RISOL";
    case 9:
      return "DC charge with RISOL";
    default:
      return "?";
  }
}

void CmfaEvBattery::append_mission_html(String& content) {
  if (!missions.read) {
    content << "<h4>Mission log: not read yet</h4>";
    return;
  }
  // Times are absolute pack time in minutes (same clock as DID 0x91C1), so
  // "ago" is relative to the current pack time life.
  content << "<h4>Mission log (index " << missions.index << ", times in pack minutes)</h4>"
          << "<table><tr><th>#</th><th>State</th><th>Start</th><th>End</th><th>Mins</th><th>Ago</th>"
             "<th>Temp &deg;C</th><th>USOC %</th><th>km</th></tr>";
  for (uint8_t i = 0; i < MISSION_ENTRIES; i++) {
    if (missions.time_beg[i] == 0 && missions.time_end[i] == 0 && missions.mode[i] == 0) {
      continue;  // Unused entry
    }
    const uint32_t end = missions.time_end[i];
    content << "<tr><td>" << i << "</td><td>" << missions.mode[i] << " " << mission_mode_name(missions.mode[i])
            << "</td><td>" << missions.time_beg[i] << "</td><td>" << end << "</td><td>"
            << (int32_t)(end - missions.time_beg[i]) << "</td><td>";
    if (pack_time_life >= end) {
      const uint32_t ago = pack_time_life - end;
      content << (ago / 1440) << "d " << ((ago / 60) % 24) << "h " << (ago % 60) << "m";
    } else {
      content << "-";
    }
    content << "</td><td>" << ((int)missions.temp_beg[i] - 40) << " &rarr; " << ((int)missions.temp_end[i] - 40)
            << "</td><td>" << missions.usoc_beg[i] << " &rarr; " << missions.usoc_end[i] << "</td><td>"
            << (int32_t)((missions.dist_end[i] - 0x80000000u) / 32) << "</td></tr>";
  }
  content << "</table>";
}

void CmfaEvBattery::update_car_state_frames() {
  if (car_mode == CAR_MODE_CHARGING) {
    // 0x3D3 as a real car sends while AC charging. Bytes 4 and 6 swap between
    // 0x5D and 0xB2 every three frames; byte 7 looks like the 12 V supply
    // voltage (0xD9 = ~13.6 V with the DC-DC running).
    const bool swap = (frames_3d3 / 3) & 1;
    frames_3d3 = (frames_3d3 + 1) % 6;
    CMFA_3D3.data.u8[0] = 0x47;
    CMFA_3D3.data.u8[1] = 0x1F;
    CMFA_3D3.data.u8[2] = 0x00;
    CMFA_3D3.data.u8[3] = 0x02;
    CMFA_3D3.data.u8[4] = swap ? 0xB2 : 0x5D;
    CMFA_3D3.data.u8[5] = 0x20;
    CMFA_3D3.data.u8[6] = swap ? 0x5D : 0xB2;
    CMFA_3D3.data.u8[7] = 0xD9;
    CMFA_3D4.data.u8[6] = 0x98;
    CMFA_3D4.data.u8[7] = 0x22;
  } else {
    // The emulator's original frames.
    static const uint8_t default_3d3[8] = {0x47, 0x30, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xE7};
    memcpy(CMFA_3D3.data.u8, default_3d3, sizeof(default_3d3));
    CMFA_3D4.data.u8[6] = 0x80;
    CMFA_3D4.data.u8[7] = 0x04;
  }
}

void CmfaEvBattery::enable_temporisation() {
  start_sequence(CMFA_STATE_TEMPORISATION_START);
}

void CmfaEvBattery::on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  // Called by the superclass when a response in a UDS sequence is received
  // (or a sequence/wait step starts, with sid 0). A negative response to any
  // request aborts the sequence; timeouts arrive via on_uds_sequence_timeout().
  if (sequence_step_rejected(state, sid, data, len)) {
    return;
  }

  switch (state) {
      // Temporisation sequence

    case CMFA_STATE_TEMPORISATION_START:
      // Enter an extended diagnostic session first.
      send_sequence_message(CMFA_STATE_TEMPORISATION_DIAG, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1,
                            CMFA_UDS_TIMEOUT_SESSION_CONTROL, 2);
      break;
    case CMFA_STATE_TEMPORISATION_DIAG:
      // Extended diagnostic session entered, write the temporisation value.
      send_sequence_message(CMFA_STATE_TEMPORISATION_SEND, SID::WriteDataByIdentifier, (const uint8_t*)"\x92\x81\x01",
                            3, CMFA_UDS_TIMEOUT_WRITE, 2);
      break;
    case CMFA_STATE_TEMPORISATION_SEND:
      logging.println("[CMFA] Temporisation enabled (DID 0x9281 set to 0x01)");
      break;

      // NVROL reset sequence

    case CMFA_STATE_NVROL_START:
      snprintf(sequence_status, sizeof(sequence_status), "NVROL reset in progress");
      logging.println("[CMFA] NVROL reset started");
      // NVROL reset, part 1: enter an extended diagnostic session (0x10 0x03).
      send_sequence_message(CMFA_STATE_NVROL_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1,
                            CMFA_NVROL_TIMEOUT_TICKS, 1);
      break;
    case CMFA_STATE_NVROL_SESSION:
      // NVROL reset, part 2: run the NVROL reset routine (0x31 01 B0 09).
      send_sequence_message(CMFA_STATE_NVROL_ROUTINE, SID::RoutineControl, (const uint8_t*)"\x01\xB0\x09", 3,
                            CMFA_NVROL_TIMEOUT_TICKS, 2);
      break;
    case CMFA_STATE_NVROL_ROUTINE:
      logging.println("[CMFA] NVROL reset routine accepted");
      // Enable temporisation before sleep, part 1: extended session again.
      send_sequence_message(CMFA_STATE_NVROL_SLEEP_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1,
                            CMFA_NVROL_TIMEOUT_TICKS, 1);
      break;
    case CMFA_STATE_NVROL_SLEEP_SESSION:
      // Enable temporisation before sleep, part 2: write DID 0x9281 = 0x01.
      send_sequence_message(CMFA_STATE_NVROL_SLEEP_WRITE, SID::WriteDataByIdentifier, (const uint8_t*)"\x92\x81\x01", 3,
                            CMFA_NVROL_TIMEOUT_TICKS, 1);
      break;
    case CMFA_STATE_NVROL_SLEEP_WRITE:
      // The temporisation write is done: stop streaming the awake byte on 0x1EA
      // so the pack saves its state and falls asleep, then hold the bus for 30 s.
      start_sleep(CMFA_STATE_NVROL_SLEEP_WAIT);
      break;
    case CMFA_STATE_NVROL_SLEEP_WAIT:
      // Wake the pack back up. The sequence ends here (no further steps sent).
      finish_sleep("NVROL reset");
      break;

      // Plain sleep sequence

    case CMFA_STATE_SLEEP_START:
      // Put the pack to sleep (no diagnostic session or reset) and hold the
      // bus for 30 s while it saves its state and falls asleep.
      start_sleep(CMFA_STATE_SLEEP_WAIT);
      break;
    case CMFA_STATE_SLEEP_WAIT:
      // Wake the pack back up. The sequence ends here (no further steps sent).
      finish_sleep("Sleep cycle");
      break;

    default:
      break;
  }
}

void CmfaEvBattery::on_uds_sequence_timeout(uint16_t state) {
  // A sequence step got no response, even after retries. The superclass has
  // already ended the sequence.
  abort_sequence(state, "no response", 0);
}

bool CmfaEvBattery::sequence_step_rejected(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  if (sid != kNegativeResponseSid) {
    return false;
  }
  // Negative response: 7F <request SID> <NRC>. Don't send the next step, so
  // the sequence ends here.
  abort_sequence(state, "negative response", len >= 3 ? data[2] : 0);
  return true;
}

void CmfaEvBattery::abort_sequence(uint16_t state, const char* reason, uint8_t nrc) {
  // Never leave the pack asleep because a sequence failed (unless the Sleep
  // button put it there).
  if (!manual_sleep) {
    end_sleep();
  }

  if (nrc) {
    snprintf(sequence_status, sizeof(sequence_status), "FAILED at %s: %s (NRC 0x%02X)", sequence_step_name(state),
             reason, nrc);
  } else {
    snprintf(sequence_status, sizeof(sequence_status), "FAILED at %s: %s", sequence_step_name(state), reason);
  }
  logging.printf("[CMFA] UDS sequence %s\n", sequence_status);
}

const char* CmfaEvBattery::sequence_step_name(uint16_t state) {
  switch (state) {
    case CMFA_STATE_TEMPORISATION_DIAG:
      return "temporisation: extended session (10 03)";
    case CMFA_STATE_TEMPORISATION_SEND:
      return "temporisation: write 2E 92 81 01";
    case CMFA_STATE_NVROL_SESSION:
      return "NVROL: extended session (10 03)";
    case CMFA_STATE_NVROL_ROUTINE:
      return "NVROL: reset routine (31 01 B0 09)";
    case CMFA_STATE_NVROL_SLEEP_SESSION:
      return "NVROL: second extended session (10 03)";
    case CMFA_STATE_NVROL_SLEEP_WRITE:
      return "NVROL: write temporisation (2E 92 81 01)";
    default:
      return "unknown step";
  }
}

void CmfaEvBattery::begin_sleep() {
  sleep_phase = SLEEP_SHUTDOWN;
  sleep_request_ms = millis();
  bms_quiet = false;
  bms_wakes = 0;
  bms_quiet_after_ms = 0;
  logging.println("[CMFA] Shutting down like a car (0x3D3 = 0x44), watching for BMS frames to stop");
}

void CmfaEvBattery::end_sleep() {
  // Normal CAN sending resumes on the next transmit_can(), which wakes the pack.
  sleep_phase = SLEEP_AWAKE;
}

void CmfaEvBattery::start_sleep(uint16_t wait_state) {
  // Allow for the ~0.9 s of shutdown frames, so the bus is silent for the full 30 s.
  if (!sequence_wait(wait_state, CMFA_SLEEP_TICKS + CMFA_SHUTDOWN_TICKS)) {
    // UDS would keep polling (and wake the pack), so don't try.
    abort_sequence(wait_state, "UDS busy, could not start the sleep", 0);
    return;
  }
  begin_sleep();
}

void CmfaEvBattery::finish_sleep(const char* what) {
  end_sleep();

  if (bms_quiet) {
    snprintf(sequence_status, sizeof(sequence_status), "%s OK: BMS went quiet %lums after sleep request%s", what,
             bms_quiet_after_ms, bms_wakes ? " (after waking up again)" : "");
  } else if (bms_wakes) {
    snprintf(sequence_status, sizeof(sequence_status),
             "%s FAILED: BMS went quiet but started transmitting again (%u times)", what, bms_wakes);
  } else {
    snprintf(sequence_status, sizeof(sequence_status), "%s FAILED: BMS never stopped transmitting - pack did not sleep",
             what);
  }
  logging.printf("[CMFA] %s\n", sequence_status);
}

void CmfaEvBattery::transmit_can(unsigned long currentMillis) {
  // Sleep/Wake buttons.
  if (manual_sleep_requested && sleep_phase == SLEEP_AWAKE) {
    manual_sleep = true;
    begin_sleep();
  } else if (!manual_sleep_requested && manual_sleep) {
    manual_sleep = false;
    end_sleep();
    logging.println("[CMFA] Waking pack");
  }
  if (sleep_phase == SLEEP_SHUTDOWN && currentMillis - sleep_request_ms >= CMFA_SHUTDOWN_MS) {
    sleep_phase = SLEEP_SILENT;
    logging.println("[CMFA] Shutdown frames sent, stopping all CAN sending");
  }
  const bool awake = (sleep_phase == SLEEP_AWAKE);
  const bool shutting_down = (sleep_phase == SLEEP_SHUTDOWN);

  // Send 10ms CAN Message
  if (currentMillis - previousMillis10ms >= INTERVAL_10_MS) {
    previousMillis10ms = currentMillis;
    if (awake) {
      transmit_can_frame(&CMFA_1EA);
      transmit_can_frame(&CMFA_135);
      transmit_can_frame(&CMFA_134);
      transmit_can_frame(&CMFA_125);
    } else if (shutting_down) {
      // A real car's 0x135 carries on for ~0.9 s after its other 10 ms frames stop.
      transmit_can_frame(&CMFA_135);
    }

    CMFA_135.data.u8[1] = content_135[counter_10ms];
    // Bytes 3-4: 0x8000 + charger current in 25 mA steps (from the car log,
    // ~40 counts per amp). Report the pack's own charge current while
    // emulating charging, otherwise 0x8001 (idle) as before.
    uint16_t charger_current = 0x8001;
    if (car_mode == CAR_MODE_CHARGING && awake) {
      const int32_t current_dA = datalayer_battery->status.current_dA;  // positive = charging
      charger_current = 0x8000 + (uint16_t)std::min<int32_t>(std::max<int32_t>(current_dA * 4, 0), 0x7FFF);
    }
    CMFA_135.data.u8[3] = charger_current >> 8;
    CMFA_135.data.u8[4] = charger_current & 0xFF;
    CMFA_125.data.u8[3] = content_125[counter_10ms];
    counter_10ms = (counter_10ms + 1) % 16;  // counter_10ms cycles between 0-1-2-3..15-0-1...
  }
  // Send 100ms CAN Message
  if (currentMillis - previousMillis100ms >= INTERVAL_100_MS) {
    previousMillis100ms = currentMillis;

    update_car_state_frames();
    if (awake) {
      transmit_can_frame(&CMFA_59B);
      transmit_can_frame(&CMFA_3D3);
      if (car_mode == CAR_MODE_CHARGING) {
        transmit_can_frame(&CMFA_3D5);
      }
    } else if (shutting_down) {
      // Bytes 4 and 6 swap between 0x5D and 0xB2 every three frames, as when awake.
      const bool swap = (frames_3d3 / 3) & 1;
      frames_3d3 = (frames_3d3 + 1) % 6;
      CMFA_3D3_SHUTDOWN.data.u8[4] = swap ? 0xB2 : 0x5D;
      CMFA_3D3_SHUTDOWN.data.u8[6] = swap ? 0x5D : 0xB2;
      transmit_can_frame(&CMFA_3D3_SHUTDOWN);
      transmit_can_frame(&CMFA_3D5_SHUTDOWN);
      CMFA_3D4.data.u8[6] = 0x80;  // idle
      CMFA_3D4.data.u8[7] = 0x04;
    }

    if (vehicle_time_valid) {
      // The clock keeps running while we're silent, like the car's does.
      while ((long)(currentMillis - vehicle_time_last_tick_ms) >= (long)VEHICLE_TIME_TICK_MS) {
        vehicle_time_last_tick_ms += VEHICLE_TIME_TICK_MS;
        if (vehicle_time_min < VEHICLE_TIME_MAX) {
          vehicle_time_min++;
        }
      }
      if (awake || shutting_down) {
        CMFA_3D4.data.u8[3] = (vehicle_time_min >> 16) & 0xFF;
        CMFA_3D4.data.u8[4] = (vehicle_time_min >> 8) & 0xFF;
        CMFA_3D4.data.u8[5] = vehicle_time_min & 0xFF;
        transmit_can_frame(&CMFA_3D4);
      }
    }
  }

  // While the pack has been asked to sleep, detect its broadcast frames stopping.
  if (!awake && !bms_quiet && (long)(currentMillis - last_bms_frame_ms) >= (long)CMFA_SLEEP_QUIET_MS) {
    bms_quiet = true;
    // If the BMS was already silent before the request, count it as immediate.
    bms_quiet_after_ms = (long)(last_bms_frame_ms - sleep_request_ms) > 0 ? last_bms_frame_ms - sleep_request_ms : 0;
    logging.printf("[CMFA] BMS went quiet %lums after sleep request\n", bms_quiet_after_ms);
  }

  // Automatically enable temporisation if the BMS reports it as disabled.
  // Don't queue it while a UDS sequence (e.g. an NVROL reset) is in progress.
  // if (!uds_is_busy() && temporisation != 0xFF &&
  //     currentMillis - previousMillisTemporisation >= TEMPORISATION_RETRY_MS) {
  //   if (temporisation == 0) {
  //     previousMillisTemporisation = currentMillis;
  //     enable_temporisation();
  //   }
  // }

  if (!awake) {
    // The pack is asleep on purpose, so don't report it missing.
    datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  }
  if (manual_sleep) {
    // Nothing at all may be sent, or the pack wakes up. (The 30 s sleep
    // sequence holds UDS itself, and needs the UDS ticks to count down.)
    return;
  }

  // UDS PID polling and DTC handling
  transmit_uds_can(currentMillis);
}

void CmfaEvBattery::setup(void) {  // Performs one time setup at startup
  // UDS: send requests to 0x79B, accept replies from the BMS on 0x7BB.
  setup_uds(0x79B, 0x7BB);

  static const uint16_t pid_scan_list[] = {
      // Read the pack's times first, so we can start sending 0x3D4 quickly.
      PID_POLL_PACK_TIME_LIFE,
      PID_POLL_ABSOLUTE_TIME_SAVED,
      PID_POLL_SOH_AVERAGE,
      PID_POLL_AVERAGE_VOLTAGE_OF_CELLS,
      PID_POLL_HIGHEST_CELL_VOLTAGE,
      PID_POLL_LOWEST_CELL_VOLTAGE,
      PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE,
      PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE,
      PID_POLL_12V_BATTERY,
      PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING,
      PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING,
      PID_POLL_CUMULATIVE_ENERGY_IN_REGEN,
      PID_POLL_SOCZ,
      PID_POLL_USOC,
      //PID_POLL_CURRENT_OFFSET,
      PID_POLL_100MS_CURRENT,
      PID_POLL_MAX_REGEN,
      PID_POLL_AVERAGE_TEMPERATURE,
      PID_POLL_MIN_TEMPERATURE,
      PID_POLL_MAX_TEMPERATURE,
      PID_POLL_END_OF_CHARGE_FLAG,
      PID_POLL_INTERLOCK_FLAG,
      PID_POLL_BALANCE_CAPACITY_TOTAL,
      PID_POLL_BALANCE_TIME_TOTAL,
      PID_POLL_BALANCE_CAPACITY_SLEEP,
      PID_POLL_BALANCE_TIME_SLEEP,
      PID_POLL_BALANCE_CAPACITY_WAKE,
      PID_POLL_BALANCE_TIME_WAKE,
      PID_POLL_BMS_STATE,
      PID_POLL_TEMPORISATION,
      PID_POLL_MISSION_INDEX,
      PID_POLL_MISSION_MODE,
      PID_POLL_MISSION_TIME_BEG,
      PID_POLL_MISSION_TIME_END,
      PID_POLL_MISSION_TEMP_BEG,
      PID_POLL_MISSION_TEMP_END,
      PID_POLL_MISSION_USOC_BEG,
      PID_POLL_MISSION_USOC_END,
      PID_POLL_MISSION_DIST_END,
      PID_POLL_BALANCE_SWITCHES,
      PID_POLL_CELL_1,
      PID_POLL_CELL_2,
      PID_POLL_CELL_3,
      PID_POLL_CELL_4,
      PID_POLL_CELL_5,
      PID_POLL_CELL_6,
      PID_POLL_CELL_7,
      PID_POLL_CELL_8,
      PID_POLL_CELL_9,
      PID_POLL_CELL_10,
      PID_POLL_CELL_11,
      PID_POLL_CELL_12,
      PID_POLL_CELL_13,
      PID_POLL_CELL_14,
      PID_POLL_CELL_15,
      PID_POLL_CELL_16,
      PID_POLL_CELL_17,
      PID_POLL_CELL_18,
      PID_POLL_CELL_19,
      PID_POLL_CELL_20,
      PID_POLL_CELL_21,
      PID_POLL_CELL_22,
      PID_POLL_CELL_23,
      PID_POLL_CELL_24,
      PID_POLL_CELL_25,
      PID_POLL_CELL_26,
      PID_POLL_CELL_27,
      PID_POLL_CELL_28,
      PID_POLL_CELL_29,
      PID_POLL_CELL_30,
      PID_POLL_CELL_31,
      PID_POLL_CELL_32,
      PID_POLL_CELL_33,
      PID_POLL_CELL_34,
      PID_POLL_CELL_35,
      PID_POLL_CELL_36,
      PID_POLL_CELL_37,
      PID_POLL_CELL_38,
      PID_POLL_CELL_39,
      PID_POLL_CELL_40,
      PID_POLL_CELL_41,
      PID_POLL_CELL_42,
      PID_POLL_CELL_43,
      PID_POLL_CELL_44,
      PID_POLL_CELL_45,
      PID_POLL_CELL_46,
      PID_POLL_CELL_47,
      PID_POLL_CELL_48,
      PID_POLL_CELL_49,
      PID_POLL_CELL_50,
      PID_POLL_CELL_51,
      PID_POLL_CELL_52,
      PID_POLL_CELL_53,
      PID_POLL_CELL_54,
      PID_POLL_CELL_55,
      PID_POLL_CELL_56,
      PID_POLL_CELL_57,
      PID_POLL_CELL_58,
      PID_POLL_CELL_59,
      PID_POLL_CELL_60,
      PID_POLL_CELL_61,
      PID_POLL_CELL_62,
      PID_POLL_CELL_63,
      PID_POLL_CELL_64,
      PID_POLL_CELL_65,
      PID_POLL_CELL_66,
      PID_POLL_CELL_67,
      PID_POLL_CELL_68,
      PID_POLL_CELL_69,
      PID_POLL_CELL_70,
      PID_POLL_CELL_71,
      PID_POLL_CELL_72,
  };
  set_pid_scan_list(pid_scan_list, sizeof(pid_scan_list) / sizeof(pid_scan_list[0]));

  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.system.status.battery_allows_contactor_closing = true;
  datalayer_battery->info.number_of_cells = 72;
  datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
}
