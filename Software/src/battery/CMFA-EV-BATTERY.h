#ifndef CMFA_EV_BATTERY_H
#define CMFA_EV_BATTERY_H

#include "../datalayer/datalayer.h"
#include "UdsCanBattery.h"

class CmfaEvBattery : public UdsCanBattery {
 public:
  // Use this constructor for the second battery.
  CmfaEvBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr, CAN_Interface targetCan) : UdsCanBattery(targetCan) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
    dtc = &datalayer_battery->dtc;

    average_voltage_of_cells = 0;
  }

  // Use the default constructor to create the first or single battery.
  CmfaEvBattery() : UdsCanBattery() {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
    dtc = &datalayer_battery->dtc;
  }

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  virtual void enable_temporisation();
  virtual void on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) override;
  virtual void on_uds_sequence_timeout(uint16_t state) override;
  static constexpr const char* Name = "CMFA platform, 27 kWh battery";

  bool supports_reset_NVROL() { return true; }
  void reset_NVROL() { start_sequence(CMFA_STATE_NVROL_START); }
  // The generic "BMS Reset" button is reused to put the pack through a plain
  // 30 s sleep cycle (no diagnostic session or NVROL reset).
  bool supports_reset_BMS() { return true; }
  void reset_BMS() { start_sequence(CMFA_STATE_SLEEP_START); }

  // Sleep/Wake buttons: put the pack to sleep like a car does (see
  // begin_sleep()) and keep the bus silent until woken. Not persisted.
  bool supports_sleep_wake() override { return true; }
  bool is_sleep_requested() override { return manual_sleep_requested; }
  void request_sleep() override { manual_sleep_requested = true; }
  void request_wake() override { manual_sleep_requested = false; }

  // Experimental: the balancing buttons switch the car state we emulate
  // between the emulator's original frames and the frames a real car sends
  // while AC charging, in the hope that the pack then behaves as it does at
  // the end of a real charge (and balances). Not persisted across reboots.
  bool supports_balancing_request() override { return true; }
  void initiate_balancing() override { car_mode = CAR_MODE_CHARGING; }
  void end_balancing() override { car_mode = CAR_MODE_DEFAULT; }

  // The pack reports charge/discharge power limits both in the 0x3D6
  // broadcast and via UDS DIDs. The broadcast limits are used by default; some
  // packs broadcast zero limits (e.g. in an error state) while the DIDs are
  // sensible, so this switches between them at runtime (not persisted).
  bool supports_toggle_power_limit_source() override { return true; }
  void toggle_power_limit_source() override { use_broadcast_power_limits = !use_broadcast_power_limits; }

  String get_uds_info_html() override;
  const char* get_dtc_json_filename() override { return "cmfa_dacia_spring_renault_kze_dtc.json"; }

 protected:
  // Called by the UDS superclass for each successful PID query response.
  uint16_t handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) override;

 private:
  DATALAYER_BATTERY_TYPE* datalayer_battery;

  // UDS sequence states for this battery.
  enum CmfaUdsState : uint16_t {
    // UDS temporisation sequence
    CMFA_STATE_TEMPORISATION_START = 0x01,
    CMFA_STATE_TEMPORISATION_DIAG,  // 0x10 0x03 (extended session)
    CMFA_STATE_TEMPORISATION_SEND,  // 0x2E 0x9281 (write temporisation)

    // NVROL reset sequence (ported from the Renault Zoe Gen2 integration).
    // After resetting the NVROL memory we enable "temporisation before sleep",
    // then shut down like a car and stop all CAN sending for 30 s so the pack
    // saves its state and falls asleep, and finally wake it back up.
    CMFA_STATE_NVROL_START = 0x10,
    CMFA_STATE_NVROL_SESSION,        // 0x10 0x03 (extended diagnostic session)
    CMFA_STATE_NVROL_ROUTINE,        // 0x31 01 B0 09 (NVROL reset routine)
    CMFA_STATE_NVROL_SLEEP_SESSION,  // 0x10 0x03 (extended session, again)
    CMFA_STATE_NVROL_SLEEP_WRITE,    // 0x2E 92 81 01 (enable temporisation before sleep)
    CMFA_STATE_NVROL_SLEEP_WAIT,     // 30 s: CAN silent so the pack falls asleep

    // Plain sleep sequence: put the pack to sleep for 30 s and wake it again,
    // without any diagnostic session or NVROL reset.
    CMFA_STATE_SLEEP_START = 0x20,
    CMFA_STATE_SLEEP_WAIT,  // 30 s: CAN silent so the pack falls asleep
  };

  // Timeouts for the UDS sequences (in UDS ticks).
  static constexpr uint16_t CMFA_UDS_TIMEOUT_SESSION_CONTROL = 10;
  static constexpr uint16_t CMFA_UDS_TIMEOUT_WRITE = 50;
  static constexpr uint16_t CMFA_NVROL_TIMEOUT_TICKS = 10;  // 1 s, per NVROL sequence step
  static constexpr uint16_t CMFA_SLEEP_TICKS = 300;         // 30 s

  // Sequence error handling: a negative response or a timeout aborts the
  // sequence, wakes the pack back up and records why in sequence_status.
  bool sequence_step_rejected(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len);
  void abort_sequence(uint16_t state, const char* reason, uint8_t nrc);
  static const char* sequence_step_name(uint16_t state);

  // Sleeping. A real car shuts down by switching 0x3D3 byte 0 to 0x44 (and
  // 0x3D5 byte 3 to 0xE0): its 10 ms frames stop at once, 0x3D3/0x3D4/0x3D5/
  // 0x135 carry on for ~0.9 s, and then the bus is silent. (0x1EA stays 0x00
  // throughout; the car never sends anything else in it.) Any frame on the bus
  // wakes the pack again, so nothing at all may be sent while it sleeps.
  //
  // Sleep detection: while the pack has been asked to sleep, watch for the
  // BMS's own broadcast frames stopping. If they don't stop, the pack never
  // went through a controlled shutdown (and an NVROL reset won't be saved).
  void begin_sleep();
  void end_sleep();
  void start_sleep(uint16_t wait_state);  // 30 s sleep as a UDS sequence step
  void finish_sleep(const char* what);
  enum SleepPhase : uint8_t {
    SLEEP_AWAKE = 0,
    SLEEP_SHUTDOWN,  // sending the car's shutdown frames
    SLEEP_SILENT,    // sending nothing
  };
  uint8_t sleep_phase = SLEEP_AWAKE;
  static constexpr unsigned long CMFA_SHUTDOWN_MS = 900;
  static constexpr uint16_t CMFA_SHUTDOWN_TICKS = 10;         // UDS ticks to allow for the shutdown frames
  static constexpr unsigned long CMFA_SLEEP_QUIET_MS = 2000;  // no BMS frames for this long = asleep
  volatile bool manual_sleep_requested = false;               // Sleep button pressed (and Wake not yet)
  bool manual_sleep = false;                                  // the current sleep was started by the Sleep button
  volatile unsigned long last_bms_frame_ms = 0;
  unsigned long sleep_request_ms = 0;
  unsigned long bms_quiet_after_ms = 0;  // how long after the sleep request the BMS went quiet
  volatile bool bms_quiet = false;
  volatile uint8_t bms_wakes = 0;  // times the BMS started transmitting again after going quiet
  char sequence_status[128] = "None";

  // If not null, this battery decides when the contactor can be closed and writes the value here.
  bool* allows_contactor_closing;

  uint16_t rescale_raw_SOC(uint32_t raw_SOC, uint32_t max_soc);

  // Vehicle clock (0x3D4). The car broadcasts a free-running minute counter
  // in bytes 3-5, and the pack keeps its own copies (DIDs 0x91C1 pack time
  // life, 0x9261 absolute time of vehicle saved). The BMS gets confused if
  // this time ever goes backwards, so we don't send 0x3D4 until both DIDs have
  // been read, start just above the larger of the two, and only ever move
  // forwards from there.
  //
  // Only 0x9261 is followed after that: in the car log 0x91C1 ran a fixed
  // offset ahead of 0x3D4, so if the pack advances it from our 0x3D4, chasing
  // it would make the time run away.
  void update_vehicle_time_from_pack();
  void log_pack_time_change(const char* name, uint32_t old_value, uint32_t new_value);
  // Diagnostics, to spot the time running ahead of real time.
  uint32_t vehicle_time_start = 0;          // first value sent
  unsigned long vehicle_time_start_ms = 0;  // when it was first sent
  uint32_t vehicle_time_jumped_min = 0;     // total forward jumps after the start
  uint16_t vehicle_time_jumps = 0;
  int32_t initial_offset_pack_time_life = 0;  // 0x91C1 - vehicle time at the start
  int32_t initial_offset_time_saved = 0;      // 0x9261 - vehicle time at the start
  static constexpr unsigned long VEHICLE_TIME_TICK_MS = 60000;
  static constexpr uint32_t VEHICLE_TIME_MAX = 0xFFFFFF;  // 24-bit counter
  bool vehicle_time_valid = false;
  uint32_t vehicle_time_min = 0;
  unsigned long vehicle_time_last_tick_ms = 0;

  // A real car charges to 304.5 V at rest (4.23 V/cell average), 306.0 V
  // briefly with ~12 A still flowing. Allow a little above that, so a
  // fully charged pack isn't flagged as overvoltage.
  static const int MAX_PACK_VOLTAGE_DV = 3080;  // 5000 = 500.0V
  static const int MIN_PACK_VOLTAGE_DV = 2185;
  static const int MAX_CELL_DEVIATION_MV = 100;
  static const int MAX_CELL_VOLTAGE_MV = 4280;  // Emergency stop if above (308.0 V / 72)
  static const int MIN_CELL_VOLTAGE_MV = 2700;  // Emergency stop if below

  // OBD2 PID polls
  static const int PID_POLL_SOCZ = 0x9001;
  static const int PID_POLL_USOC = 0x9002;
  static const int PID_POLL_SOH_AVERAGE = 0x9003;
  static const int PID_POLL_AVERAGE_VOLTAGE_OF_CELLS = 0x9006;
  static const int PID_POLL_HIGHEST_CELL_VOLTAGE = 0x9007;
  static const int PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE = 0x9008;
  static const int PID_POLL_LOWEST_CELL_VOLTAGE = 0x9009;
  static const int PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE = 0x900A;
  static const int PID_POLL_CURRENT_OFFSET = 0x900C;
  static const int PID_POLL_INSTANT_CURRENT = 0x900D;
  static const int PID_POLL_MAX_REGEN = 0x900E;
  static const int PID_POLL_MAX_DISCHARGE_POWER = 0x900F;
  static const int PID_POLL_12V_BATTERY = 0x9011;
  static const int PID_POLL_AVERAGE_TEMPERATURE = 0x9012;
  static const int PID_POLL_MIN_TEMPERATURE = 0x9013;
  static const int PID_POLL_MAX_TEMPERATURE = 0x9014;
  static const int PID_POLL_MAX_CHARGE_POWER = 0x9018;
  static const int PID_POLL_END_OF_CHARGE_FLAG = 0x9019;
  static const int PID_POLL_INTERLOCK_FLAG = 0x901A;
  static const int PID_POLL_BATTERY_IDENTIFICATION = 0x901B;
  static const int PID_POLL_100MS_CURRENT = 0x925D;

  static const int PID_POLL_TEMPORISATION = 0x9281;

  static const int PID_POLL_PACK_TIME_LIFE = 0x91C1;
  static const int PID_POLL_ABSOLUTE_TIME_SAVED = 0x9261;

  static const int PID_POLL_CELL_1 = 0x9021;
  static const int PID_POLL_CELL_2 = 0x9022;
  static const int PID_POLL_CELL_3 = 0x9023;
  static const int PID_POLL_CELL_4 = 0x9024;
  static const int PID_POLL_CELL_5 = 0x9025;
  static const int PID_POLL_CELL_6 = 0x9026;
  static const int PID_POLL_CELL_7 = 0x9027;
  static const int PID_POLL_CELL_8 = 0x9028;
  static const int PID_POLL_CELL_9 = 0x9029;
  static const int PID_POLL_CELL_10 = 0x902A;
  static const int PID_POLL_CELL_11 = 0x902B;
  static const int PID_POLL_CELL_12 = 0x902C;
  static const int PID_POLL_CELL_13 = 0x902D;
  static const int PID_POLL_CELL_14 = 0x902E;
  static const int PID_POLL_CELL_15 = 0x902F;
  static const int PID_POLL_CELL_16 = 0x9030;
  static const int PID_POLL_CELL_17 = 0x9031;
  static const int PID_POLL_CELL_18 = 0x9032;
  static const int PID_POLL_CELL_19 = 0x9033;
  static const int PID_POLL_CELL_20 = 0x9034;
  static const int PID_POLL_CELL_21 = 0x9035;
  static const int PID_POLL_CELL_22 = 0x9036;
  static const int PID_POLL_CELL_23 = 0x9037;
  static const int PID_POLL_CELL_24 = 0x9038;
  static const int PID_POLL_CELL_25 = 0x9039;
  static const int PID_POLL_CELL_26 = 0x903A;
  static const int PID_POLL_CELL_27 = 0x903B;
  static const int PID_POLL_CELL_28 = 0x903C;
  static const int PID_POLL_CELL_29 = 0x903D;
  static const int PID_POLL_CELL_30 = 0x903E;
  static const int PID_POLL_CELL_31 = 0x903F;

  static const int PID_POLL_DIDS_SUPPORTED_IN_RANGE_9041_9060 = 0x9040;

  static const int PID_POLL_CELL_32 = 0x9041;
  static const int PID_POLL_CELL_33 = 0x9042;
  static const int PID_POLL_CELL_34 = 0x9043;
  static const int PID_POLL_CELL_35 = 0x9044;
  static const int PID_POLL_CELL_36 = 0x9045;
  static const int PID_POLL_CELL_37 = 0x9046;
  static const int PID_POLL_CELL_38 = 0x9047;
  static const int PID_POLL_CELL_39 = 0x9048;
  static const int PID_POLL_CELL_40 = 0x9049;
  static const int PID_POLL_CELL_41 = 0x904A;
  static const int PID_POLL_CELL_42 = 0x904B;
  static const int PID_POLL_CELL_43 = 0x904C;
  static const int PID_POLL_CELL_44 = 0x904D;
  static const int PID_POLL_CELL_45 = 0x904E;
  static const int PID_POLL_CELL_46 = 0x904F;
  static const int PID_POLL_CELL_47 = 0x9050;
  static const int PID_POLL_CELL_48 = 0x9051;
  static const int PID_POLL_CELL_49 = 0x9052;
  static const int PID_POLL_CELL_50 = 0x9053;
  static const int PID_POLL_CELL_51 = 0x9054;
  static const int PID_POLL_CELL_52 = 0x9055;
  static const int PID_POLL_CELL_53 = 0x9056;
  static const int PID_POLL_CELL_54 = 0x9057;
  static const int PID_POLL_CELL_55 = 0x9058;
  static const int PID_POLL_CELL_56 = 0x9059;
  static const int PID_POLL_CELL_57 = 0x905A;
  static const int PID_POLL_CELL_58 = 0x905B;
  static const int PID_POLL_CELL_59 = 0x905C;
  static const int PID_POLL_CELL_60 = 0x905D;
  static const int PID_POLL_CELL_61 = 0x905E;
  static const int PID_POLL_CELL_62 = 0x905F;

  static const int PID_POLL_DIDS_SUPPORTED_IN_RANGE_9061_9080 = 0x9060;

  static const int PID_POLL_CELL_63 = 0x9061;
  static const int PID_POLL_CELL_64 = 0x9062;
  static const int PID_POLL_CELL_65 = 0x9063;
  static const int PID_POLL_CELL_66 = 0x9064;
  static const int PID_POLL_CELL_67 = 0x9065;
  static const int PID_POLL_CELL_68 = 0x9066;
  static const int PID_POLL_CELL_69 = 0x9067;
  static const int PID_POLL_CELL_70 = 0x9068;
  static const int PID_POLL_CELL_71 = 0x9069;
  static const int PID_POLL_CELL_72 = 0x906A;

  static const int PID_POLL_SOH_AVAILABLE_POWER_CALCULATION = 0x91BC;
  static const int PID_POLL_SOH_GENERATED_POWER_CALCULATION = 0x91BD;

  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING = 0x9243;
  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING = 0x9245;
  static const int PID_POLL_CUMULATIVE_ENERGY_IN_REGEN = 0x9247;

  static const int PID_POLL_BALANCE_CAPACITY_TOTAL = 0x924F;
  static const int PID_POLL_BALANCE_TIME_TOTAL = 0x9250;
  static const int PID_POLL_BALANCE_CAPACITY_SLEEP = 0x9251;
  static const int PID_POLL_BALANCE_TIME_SLEEP = 0x9252;
  static const int PID_POLL_BALANCE_CAPACITY_WAKE = 0x9262;
  static const int PID_POLL_BALANCE_TIME_WAKE = 0x9263;
  static const int PID_POLL_BMS_STATE = 0x9259;
  static const int PID_POLL_BALANCE_SWITCHES = 0x912B;

  // Mission log: 32-entry tables, one entry per "mission" (charge/drive/...).
  static const int PID_POLL_MISSION_MODE = 0x9270;      // BMS state, 1 byte each
  static const int PID_POLL_MISSION_TEMP_BEG = 0x9271;  // degC + 40, 1 byte each
  static const int PID_POLL_MISSION_TEMP_END = 0x9272;  // degC + 40, 1 byte each
  static const int PID_POLL_MISSION_USOC_BEG = 0x9273;  // %, 1 byte each
  static const int PID_POLL_MISSION_USOC_END = 0x9274;  // %, 1 byte each
  static const int PID_POLL_MISSION_TIME_BEG = 0x9275;  // pack time minutes, 3 bytes each
  static const int PID_POLL_MISSION_TIME_END = 0x9276;  // pack time minutes, 3 bytes each
  static const int PID_POLL_MISSION_DIST_END = 0x9278;  // 0x80000000 + km/32, 4 bytes each
  static const int PID_POLL_MISSION_INDEX = 0x9279;

  CAN_frame CMFA_1EA = {.FD = false, .ext_ID = false, .DLC = 1, .ID = 0x1EA, .data = {0x00}};
  CAN_frame CMFA_125 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x125,
                        .data = {0x7D, 0x7D, 0x7D, 0x07, 0x82, 0x6A, 0x8A}};
  CAN_frame CMFA_134 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x134,
                        .data = {0x90, 0x8A, 0x7E, 0x3E, 0xB2, 0x4C, 0x80, 0x00}};
  CAN_frame CMFA_135 = {.FD = false, .ext_ID = false, .DLC = 5, .ID = 0x135, .data = {0xD5, 0x85, 0x38, 0x80, 0x01}};
  CAN_frame CMFA_3D3 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x3D3,
                        .data = {0x47, 0x30, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xE7}};
  // Bytes 0-2 and 6-7 are copied from a real car at idle (meaning unknown /
  // 0x80 = no power demand / idle flags); bytes 3-5 are the vehicle time.
  CAN_frame CMFA_3D4 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x3D4,
                        .data = {0x01, 0x2A, 0xD9, 0x00, 0x00, 0x00, 0x80, 0x04}};
  // Only sent while emulating charging: byte 3 0x60 = charging (0x40 plug
  // present, 0x00 unplugged, 0xE0 shutting down in the car log).
  CAN_frame CMFA_3D5 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x3D5,
                        .data = {0xFF, 0xFF, 0x1E, 0x60, 0x00, 0x00, 0x00}};
  // 0x3D3/0x3D5 as a real car sends them while shutting down.
  CAN_frame CMFA_3D3_SHUTDOWN = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 8,
                                 .ID = 0x3D3,
                                 .data = {0x44, 0x1E, 0x00, 0x01, 0x5D, 0xA0, 0xB2, 0xCA}};
  CAN_frame CMFA_3D5_SHUTDOWN = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 7,
                                 .ID = 0x3D5,
                                 .data = {0xFF, 0xFF, 0x1E, 0xE0, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_59B = {.FD = false, .ext_ID = false, .DLC = 3, .ID = 0x59B, .data = {0x00, 0x02, 0x00}};
  bool end_of_charge = false;
  bool interlock_flag = false;
  uint16_t soc_z = 0;
  uint16_t soc_u = 0;
  uint32_t max_regen_power = 0;
  uint32_t max_discharge_power = 0;
  int16_t average_temperature = 0;
  int16_t minimum_temperature = 0;
  int16_t maximum_temperature = 0;
  uint32_t maximum_charge_power = 0;
  uint16_t SOH_available_power = 0;
  uint16_t SOH_generated_power = 0;
  uint32_t average_voltage_of_cells = 270000;
  uint16_t highest_cell_voltage_mv = 3700;
  uint16_t lowest_cell_voltage_mv = 3700;
  uint16_t lead_acid_voltage = 12000;
  uint8_t highest_cell_voltage_number = 0;
  uint8_t lowest_cell_voltage_number = 0;
  uint32_t cumulative_energy_when_discharging = 0;
  uint32_t cumulative_energy_when_charging = 0;
  uint32_t cumulative_energy_in_regen = 0;
  uint32_t pack_time_life = 0;       // Minutes
  uint32_t absolute_time_saved = 0;  // Minutes
  // Initial values, UINT32_MAX until the DID has been read at least once
  uint32_t initial_pack_time_life = UINT32_MAX;
  uint32_t initial_absolute_time_saved = UINT32_MAX;
  uint16_t soh_average = 10000;
  int32_t balance_capacity_total = 0;  // 1/1024 Ah
  int32_t balance_time_total = 0;      // 1/1024 h
  int32_t balance_capacity_sleep = 0;  // 1/1024 Ah
  int32_t balance_time_sleep = 0;      // 1/1024 h
  int32_t balance_capacity_wake = 0;   // 1/1024 Ah
  int32_t balance_time_wake = 0;       // 1/1024 h
  int32_t initial_balance_capacity_total = INT32_MIN;
  int32_t initial_balance_time_total = INT32_MIN;
  int32_t initial_balance_capacity_sleep = INT32_MIN;
  int32_t initial_balance_time_sleep = INT32_MIN;
  int32_t initial_balance_capacity_wake = INT32_MIN;
  int32_t initial_balance_time_wake = INT32_MIN;
  uint8_t bms_state = 0;

  static constexpr uint8_t MISSION_ENTRIES = 32;
  struct {
    uint8_t mode[MISSION_ENTRIES];
    uint8_t temp_beg[MISSION_ENTRIES];
    uint8_t temp_end[MISSION_ENTRIES];
    uint8_t usoc_beg[MISSION_ENTRIES];
    uint8_t usoc_end[MISSION_ENTRIES];
    uint32_t time_beg[MISSION_ENTRIES];
    uint32_t time_end[MISSION_ENTRIES];
    uint32_t dist_end[MISSION_ENTRIES];
    uint8_t index = 0xFF;
    bool read = false;  // true once the times have been read
  } missions = {};
  void handle_mission_pid(uint16_t pid, const uint8_t* data, uint16_t length);
  void append_mission_html(String& content);
  uint8_t temporisation = 0xFF;  // 0xFF until the DID has been read at least once

  // Automatically re-enable temporisation at most every 60s.
  static constexpr unsigned long TEMPORISATION_RETRY_MS = 60000;
  unsigned long previousMillisTemporisation = 0;

  uint8_t counter_10ms = 0;
  uint8_t content_125[16] = {0x07, 0x0C, 0x01, 0x06, 0x0B, 0x00, 0x05, 0x0A,
                             0x0F, 0x04, 0x09, 0x0E, 0x03, 0x08, 0x0D, 0x02};
  uint8_t content_135[16] = {0x85, 0xD5, 0x25, 0x75, 0xC5, 0x15, 0x65, 0xB5,
                             0x05, 0x55, 0xA5, 0xF5, 0x45, 0x95, 0xE5, 0x35};
  unsigned long previousMillis100ms = 0;
  unsigned long previousMillis10ms = 0;

  static const int MAXSOC = 9000;  //90.00 Raw SOC displays this value when battery is at 100%
  // While emulating charging, report 100% only when the pack itself says
  // 100.00% (a real car charges on until then and beyond: its dash shows 100%
  // at raw 97.5%, and the pack ends the charge itself after reaching 100%).
  static const int MAXSOC_CHARGING = 10000;
  static const int MINSOC = 500;  //5.00 Raw SOC displays this value when battery is at 0%

  uint8_t heartbeat = 0;   //Alternates between 0x55 and 0xAA every 5th frame
  uint8_t heartbeat2 = 0;  //Alternates between 0x55 and 0xAA every 5th frame
  uint32_t SOC_raw = 20000;
  uint16_t SOH = 99;
  int16_t current_raw = 2000;
  uint32_t instant_current_raw = 48000;
  uint32_t averaged_current_raw = 32640;
  uint16_t pack_voltage = 0;
  int16_t highest_cell_temperature = 0;
  int16_t lowest_cell_temperature = 0;
  uint32_t discharge_power_w = 0;
  uint32_t charge_power_w = 0;
  // 0x127 byte 0: the pack's own charge power request (same as Zoe 0x155
  // byte 0, 300 W/bit). Steps down to 0 as the pack ends a charge itself.
  uint32_t charge_power_request_w = 0;
  volatile bool use_broadcast_power_limits = true;
  // 0x43C byte 0: bit 6 set (0x95/0x96) while the pack is idle, clear (0x55)
  // while charging or READY. 0xFF = not seen yet.
  uint8_t status_43c = 0xFF;
  // 0x3D7 byte 0: the pack's mode as seen in a real car: 0x0A charging,
  // 0x24 idle, 0x2A READY. 0xFF = not seen yet.
  uint8_t status_3d7 = 0xFF;

  // Car state we emulate on 0x3D3/0x3D5/0x3D4/0x135.
  enum CarMode : uint8_t {
    CAR_MODE_DEFAULT = 0,   // The emulator's original frames (pack reports idle)
    CAR_MODE_CHARGING = 1,  // As a real car sends while AC charging
  };
  volatile uint8_t car_mode = CAR_MODE_DEFAULT;
  uint8_t frames_3d3 = 0;  // for the 0x5D/0xB2 swap in bytes 4/6 every 3 frames
  void update_car_state_frames();
};

#endif
