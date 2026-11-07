#ifndef CMFA_EV_BATTERY_H
#define CMFA_EV_BATTERY_H

#include "../datalayer/datalayer.h"
#include "../devboard/utils/IndexableCircularBuffer.h"
#include "../devboard/utils/types.h"
#include "CanBattery.h"

class CmfaEvBattery : public CanBattery {
 public:
  CmfaEvBattery() : CanBattery(CAN_Speed::CAN_SPEED_500KBPS) {}
  static constexpr const char* Name = "Dacia Spring";

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  void enable_temporisation() { start_uds_sequence(UDS_TEMP_EXT_SESSION, "Temporisation"); }
  void on_uds_sequence_step();
  void on_uds_sequence_timeout();

  bool supports_reset_NVROL() override { return true; }
  void reset_NVROL() override { start_uds_sequence(UDS_NVROL_EXT_SESSION_1, "NVROL"); }

  bool supports_reset_BMS() override { return true; }
  void reset_BMS() override { start_sleep("Sleep button"); }

  bool supports_sleep_wake() override { return true; }
  bool is_sleep_requested() const override { return sleep_requested; }
  void request_sleep() override { begin_sleep("Sleep button", 0); }
  void request_wake() override { end_sleep(); }

  bool supports_balancing_request() override { return true; }
  void initiate_balancing() override { car_mode = CAR_MODE_CHARGING; }
  void end_balancing() override { car_mode = CAR_MODE_DEFAULT; }

  bool supports_toggle_power_limit_source() override { return true; }
  void toggle_power_limit_source() override { use_uds_power_limits = !use_uds_power_limits; }

  String get_uds_info_html() override;

  BatteryHtmlRenderer& get_status_renderer() { return renderer; }

 private:
  void handle_pid(uint16_t poll_pid, const uint8_t* data, uint8_t length);

  BatteryDefaultRenderer renderer;

  static const uint16_t CMFA_UDS_REQ_ID = 0x79B;
  static const uint16_t CMFA_UDS_RESP_ID = 0x7BB;
  enum CmfaUdsState : uint8_t {
    UDS_IDLE = 0,
    UDS_TEMP_EXT_SESSION,
    UDS_TEMP_WRITE_9281,
    UDS_NVROL_EXT_SESSION_1,
    UDS_NVROL_ROUTINE_B009,
    UDS_NVROL_EXT_SESSION_2,
    UDS_NVROL_WRITE_9281,
    UDS_DTC_READ,
    UDS_DTC_CLEAR,
  };
  CmfaUdsState uds_state = UDS_IDLE;
  unsigned long uds_step_started_ms = 0;
  static const unsigned long UDS_STEP_TIMEOUT_MS = 1000;
  bool uds_wait_done = false;
  uint16_t current_poll_pid = 0;

  static const unsigned long NVROL_SLEEP_DURATION_MS = 30000;
  String last_nvrol_status = "None";

  void start_uds_sequence(CmfaUdsState first_step, const char* label);
  void sequence_step_rejected(const char* tag);
  void abort_sequence(const char* reason);

  enum SleepPhase : uint8_t { SLEEP_AWAKE = 0, SLEEP_STOPPING, SLEEP_SHUTDOWN, SLEEP_SILENT };
  SleepPhase sleep_phase = SLEEP_AWAKE;
  unsigned long sleep_phase_started_ms = 0;
  unsigned long sleep_silent_duration_ms = 0;
  unsigned long sleep_low_current_since_ms = 0;
  static const unsigned long CMFA_SLEEP_STOP_SETTLE_MS = 3000;
  static const unsigned long CMFA_SLEEP_MAX_WAIT_MS = 30000;
  static const int16_t CMFA_SLEEP_MAX_CURRENT_DA = 10;
  static const unsigned long CMFA_SLEEP_SHUTDOWN_MS = 900;
  bool sleep_requested = false;
  bool bms_seen_silent_in_sleep = false;
  unsigned long last_rx_ms = 0;
  static const unsigned long BMS_SILENT_THRESHOLD_MS = 500;
  void begin_sleep(const char* label, unsigned long silent_duration_ms);
  void end_sleep();
  void start_sleep(const char* label) { begin_sleep(label, NVROL_SLEEP_DURATION_MS); }
  void continue_sleep(unsigned long now);
  void finish_sleep();

  int16_t pack_current_dA() const {
    return (instant_current_raw == 0) ? 0 : (int16_t)((((int32_t)instant_current_raw - 32767) * 25) / 10);
  }
  uint32_t rescale_raw_SOC(uint32_t raw_SOC);

  static const uint16_t MAX_PACK_VOLTAGE_DV = 3060;
  static const uint16_t MIN_PACK_VOLTAGE_DV = 1800;
  static const uint16_t MAX_CELL_DEVIATION_MV = 500;
  static const uint16_t MAX_CELL_VOLTAGE_MV = 4250;
  static const uint16_t MIN_CELL_VOLTAGE_MV = 2700;
  static const uint8_t NUMBER_OF_CELLS = 72;

  unsigned long previousMillis10 = 0;
  unsigned long previousMillis100 = 0;
  unsigned long previousMillis200 = 0;
  unsigned long previousMillis1000 = 0;

  static const uint32_t CMFA_VEHICLE_TIME_1_OFFSET = 0x12AD9;
  uint32_t initial_pack_time_life = UINT32_MAX;
  uint32_t initial_absolute_time_saved = UINT32_MAX;
  uint32_t vehicle_time_sent = 0;
  bool vehicle_time_valid = false;
  void update_vehicle_time_from_pack();
  void log_pack_time_change(const char* name, uint32_t prev, uint32_t now);

  uint32_t soc_u = 0;
  uint32_t soc_z = 0;
  uint16_t soh_average = 9900;
  uint16_t pack_voltage = 26000;
  uint16_t lv_bat_mv = 12000;
  uint16_t cell_voltages[NUMBER_OF_CELLS] = {0};
  uint16_t max_cell_voltage = 3700;
  uint16_t min_cell_voltage = 3700;
  uint16_t sum_of_cell_voltages = 0;
  uint16_tinstant_current_raw_pad = 0;
  uint16_t instant_current_raw = 32767;
  uint16_t averaged_current_raw = 32640;
  uint32_t max_regen_power = 0;
  uint32_t max_discharge_power = 0;
  uint32_t max_charge_power = 0;
  int16_t average_temperature = 100;
  int16_t minimum_temperature = 100;
  int16_t maximum_temperature = 100;
  uint8_t highest_cell_number = 0;
  uint8_t lowest_cell_number = 0;
  uint64_t cumulative_energy_when_discharging = 0;
  uint64_t cumulative_energy_when_charging = 0;
  uint64_t cumulative_energy_in_regen = 0;
  uint16_t soh_available_power = 0;
  uint16_t soh_generated_power = 0;
  uint32_t lead_acid_voltage = 12000;
  uint8_t end_of_charge_flag = 0;
  uint8_t interlock_flag = 0;

  static const uint16_t BAL_UNSET = 0xFFFF;
  uint16_t bal_cap_total_raw = BAL_UNSET;
  uint16_t bal_time_total_raw = BAL_UNSET;
  uint16_t bal_cap_sleep_raw = BAL_UNSET;
  uint16_t bal_time_sleep_raw = BAL_UNSET;
  uint16_t bal_cap_wake_raw = BAL_UNSET;
  uint16_t bal_time_wake_raw = BAL_UNSET;
  uint16_t initial_bal_cap_total_raw = BAL_UNSET;
  uint16_t initial_bal_time_total_raw = BAL_UNSET;
  uint16_t initial_bal_cap_sleep_raw = BAL_UNSET;
  uint16_t initial_bal_time_sleep_raw = BAL_UNSET;
  uint16_t initial_bal_cap_wake_raw = BAL_UNSET;
  uint16_t initial_bal_time_wake_raw = BAL_UNSET;
  uint8_t temporisation_flag = 0xFF;
  uint32_t pack_time_life = 0;
  uint32_t absolute_time_saved = 0;
  uint8_t bms_state = 0;
  uint8_t request_from_can = 7;

  bool use_uds_power_limits = false;
  uint16_t broadcast_charge_power_w = 0;
  uint16_t broadcast_discharge_power_w = 0;
  uint16_t broadcast_127_charge_power_w = 0;
  uint8_t broadcast_3D7_mode_byte = 0;
  uint8_t broadcast_43C_byte0 = 0;

  static const int PID_POLL_SOC_Z = 0x9001;
  static const int PID_POLL_SOC_U = 0x9002;
  static const int PID_POLL_SOH_AVERAGE = 0x9003;
  static const int PID_POLL_PACK_VOLTAGE = 0x9005;
  static const int PID_POLL_AVERAGE_VOLTAGE_OF_CELLS = 0x9006;
  static const int PID_POLL_MAX_CELL_VOLTAGE = 0x9007;
  static const int PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE = 0x9008;
  static const int PID_POLL_MIN_CELL_VOLTAGE = 0x9009;
  static const int PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE = 0x900A;
  static const int PID_POLL_INSTANT_CURRENT = 0x900D;
  static const int PID_POLL_MAXIMUM_GENERATED_POWER = 0x900E;
  static const int PID_POLL_AVAILABLE_POWER = 0x900F;
  static const int PID_POLL_LV_BAT = 0x9011;
  static const int PID_POLL_AVERAGE_TEMPERATURE = 0x9012;
  static const int PID_POLL_MIN_TEMPERATURE = 0x9013;
  static const int PID_POLL_MAX_TEMPERATURE = 0x9014;
  static const int PID_POLL_MAXIMUM_CHARGE_POWER = 0x9018;
  static const int PID_POLL_END_OF_CHARGE_FLAG = 0x9019;
  static const int PID_POLL_HVBATT_INTERLOCK_FLAG = 0x901A;
  static const int PID_POLL_PACK_TIME_LIFE = 0x91C1;
  static const int PID_POLL_ABSOLUTE_TIME_SAVED = 0x9261;
  static const int PID_POLL_BMS_STATE = 0x901E;
  static const int PID_POLL_REQUEST_FROM_CAN = 0x901F;
  static const int PID_POLL_TEMPORISATION = 0x9281;

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

  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING = 0x9243;
  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING = 0x9245;
  static const int PID_POLL_CUMULATIVE_ENERGY_IN_REGEN = 0x9247;
  static const int PID_POLL_BALANCE_CAPACITY_TOTAL = 0x924F;
  static const int PID_POLL_BALANCE_TIME_TOTAL = 0x9250;
  static const int PID_POLL_BALANCE_CAPACITY_SLEEP = 0x9251;
  static const int PID_POLL_BALANCE_TIME_SLEEP = 0x9252;
  static const int PID_POLL_SOH_AVAILABLE_POWER = 0x925A;
  static const int PID_POLL_SOH_GENERATED_POWER = 0x925B;
  static const int PID_POLL_100MS_CURRENT = 0x925D;
  static const int PID_POLL_BALANCE_CAPACITY_WAKE = 0x9262;
  static const int PID_POLL_BALANCE_TIME_WAKE = 0x9263;

  static const int PID_POLL_MISSION_FIRST = 0x91E1;
  static const int PID_POLL_MISSION_LAST = 0x91E4;

  static const int FAST_POLL_COUNT = 3;
  const int fast_poll_pids[FAST_POLL_COUNT] = {
      PID_POLL_INSTANT_CURRENT,
      PID_POLL_MAXIMUM_CHARGE_POWER,
      PID_POLL_AVAILABLE_POWER,
  };
  uint8_t fast_poll_index = 0;
  uint8_t fast_slot_counter = 0;

  IndexableCircularBuffer<int, 100> slow_poll_pids;
  uint8_t slow_poll_cursor = 0;

  CAN_frame CMFA_1EA = {.FD = false, .ext_ID = false, .DLC = 1, .ID = 0x1EA, .data = {0x00}};
  CAN_frame CMFA_125 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x125,
                        .data = {0x7D, 0x7D, 0x7D, 0x03, 0x82, 0x6A, 0x8A}};
  CAN_frame CMFA_134 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x134,
                        .data = {0x05, 0x4A, 0xCC, 0x41, 0x62, 0x4C, 0x80, 0x00}};
  CAN_frame CMFA_135 = {.FD = false, .ext_ID = false, .DLC = 5, .ID = 0x135, .data = {0xD5, 0x45, 0x38, 0x80, 0x01}};
  CAN_frame CMFA_3D3 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x3D3,
                        .data = {0x47, 0x1E, 0x00, 0x01, 0x5D, 0xA0, 0xB2, 0xDB}};
  CAN_frame CMFA_3D4 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x3D4,
                        .data = {0x01, 0x2A, 0xD9, 0x00, 0x00, 0x00, 0x80, 0x04}};
  CAN_frame CMFA_3D5 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x3D5,
                        .data = {0xFF, 0xFF, 0x1E, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_42B = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x42B,
                        .data = {0x40, 0x00, 0x0E, 0xB0, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_431 = {.FD = false, .ext_ID = false, .DLC = 2, .ID = 0x431, .data = {0x9C, 0x40}};
  CAN_frame CMFA_442 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 6,
                        .ID = 0x442,
                        .data = {0x60, 0x00, 0x29, 0x00, 0x40, 0x10}};
  CAN_frame CMFA_443 = {.FD = false, .ext_ID = false, .DLC = 3, .ID = 0x443, .data = {0x00, 0x35, 0x00}};
  CAN_frame CMFA_511 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x511,
                        .data = {0x04, 0x3F, 0x1C, 0xCC, 0xC8, 0x1D, 0x34}};
  CAN_frame CMFA_59B = {.FD = false, .ext_ID = false, .DLC = 3, .ID = 0x59B, .data = {0x00, 0x02, 0x00}};
  CAN_frame CMFA_69F = {.FD = false, .ext_ID = false, .DLC = 4, .ID = 0x69F, .data = {0x16, 0x63, 0x22, 0x5F}};
  CAN_frame CMFA_ACK = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x79B,
                        .data = {0x30, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}};

  CAN_frame CMFA_EXT_SESSION = {.FD = false,
                                .ext_ID = false,
                                .DLC = 8,
                                .ID = 0x79B,
                                .data = {0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_WRITE_9281 = {.FD = false,
                               .ext_ID = false,
                               .DLC = 8,
                               .ID = 0x79B,
                               .data = {0x04, 0x2E, 0x92, 0x81, 0x01, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_ROUTINE_B009 = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 8,
                                 .ID = 0x79B,
                                 .data = {0x04, 0x31, 0x01, 0xB0, 0x09, 0x00, 0x00, 0x00}};

  struct MissionEntry {
    int8_t t_end = 0;
    int8_t t_init = 0;
    uint8_t soc_end = 0;
    uint8_t soc_init = 0;
    uint8_t mode_flags = 0;
    uint8_t charge_ah = 0;
    uint8_t discharge_ah = 0;
    uint16_t duration_min = 0;
    uint8_t raw[8] = {0};
    bool valid = false;
  };
  static const uint8_t MISSION_BLOCK_COUNT = 10;
  static const uint8_t MISSION_ENTRIES_PER_BLOCK = 16;
  static const uint8_t MISSION_TOTAL_ENTRIES = MISSION_BLOCK_COUNT * MISSION_ENTRIES_PER_BLOCK;
  static const uint8_t MISSION_RING_SIZE = 20;
  MissionEntry mission_ring[MISSION_RING_SIZE];
  uint8_t mission_ring_head = 0;
  uint8_t mission_ring_count = 0;
  uint16_t mission_blocks_seen_mask = 0;
  MissionEntry mission_initial[MISSION_TOTAL_ENTRIES];
  bool mission_initial_captured = false;
  void handle_mission_pid(uint16_t poll_pid, const uint8_t* data, uint8_t length);
  void append_mission_html(String& content);

  CAN_frame CMFA_3D3_SHUTDOWN = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 8,
                                 .ID = 0x3D3,
                                 .data = {0x44, 0x30, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xE7}};
  CAN_frame CMFA_3D5_SHUTDOWN = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 7,
                                 .ID = 0x3D5,
                                 .data = {0xFF, 0xFF, 0x1E, 0xE0, 0x00, 0x00, 0x00}};
  bool shutdown_3D3_toggle = false;

  enum CarMode : uint8_t {
    CAR_MODE_DEFAULT = 0,
    CAR_MODE_CHARGING = 1,
  };
  CarMode car_mode = CAR_MODE_DEFAULT;
  bool charging_3D3_toggle = false;
  void update_car_state_frames();
};

#endif