#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "../../Software/src/battery/CMFA-EV-BATTERY.h"
#include "../../Software/src/datalayer/datalayer.h"

#include "Arduino.h"

// TX frame capture injected by the emulated CAN layer (see emul/can.cpp).
void clear_transmitted_frames();
const std::vector<CAN_frame>& get_transmitted_frames();

namespace {

constexpr uint16_t PID_PACK_TIME_LIFE = 0x91C1;
constexpr uint16_t PID_ABSOLUTE_TIME_SAVED = 0x9261;

// Exposes the protected PID handler so tests can feed DID values directly.
class TestCmfaEvBattery : public CmfaEvBattery {
 public:
  using CmfaEvBattery::handle_pid;
};

class CmfaEvBatteryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clear_transmitted_frames();
    datalayer = DataLayer();
    now = 1000;
    set_millis64(now);
    battery = new TestCmfaEvBattery();
    battery->setup();
  }

  void TearDown() override { delete battery; }

  // Feed a 24-bit minutes DID value, as the pack sends it (big-endian).
  void feed_time(uint16_t pid, uint32_t minutes) {
    const uint8_t data[3] = {(uint8_t)(minutes >> 16), (uint8_t)(minutes >> 8), (uint8_t)minutes};
    battery->handle_pid(pid, minutes, data, 3);
  }

  // Advance the emulated clock in 100 ms steps, running transmit_can() each step.
  void run_for(unsigned long ms) {
    for (unsigned long end = now + ms; now < end;) {
      now += 100;
      set_millis64(now);
      battery->transmit_can(now);
    }
  }

  std::vector<CAN_frame> frames_3d4() {
    std::vector<CAN_frame> out;
    for (const auto& f : get_transmitted_frames()) {
      if (f.ID == 0x3D4) {
        out.push_back(f);
      }
    }
    return out;
  }

  static uint32_t time_of(const CAN_frame& f) {
    return ((uint32_t)f.data.u8[3] << 16) | ((uint32_t)f.data.u8[4] << 8) | f.data.u8[5];
  }

  uint32_t last_sent_time() {
    auto frames = frames_3d4();
    EXPECT_FALSE(frames.empty());
    return frames.empty() ? 0 : time_of(frames.back());
  }

  unsigned long now = 0;
  TestCmfaEvBattery* battery = nullptr;
};

TEST_F(CmfaEvBatteryTest, NoVehicleTimeUntilBothPackTimesRead) {
  run_for(1000);
  EXPECT_TRUE(frames_3d4().empty());

  feed_time(PID_PACK_TIME_LIFE, 2649903);
  run_for(1000);
  EXPECT_TRUE(frames_3d4().empty());

  feed_time(PID_ABSOLUTE_TIME_SAVED, 2640554);
  run_for(1000);
  EXPECT_FALSE(frames_3d4().empty());
}

TEST_F(CmfaEvBatteryTest, StartsAboveTheLargerPackTime) {
  // As in the car: pack time life is ahead of the saved vehicle time.
  feed_time(PID_PACK_TIME_LIFE, 2649903);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 2640554);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 2649904u);

  // The rest of the frame matches a real car at idle.
  const CAN_frame f = frames_3d4().back();
  EXPECT_EQ(f.DLC, 8);
  EXPECT_EQ(f.data.u8[0], 0x01);
  EXPECT_EQ(f.data.u8[1], 0x2A);
  EXPECT_EQ(f.data.u8[2], 0xD9);
  EXPECT_EQ(f.data.u8[6], 0x80);
  EXPECT_EQ(f.data.u8[7], 0x04);
}

TEST_F(CmfaEvBatteryTest, StartsAboveSavedTimeWhenItIsLarger) {
  // As in UDS_dump_1: the saved vehicle time is ahead of pack time life.
  feed_time(PID_PACK_TIME_LIFE, 1816132);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 1816152);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 1816153u);
}

TEST_F(CmfaEvBatteryTest, CountsUpOncePerMinute) {
  feed_time(PID_PACK_TIME_LIFE, 100);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 50);
  run_for(59000);
  EXPECT_EQ(last_sent_time(), 101u);
  run_for(1100);
  EXPECT_EQ(last_sent_time(), 102u);
  run_for(10 * 60000);
  EXPECT_EQ(last_sent_time(), 112u);
}

TEST_F(CmfaEvBatteryTest, NeverGoesBackwards) {
  feed_time(PID_PACK_TIME_LIFE, 1000);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 1000);
  run_for(200);
  ASSERT_EQ(last_sent_time(), 1001u);

  // A later read with lower values (e.g. after an NVROL reset) is ignored.
  feed_time(PID_PACK_TIME_LIFE, 0);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 0);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 1001u);

  // A pack time ahead of ours makes us jump forwards.
  feed_time(PID_ABSOLUTE_TIME_SAVED, 5000);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 5000u);

  uint32_t previous = 0;
  for (const auto& f : frames_3d4()) {
    EXPECT_GE(time_of(f), previous);
    previous = time_of(f);
  }
}

CAN_frame last_frame_with_id(uint32_t id) {
  CAN_frame found = {};
  for (const auto& f : get_transmitted_frames()) {
    if (f.ID == id) {
      found = f;
    }
  }
  return found;
}

size_t count_frames_with_id(uint32_t id) {
  size_t n = 0;
  for (const auto& f : get_transmitted_frames()) {
    n += (f.ID == id);
  }
  return n;
}

TEST_F(CmfaEvBatteryTest, DefaultCarStateIsUnchanged) {
  datalayer.battery.status.current_dA = 170;
  run_for(1000);
  const CAN_frame f = last_frame_with_id(0x3D3);
  const uint8_t expected[8] = {0x47, 0x30, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xE7};
  for (int i = 0; i < 8; i++) {
    EXPECT_EQ(f.data.u8[i], expected[i]) << "byte " << i;
  }
  EXPECT_EQ(count_frames_with_id(0x3D5), 0u);
  const CAN_frame f135 = last_frame_with_id(0x135);
  EXPECT_EQ(f135.data.u8[3], 0x80);
  EXPECT_EQ(f135.data.u8[4], 0x01);
}

TEST_F(CmfaEvBatteryTest, ChargingCarStateSendsCarLogFrames) {
  battery->initiate_balancing();
  datalayer.battery.status.current_dA = 170;  // 17.0 A charging
  run_for(1000);

  const CAN_frame f = last_frame_with_id(0x3D3);
  EXPECT_EQ(f.data.u8[0], 0x47);
  EXPECT_EQ(f.data.u8[1], 0x1F);
  EXPECT_EQ(f.data.u8[3], 0x02);
  EXPECT_EQ(f.data.u8[5], 0x20);
  EXPECT_EQ(f.data.u8[7], 0xD9);
  // Bytes 4 and 6 are always opposite 0x5D/0xB2, and both values appear.
  bool seen_5d = false, seen_b2 = false;
  for (const auto& fr : get_transmitted_frames()) {
    if (fr.ID != 0x3D3)
      continue;
    EXPECT_TRUE((fr.data.u8[4] == 0x5D && fr.data.u8[6] == 0xB2) || (fr.data.u8[4] == 0xB2 && fr.data.u8[6] == 0x5D));
    seen_5d |= fr.data.u8[4] == 0x5D;
    seen_b2 |= fr.data.u8[4] == 0xB2;
  }
  EXPECT_TRUE(seen_5d && seen_b2);

  ASSERT_GT(count_frames_with_id(0x3D5), 0u);
  EXPECT_EQ(last_frame_with_id(0x3D5).data.u8[3], 0x60);

  // 0x8000 + 17.0 A * 40 = 0x82A8, as the car sends at ~17 A.
  const CAN_frame f135 = last_frame_with_id(0x135);
  EXPECT_EQ(f135.data.u8[3], 0x82);
  EXPECT_EQ(f135.data.u8[4], 0xA8);

  // Discharging reports zero charger current.
  datalayer.battery.status.current_dA = -50;
  run_for(200);  // bytes 3-4 are updated after each send, so allow one extra frame
  EXPECT_EQ(last_frame_with_id(0x135).data.u8[3], 0x80);
  EXPECT_EQ(last_frame_with_id(0x135).data.u8[4], 0x00);

  battery->end_balancing();
  clear_transmitted_frames();
  run_for(1000);
  EXPECT_EQ(last_frame_with_id(0x3D3).data.u8[1], 0x30);
  EXPECT_EQ(count_frames_with_id(0x3D5), 0u);
}

TEST_F(CmfaEvBatteryTest, MissionLogIsDecodedIntoHtml) {
  EXPECT_NE(battery->get_uds_info_html().str().find("Mission log: not read yet"), std::string::npos);

  // Two entries from UDS_dump_1.txt: slot 0 (isolated charge) and slot 1 (driving).
  uint8_t modes[32] = {3, 6};
  uint8_t times[96] = {0x1b, 0xb5, 0xf3, 0x06, 0x43, 0x22};
  uint8_t times_end[96] = {0x1b, 0xb5, 0xf3, 0x06, 0x43, 0x28};
  uint8_t temps[32] = {40, 57};
  battery->handle_pid(0x9270, 0, modes, 32);
  battery->handle_pid(0x9275, 0, times, 96);
  battery->handle_pid(0x9276, 0, times_end, 96);
  battery->handle_pid(0x9271, 0, temps, 32);
  feed_time(PID_PACK_TIME_LIFE, 1816132);

  std::string html = battery->get_uds_info_html().str();
  EXPECT_NE(html.find("3 Isolated charge"), std::string::npos);
  EXPECT_NE(html.find("6 Driving"), std::string::npos);
  EXPECT_NE(html.find("1816051"), std::string::npos);
  EXPECT_NE(html.find("410402"), std::string::npos);
  EXPECT_NE(html.find("410408"), std::string::npos);
  EXPECT_NE(html.find("0d 1h 21m"), std::string::npos);  // 1816132 - 1816051 = 81 min ago
  EXPECT_NE(html.find("17 &rarr;"), std::string::npos);  // 57 - 40
}

TEST_F(CmfaEvBatteryTest, DoesNotChasePackTimeLife) {
  // Car-like pack: pack time life runs ahead of the saved vehicle time.
  feed_time(PID_PACK_TIME_LIFE, 2649903);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 2640554);
  run_for(200);
  ASSERT_EQ(last_sent_time(), 2649904u);

  // If the pack advances 0x91C1 from our 0x3D4 (keeping its offset), it
  // stays ahead of us. We must not chase it, or the time would run away.
  feed_time(PID_PACK_TIME_LIFE, 2649904 + 9349);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 2649904u);
  std::string html = battery->get_uds_info_html().str();
  EXPECT_NE(html.find("0 forward jumps"), std::string::npos);
  EXPECT_EQ(html.find("AHEAD OF REAL TIME"), std::string::npos);
  EXPECT_NE(html.find("Pack time life - vehicle time: 9349m (was -1m)"), std::string::npos);
}

TEST_F(CmfaEvBatteryTest, ForwardJumpsAreReported) {
  feed_time(PID_PACK_TIME_LIFE, 1000);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 1000);
  run_for(200);
  feed_time(PID_ABSOLUTE_TIME_SAVED, 1100);
  run_for(200);
  EXPECT_EQ(last_sent_time(), 1100u);
  std::string html = battery->get_uds_info_html().str();
  EXPECT_NE(html.find("1 forward jumps (99m) - AHEAD OF REAL TIME"), std::string::npos);
}

TEST_F(CmfaEvBatteryTest, PowerLimitSourceToggle) {
  ASSERT_TRUE(battery->supports_toggle_power_limit_source());

  // UDS limits (10 W/bit): 6 kW charge, 40 kW discharge.
  const uint8_t chg[2] = {0x02, 0x58};
  const uint8_t dis[2] = {0x0F, 0xA0};
  battery->handle_pid(0x9018, 600, chg, 2);
  battery->handle_pid(0x900F, 4000, dis, 2);
  // Broadcast limits (500 W/bit): 10 kW charge, 50 kW discharge.
  CAN_frame f3d6 = {
      .FD = false, .ext_ID = false, .DLC = 8, .ID = 0x3D6, .data = {0x11, 0x40, 20, 100, 0x3D, 0x5C, 0x55, 0x3D}};
  battery->handle_incoming_can_frame(f3d6);

  // Broadcast limits by default.
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 10000u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 50000u);

  // First click: UDS limits.
  battery->toggle_power_limit_source();
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 6000u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 40000u);

  battery->toggle_power_limit_source();
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 10000u);
}

TEST_F(CmfaEvBatteryTest, ChargeModeChargesToTrueFull) {
  // 0x127: charge power request 0x0F (4.5 kW), raw SOC 38000 (95.00%).
  CAN_frame f127 = {
      .FD = false, .ext_ID = false, .DLC = 8, .ID = 0x127, .data = {0x0F, 0x87, 0xD0, 0x54, 0x94, 0x70, 0x02, 0x61}};
  battery->handle_incoming_can_frame(f127);
  // 0x3D6: DC charge limit already 0, discharge 50 kW.
  CAN_frame f3d6 = {
      .FD = false, .ext_ID = false, .DLC = 8, .ID = 0x3D6, .data = {0x11, 0x40, 0, 100, 0x3D, 0x5C, 0x55, 0x3D}};
  battery->handle_incoming_can_frame(f3d6);

  // Default: 95% raw is already reported as full, and 0x3D6 stops charging.
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.real_soc, 10000);
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 0u);

  // Charge mode: 100% only at 100.00% raw, and the 0x127 request is the charge limit.
  battery->initiate_balancing();
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.real_soc, (10000 * (9500 - 500)) / (10000 - 500));
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 4500u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 50000u);

  // The pack ends the charge itself.
  f127.data.u8[0] = 0;
  battery->handle_incoming_can_frame(f127);
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 0u);
}

TEST_F(CmfaEvBatteryTest, LimitsAllowAFullPack) {
  // A real car's full pack: 304.5 V at rest, 306.0 V briefly while charging.
  EXPECT_GE(datalayer.battery.info.max_design_voltage_dV, 3060);
  EXPECT_LE(datalayer.battery.info.max_design_voltage_dV, 3100);
  EXPECT_GE(datalayer.battery.info.max_cell_voltage_mV, 4250);
}

// Frames sent in (from, to] ms, in order.
std::vector<CAN_frame> frames_between(const std::vector<std::pair<unsigned long, CAN_frame>>& log, unsigned long from,
                                      unsigned long to) {
  std::vector<CAN_frame> out;
  for (const auto& e : log) {
    if (e.first > from && e.first <= to) {
      out.push_back(e.second);
    }
  }
  return out;
}

class CmfaEvBatterySleepTest : public CmfaEvBatteryTest {
 protected:
  // Run in 10 ms steps, recording when each frame was sent.
  void run_10ms(unsigned long ms) {
    for (unsigned long end = now + ms; now < end;) {
      now += 10;
      set_millis64(now);
      const size_t before = get_transmitted_frames().size();
      battery->transmit_can(now);
      const auto& frames = get_transmitted_frames();
      for (size_t i = before; i < frames.size(); i++) {
        log.push_back({now, frames[i]});
      }
    }
  }
  std::vector<std::pair<unsigned long, CAN_frame>> log;
};

TEST_F(CmfaEvBatterySleepTest, OxEAIsAlwaysZero) {
  battery->reset_BMS();  // 30 s sleep cycle
  run_10ms(40000);
  battery->request_sleep();
  run_10ms(5000);
  battery->request_wake();
  run_10ms(2000);
  size_t n = 0;
  for (const auto& e : log) {
    if (e.second.ID == 0x1EA) {
      EXPECT_EQ(e.second.data.u8[0], 0x00);
      n++;
    }
  }
  EXPECT_GT(n, 0u);
}

TEST_F(CmfaEvBatterySleepTest, SleepButtonShutsDownLikeACarThenGoesSilent) {
  run_10ms(1000);
  const unsigned long t_sleep = now;
  battery->request_sleep();
  EXPECT_TRUE(battery->is_sleep_requested());
  run_10ms(5000);

  // Shutdown phase (~0.9 s): only 0x3D3 (0x44), 0x3D5 (0xE0), 0x135, 0x3D4.
  bool seen_3d3 = false, seen_3d5 = false;
  for (const auto& f : frames_between(log, t_sleep, t_sleep + 900)) {
    EXPECT_TRUE(f.ID == 0x3D3 || f.ID == 0x3D5 || f.ID == 0x135 || f.ID == 0x3D4) << std::hex << f.ID;
    if (f.ID == 0x3D3) {
      EXPECT_EQ(f.data.u8[0], 0x44);
      seen_3d3 = true;
    }
    if (f.ID == 0x3D5) {
      EXPECT_EQ(f.data.u8[3], 0xE0);
      seen_3d5 = true;
    }
  }
  EXPECT_TRUE(seen_3d3 && seen_3d5);
  // Then nothing at all, not even UDS.
  EXPECT_TRUE(frames_between(log, t_sleep + 950, now).empty());

  // Deliberate sleep: battery still counted as alive, inverter gets 0 W.
  EXPECT_EQ(datalayer.battery.status.CAN_battery_still_alive, CAN_STILL_ALIVE);
  battery->update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 0u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 0u);
  EXPECT_NE(battery->get_uds_info_html().str().find("ASLEEP (Sleep button)"), std::string::npos);

  // Wake: normal frames resume straight away.
  const unsigned long t_wake = now;
  battery->request_wake();
  EXPECT_FALSE(battery->is_sleep_requested());
  run_10ms(200);
  bool seen_125 = false, seen_awake_3d3 = false;
  for (const auto& f : frames_between(log, t_wake, now)) {
    seen_125 |= f.ID == 0x125;
    seen_awake_3d3 |= (f.ID == 0x3D3 && f.data.u8[0] == 0x47);
  }
  EXPECT_TRUE(seen_125 && seen_awake_3d3);
  EXPECT_NE(battery->get_uds_info_html().str().find("Sleep: AWAKE"), std::string::npos);
}

TEST_F(CmfaEvBatterySleepTest, SleepCycleIsSilentFor30Seconds) {
  run_10ms(500);
  battery->reset_BMS();
  run_10ms(10000);  // the sequence starts once the PID request in flight (unanswered here) times out
  // Find the first shutdown frame.
  unsigned long t_shutdown = 0;
  for (const auto& e : log) {
    if (e.second.ID == 0x3D3 && e.second.data.u8[0] == 0x44) {
      t_shutdown = e.first;
      break;
    }
  }
  ASSERT_NE(t_shutdown, 0u);
  run_10ms(30000);
  // Silent from the end of the shutdown frames for the full 30 s.
  EXPECT_TRUE(frames_between(log, t_shutdown + 950, t_shutdown + 30850).empty());
  // Awake again afterwards.
  bool seen_125 = false;
  for (const auto& f : frames_between(log, t_shutdown + 31000, now)) {
    seen_125 |= f.ID == 0x125;
  }
  EXPECT_TRUE(seen_125);
}

}  // namespace
