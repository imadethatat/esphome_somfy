// Host-side regression tests for the Somfy RTS cover.
//
// These compile the REAL production sources (somfy_rts.cpp, somfy_hub_rts.cpp,
// somfy_time_based_cover.cpp) against minimal ESPHome stubs in ./stubs, so the
// behaviour under test is the code that ships — not a re-implementation.
//
// Scope: the RX state-sync feature, i.e. "pressing a button on the physical
// remote keeps the Home Assistant entity in sync". That path has no coverage in
// the ESPHome compile matrix and can therefore break silently — a successful
// firmware build proves nothing about it.
//
// The 433 MHz demodulator (SomfyRtsHub::decode_frame_) is deliberately NOT
// driven from synthetic timings here; it is exercised by the real radio. Below
// the cover level this only verifies that a frame decoded by the hub reaches
// the covers registered during setup().

#include "../../components/somfy/somfy_rts.h"

#include "esphome/components/button/button.h"
#include "esphome/components/logger/logger.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/application.h"

#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Stub runtime
// ---------------------------------------------------------------------------

namespace esphome {

static uint32_t g_millis = 0;
uint32_t millis() { return g_millis; }

Application App;  // NOLINT

namespace logger {
static Logger g_logger;
Logger *global_logger = &g_logger;
}  // namespace logger

}  // namespace esphome

// In-memory replacement for the ESP32 NVS-backed rolling code counter. The real
// implementation is exercised on-device; here codes only need to advance.
static uint16_t g_rolling_code = 1;
NVSRollingCodeStorage::NVSRollingCodeStorage(const char *name, const char *key, uint16_t initial_code)
    : name_(name), key_(key), initial_code_(initial_code) {}
NVSRollingCodeStorage::~NVSRollingCodeStorage() = default;
uint16_t NVSRollingCodeStorage::nextCode() { return g_rolling_code++; }

// ---------------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_passed = 0;

static void check(bool ok, const char *name) {
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s\n", name);
  }
}

static void check_close(float actual, float expected, float tol, const char *name) {
  const bool ok = std::fabs(actual - expected) <= tol;
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s (got %.4f, want %.4f +/- %.4f)\n", name, actual, expected, tol);
  }
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

using namespace esphome;

namespace {

constexpr uint32_t REMOTE_CODE = 0x123456;
constexpr uint32_t FOREIGN_CODE = 0xABCDEF;
constexpr uint32_t TRAVEL_MS = 10000;

/// Exposes the protected surface the tests need to drive and observe.
class TestCover : public somfy::SomfyCover {
 public:
  using SomfyCover::build_long_frame_;
  using SomfyCover::close;
  using SomfyCover::control;
  using SomfyCover::on_rts_frame_;
  using SomfyCover::open;
  using SomfyCover::rx_sync_;
  using SomfyCover::stop;

  bool rx_active() const { return this->rx_sync_.active(); }
};

somfy::RtsDecodedFrame make_frame(uint32_t remote_code, somfy::RtsCommand command) {
  somfy::RtsDecodedFrame frame{};
  frame.remote_code = remote_code;
  frame.command = command;
  frame.rolling_code = 0x0042;
  return frame;
}

/// Turn a transmitted buffer into what the receiver reports off the air.
///
/// The encoder emits one entry per Manchester half-symbol, so two consecutive
/// half-symbols at the same level appear as two entries; on the air they are a
/// single pulse of twice the length. Feeding the raw TX buffer to the decoder
/// therefore does not exercise the demodulator, it just yields an all-zero
/// frame that happens to satisfy the checksum.
remote_base::RawTimings as_received(const remote_base::RawTimings &transmitted) {
  remote_base::RawTimings air;
  for (const int32_t value : transmitted) {
    if (!air.empty() && ((air.back() < 0) == (value < 0)))
      air.back() += value;
    else
      air.push_back(value);
  }
  return air;
}

struct Rig {
  remote_transmitter::RemoteTransmitterComponent tx;
  remote_receiver::RemoteReceiverComponent rx;
  button::Button prog;
  text_sensor::TextSensor detected;
  somfy::SomfyRtsHub hub;
  TestCover cover;

  /// @param with_receiver false models a TX-only hub (RX is optional for RTS).
  explicit Rig(bool with_receiver = true, uint8_t tilt_steps = 0) {
    g_millis = 1000;
    g_rolling_code = 1;

    this->hub.set_remote_transmitter(&this->tx);
    if (with_receiver)
      this->hub.set_remote_receiver(&this->rx);
    this->hub.setup();

    this->cover.set_hub(&this->hub);
    this->cover.set_prog_button(&this->prog);
    this->cover.set_remote_code(REMOTE_CODE);
    this->cover.set_storage_namespace("somfy");
    this->cover.set_storage_key("test");
    this->cover.set_repeat_count(2);
    this->cover.set_tilt_steps(tilt_steps);
    this->cover.set_open_duration(TRAVEL_MS);
    this->cover.set_close_duration(TRAVEL_MS);
    this->cover.set_log_text_sensor(&this->detected);
    this->cover.add_receive_remote_code(REMOTE_CODE);
    this->cover.setup();
  }

  /// Deliver a frame as if the hub had just decoded it off the air.
  void receive(uint32_t remote_code, somfy::RtsCommand command) {
    this->cover.on_rts_frame_(make_frame(remote_code, command));
  }

  void receive(uint32_t remote_code, somfy::RtsCommand command, uint8_t steps) {
    auto frame = make_frame(remote_code, command);
    frame.bit_length = 80;
    frame.step_size = steps;
    this->cover.on_rts_frame_(frame);
  }

  /// Advance the simulated clock, running the cover's loop at ~50 Hz.
  void advance(uint32_t ms) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 20) {
      g_millis += 20;
      this->cover.loop();
    }
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// RTS transmit is mandatory; reception is an optional add-on.
static void test_tx_mandatory_rx_optional() {
  printf("TX mandatory / RX optional\n");

  Rig tx_only(/*with_receiver=*/false);
  check(tx_only.rx.listeners.empty(), "TX-only hub registers no RX listener");

  tx_only.cover.open();
  check(tx_only.tx.transmit_count == 1, "open() transmits a frame");
  check(!tx_only.tx.last_data.get_data().empty(), "transmitted frame carries raw timings");

  tx_only.cover.close();
  check(tx_only.tx.transmit_count == 2, "close() transmits a frame");

  tx_only.cover.stop();
  check(tx_only.tx.transmit_count == 3, "stop() transmits a frame");

  Rig with_rx;
  check(with_rx.rx.listeners.size() == 1, "hub with a receiver registers exactly one RX listener");
}

/// setup() must register the cover on the hub, otherwise decoded frames never
/// reach it. Doubles as the encoder/decoder round trip: a frame this component
/// transmits must come back out of its own demodulator unchanged.
static void test_setup_registers_cover_on_hub() {
  printf("Hub -> cover wiring\n");

  Rig rig;
  rig.cover.open();
  g_millis += 1000;

  const int before = rig.detected.publish_count;
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.publish_count > before, "a frame decoded by the hub reaches the cover registered in setup()");
  check(rig.detected.last_state.find("123456") != std::string::npos, "the transmitted remote code survives the round trip");
  check(rig.detected.last_state.find("UP") != std::string::npos, "the transmitted command survives the round trip");
}

/// The regression guard: a frame from an allow-listed remote must drive the HA
/// entity through an OPENING animation and settle at the open end stop.
static void test_rx_frame_syncs_ha_state() {
  printf("RX state sync (physical remote -> HA entity)\n");

  Rig rig;
  rig.cover.position = 0.5f;
  const int publishes_before = rig.cover.publish_count;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);

  check(rig.cover.rx_active(), "UP frame starts the RX animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_OPENING, "entity reports OPENING");
  check(rig.cover.publish_count > publishes_before, "state is published immediately");

  // Half open with a 10 s full travel => 5 s of remaining travel.
  rig.advance(2500);
  check_close(rig.cover.position, 0.75f, 0.05f, "position advances while the motor runs");
  check(rig.cover.rx_active(), "animation still running mid-travel");

  rig.advance(3000);
  check_close(rig.cover.position, 1.0f, 0.001f, "position lands on the open end stop");
  check(!rig.cover.rx_active(), "animation finishes");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity returns to IDLE");
  check_close(rig.cover.last_published_position, 1.0f, 0.001f, "final position is published to HA");
}

/// Closing mirrors opening, and travel time scales with the remaining distance.
static void test_rx_close_from_open() {
  printf("RX close\n");

  Rig rig;
  rig.cover.position = 1.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Down);
  check(rig.cover.current_operation == cover::COVER_OPERATION_CLOSING, "DOWN frame reports CLOSING");

  rig.advance(5000);
  check_close(rig.cover.position, 0.5f, 0.05f, "half-way closed after half the full travel");
  check(rig.cover.rx_active(), "still closing");

  rig.advance(5200);
  check_close(rig.cover.position, 0.0f, 0.001f, "position lands on the closed end stop");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity returns to IDLE");
}

/// A MY frame from the remote must halt the animation where it is.
static void test_rx_stop_halts_animation() {
  printf("RX stop\n");

  Rig rig;
  rig.cover.position = 0.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  check(rig.cover.rx_active(), "UP frame starts the RX animation");

  rig.advance(3000);
  const float mid = rig.cover.position;
  check(mid > 0.1f && mid < 0.9f, "cover is part-way open");

  rig.receive(REMOTE_CODE, somfy::RtsCommand::My);
  check(!rig.cover.rx_active(), "MY frame stops the RX animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity reports IDLE after stop");

  rig.advance(3000);
  check_close(rig.cover.position, mid, 0.05f, "position holds after stop");
}

/// Position updates must be throttled so a moving cover does not flood the API.
static void test_rx_publishes_are_throttled() {
  printf("RX publish throttling\n");

  Rig rig;
  rig.cover.position = 0.0f;
  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);

  const int after_start = rig.cover.publish_count;
  rig.advance(5000);  // 250 loop iterations at 50 Hz
  const int during = rig.cover.publish_count - after_start;

  check(during > 0, "position is published while moving");
  check(during <= 30, "publishes are throttled well below the loop rate");
}

/// Frames from remotes outside allowed_remotes must not move the entity, but
/// must still surface on the discovery text sensor so they can be paired.
static void test_foreign_remote_reported_but_ignored() {
  printf("Foreign remote handling\n");

  Rig rig;
  rig.cover.position = 0.5f;
  const int publishes_before = rig.cover.publish_count;

  rig.receive(FOREIGN_CODE, somfy::RtsCommand::Up);

  check(rig.detected.publish_count == 1, "foreign frame is published for discovery");
  check(rig.detected.last_state.find("ABCDEF") != std::string::npos, "discovery text carries the remote code");
  check(rig.detected.last_state.find("UP") != std::string::npos, "discovery text names the command");
  check(!rig.cover.rx_active(), "foreign frame does not start an animation");
  check(rig.cover.publish_count == publishes_before, "foreign frame does not touch entity state");
  check_close(rig.cover.position, 0.5f, 0.001f, "position unchanged by a foreign frame");
}

/// The RTS cover advertises itself as a positionable, assumed-state cover.
static void test_traits() {
  printf("Cover traits\n");

  Rig rig;
  auto traits = rig.cover.get_traits();
  check(!traits.get_supports_tilt(), "tilt is not advertised");
  check(traits.get_is_assumed_state(), "assumed state is advertised");
}

static std::array<uint8_t, 10> make_80_bit_frame(somfy::RtsCommand command, uint16_t rolling,
                                                 uint8_t step_size = 1) {
  std::array<uint8_t, 10> frame{};
  const uint8_t base = static_cast<uint16_t>(command) & 0x0F;
  frame[0] = 0xB3;
  frame[1] = base << 4;
  frame[2] = rolling >> 8;
  frame[3] = rolling;
  frame[4] = static_cast<uint8_t>(REMOTE_CODE >> 16);
  frame[5] = static_cast<uint8_t>(REMOTE_CODE >> 8);
  frame[6] = static_cast<uint8_t>(REMOTE_CODE);

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++)
    checksum ^= frame[i] ^ (frame[i] >> 4);
  frame[1] |= checksum & 0x0F;

  if (base == 0x0B) {
    frame[7] = 0x84;
    frame[8] = static_cast<uint8_t>(0x30 | ((step_size & 0x70) >> 4));
    if (command == somfy::RtsCommand::StepUp)
      frame[8] |= 0x08;
    frame[9] = static_cast<uint8_t>((step_size & 0x0F) << 4);
  } else if (command == somfy::RtsCommand::Up) {
    frame[7] = 0xC4;
    frame[8] = 0x20;
  } else if (command == somfy::RtsCommand::Down) {
    frame[7] = 0xC4;
    frame[8] = 0x2C;
    frame[9] = 0x80;
  } else {
    frame[7] = 0xC4;
    frame[9] = 0x10;
  }
  frame[9] |= static_cast<uint8_t>((frame[7] >> 4) ^ (frame[8] >> 4) ^ (frame[9] >> 4) ^
                                   (frame[7] & 0x0F) ^ (frame[8] & 0x0F));

  for (uint8_t i = 1; i < 7; i++)
    frame[i] ^= frame[i - 1];
  return frame;
}

static void test_80_bit_decode_and_tilt() {
  printf("80-bit RX commands and Venetian tilt\n");
  Rig rig(true, 10);
  rig.cover.tilt = 0.5f;

  struct Vector { somfy::RtsCommand command; const char *name; } vectors[] = {
      {somfy::RtsCommand::Up, "UP"}, {somfy::RtsCommand::Down, "DOWN"}, {somfy::RtsCommand::My, "MY"}};
  uint16_t rolling = 0x100;
  for (const auto &vector : vectors) {
    rig.hub.send_frame(make_80_bit_frame(vector.command, rolling++), 0);
    rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
    check(rig.detected.last_state.find(vector.name) != std::string::npos, "80-bit standard command decodes");
  }

  rig.hub.send_frame(make_80_bit_frame(somfy::RtsCommand::StepUp, rolling++, 1), 0);
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.last_state.find("STEP_UP") != std::string::npos, "80-bit STEP_UP decodes");
  check_close(rig.cover.tilt, 0.6f, 0.001f, "STEP_UP changes tilt but not lift position");

  rig.hub.send_frame(make_80_bit_frame(somfy::RtsCommand::StepDown, rolling++, 1), 0);
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.last_state.find("STEP_DOWN") != std::string::npos, "80-bit STEP_DOWN decodes");
  check_close(rig.cover.tilt, 0.5f, 0.001f, "STEP_DOWN restores tilt estimate");
  check(!rig.cover.rx_active(), "step commands never start lift animation");
}

static void test_captured_telis_mod_var_frames() {
  printf("Captured Telis 4 Mod/Var vectors\n");
  Rig rig(true, 10);
  // The remote address in these captured vectors was replaced with 0x112233;
  // command, rolling-code, and extension bytes retain the observed values.
  const std::array<uint8_t, 10> wheel_up_air{{0xB3, 0x05, 0x06, 0xFF, 0xEE, 0xCC, 0xFF, 0x84, 0x30, 0x1E}};
  const std::array<uint8_t, 10> wheel_down_air{{0xB4, 0x06, 0x05, 0xFF, 0xEE, 0xCC, 0xFF, 0x84, 0x38, 0x16}};

  rig.hub.send_frame(wheel_up_air, 0);
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.last_state.find("112233 STEP_DOWN") != std::string::npos,
        "captured wheel-up frame is protocol STEP_DOWN");

  rig.hub.send_frame(wheel_down_air, 0);
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.last_state.find("112233 STEP_UP") != std::string::npos,
        "captured wheel-down frame is protocol STEP_UP");
}

static void test_ha_tilt_control() {
  printf("Home Assistant tilt control\n");
  Rig rig(true, 10);
  rig.cover.tilt = 0.2f;
  const uint16_t rolling_before = g_rolling_code;
  cover::CoverCall call;
  call.set_tilt(0.7f);
  rig.cover.control(call);
  check(rig.tx.transmit_count == 1, "one 80-bit frame is sent for a multi-step tilt target");
  check(g_rolling_code == rolling_before + 1, "one rolling code is consumed per tilt target");
  check_close(rig.cover.tilt, 0.7f, 0.001f, "tilt estimate reaches the quantized HA target");
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(rig.detected.last_state.find("STEP_UP") != std::string::npos, "HA tilt TX round-trips as STEP_UP");
}

static void test_tilt_tx_failure_does_not_publish_false_state() {
  printf("Tilt TX failure\n");
  Rig rig(true, 10);
  rig.cover.tilt = 0.2f;
  g_rolling_code = 0;  // The production storage uses zero as its failure sentinel.
  cover::CoverCall call;
  call.set_tilt(0.7f);
  rig.cover.control(call);
  check(rig.tx.transmit_count == 0, "failed rolling-code allocation sends no frame");
  check_close(rig.cover.tilt, 0.2f, 0.001f, "failed TX leaves the reported tilt unchanged");
}

static void test_zero_step_rx_is_ignored() {
  printf("Invalid zero-magnitude tilt RX\n");
  Rig rig(true, 10);
  rig.cover.tilt = 0.5f;
  const int publishes_before = rig.cover.publish_count;
  rig.receive(REMOTE_CODE, somfy::RtsCommand::StepUp, 0);
  check_close(rig.cover.tilt, 0.5f, 0.001f, "zero magnitude does not move the tilt estimate");
  check(rig.cover.publish_count == publishes_before, "zero magnitude does not publish a false state");
}

static void test_venetian_tx_golden_vectors() {
  printf("80-bit TX golden vectors\n");
  Rig rig(true, 12);
  struct Vector {
    somfy::RtsCommand command;
    std::array<uint8_t, 10> air;
    const char *name;
  } vectors[] = {
      {somfy::RtsCommand::Up,   {{0xA7, 0x8E, 0x8E, 0x8F, 0x9D, 0xA9, 0xFF, 0xC4, 0x20, 0x0A}}, "UP"},
      {somfy::RtsCommand::Down, {{0xA7, 0xE8, 0xE8, 0xE9, 0xFB, 0xCF, 0x99, 0xC4, 0x2C, 0x8E}}, "DOWN"},
      {somfy::RtsCommand::My,   {{0xA7, 0xBD, 0xBD, 0xBC, 0xAE, 0x9A, 0xCC, 0xC4, 0x00, 0x19}}, "MY"},
  };
  for (const auto &vector : vectors) {
    std::array<uint8_t, 10> actual{};
    rig.cover.build_long_frame_(actual, vector.command, 1);
    check(actual == vector.air, vector.name);
  }

  // Repeat extension ordinals are part of the 80-bit wire format. Verify the
  // raw Manchester payloads rather than decoding them with our own receiver.
  const std::array<uint8_t, 10> first_repeat{{0xA7, 0x8E, 0x8E, 0x8F, 0x9D, 0xA9, 0xFF, 0xC8, 0x20, 0x06}};
  const std::array<uint8_t, 10> second_repeat{{0xA7, 0x8E, 0x8E, 0x8F, 0x9D, 0xA9, 0xFF, 0xCC, 0x20, 0x02}};
  auto manchester = [](const std::array<uint8_t, 10> &bytes) {
    remote_base::RawTimings data;
    for (uint8_t bit = 0; bit < 80; bit++) {
      const bool one = ((bytes[bit / 8] >> (7 - bit % 8)) & 1) != 0;
      data.push_back(one ? -640 : 640);
      data.push_back(one ? 640 : -640);
    }
    return data;
  };
  rig.hub.send_frame(vectors[0].air, 2, true);
  const auto &raw = rig.tx.last_data.get_data();
  const auto repeat1 = manchester(first_repeat);
  const auto repeat2 = manchester(second_repeat);
  // First long frame: 26 sync entries + 160 data + gap. Repeat sync is 14 entries.
  check(std::equal(repeat1.begin(), repeat1.end(), raw.begin() + 201),
        "first repeat carries the C8 extension and independent checksum");
  check(std::equal(repeat2.begin(), repeat2.end(), raw.begin() + 376),
        "second repeat carries the CC extension and independent checksum");
}

static void test_venetian_lift_uses_80_bit_frames() {
  printf("Venetian lift TX framing\n");
  Rig roller;
  roller.cover.open();
  const size_t short_size = roller.tx.last_data.get_data().size();

  Rig blind(true, 12);
  struct Action { void (TestCover::*send)(); const char *name; } actions[] = {
      {&TestCover::open, "UP"}, {&TestCover::close, "DOWN"}, {&TestCover::stop, "MY"}};
  for (const auto &action : actions) {
    (blind.cover.*action.send)();
    check(blind.tx.last_data.get_data().size() > short_size, "Venetian standard command uses longer 80-bit TX");
    blind.hub.on_receive(remote_base::RemoteReceiveData(as_received(blind.tx.last_data.get_data())));
    check(blind.detected.last_state.find(action.name) != std::string::npos,
          "Venetian 80-bit lift command round-trips through decoder");
  }
}

/// A press makes the remote send the same frame several times, ~143 ms apart,
/// and the receiver hands us each copy. Repeats must collapse, but the next
/// press must be acted on immediately however fast it follows — a time-based
/// mute cannot do both, because its dead time is longer than the repeat period.
static void test_repeat_burst_collapses_but_new_press_gets_through() {
  printf("RX burst suppression\n");

  Rig rig;
  rig.cover.position = 0.0f;

  // Borrow the encoder to produce a real on-air burst for an allow-listed remote.
  rig.cover.open();
  const auto up_burst = as_received(rig.tx.last_data.get_data());

  rig.hub.on_receive(remote_base::RemoteReceiveData(up_burst));
  check(rig.cover.rx_active(), "first copy of the burst starts the animation");

  const int cover_publishes = rig.cover.publish_count;
  const int discovery_publishes = rig.detected.publish_count;

  for (int repeat = 0; repeat < 4; repeat++) {
    g_millis += 143;  // measured RTS repeat-frame period
    rig.hub.on_receive(remote_base::RemoteReceiveData(up_burst));
  }
  check(rig.cover.publish_count == cover_publishes, "repeats do not restart the animation");
  check(rig.detected.publish_count == discovery_publishes, "repeats are not re-reported for discovery");

  // Same remote, next rolling code, well inside the old 150 ms dead time.
  rig.cover.stop();
  const auto my_burst = as_received(rig.tx.last_data.get_data());
  g_millis += 20;
  rig.hub.on_receive(remote_base::RemoteReceiveData(my_burst));

  check(!rig.cover.rx_active(), "a following press is acted on, not swallowed");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "MY from the remote stops the entity");
}

/// While the animation runs it owns current_operation, so a Home Assistant
/// command has to cancel it — otherwise the entity keeps travelling to the end
/// stop and drifts away from what the motor is actually doing.
static void test_ha_command_cancels_remote_animation() {
  printf("HA command vs. physical remote\n");

  Rig rig;
  rig.cover.position = 0.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  rig.advance(2000);
  check(rig.cover.rx_active(), "remote animation is running");

  const float at_stop = rig.cover.position;
  cover::CoverCall call;
  call.set_command_stop();
  rig.cover.control(call);

  check(!rig.cover.rx_active(), "HA command cancels the remote animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity reports IDLE");

  rig.advance(4000);
  check_close(rig.cover.position, at_stop, 0.01f, "position no longer drifts to the end stop");
}

int main() {
  printf("Somfy RTS host tests\n\n");

  test_tx_mandatory_rx_optional();
  printf("\n");
  test_setup_registers_cover_on_hub();
  printf("\n");
  test_rx_frame_syncs_ha_state();
  printf("\n");
  test_rx_close_from_open();
  printf("\n");
  test_rx_stop_halts_animation();
  printf("\n");
  test_rx_publishes_are_throttled();
  printf("\n");
  test_foreign_remote_reported_but_ignored();
  printf("\n");
  test_traits();
  printf("\n");
  test_80_bit_decode_and_tilt();
  printf("\n");
  test_captured_telis_mod_var_frames();
  printf("\n");
  test_ha_tilt_control();
  printf("\n");
  test_tilt_tx_failure_does_not_publish_false_state();
  printf("\n");
  test_zero_step_rx_is_ignored();
  printf("\n");
  test_venetian_tx_golden_vectors();
  printf("\n");
  test_venetian_lift_uses_80_bit_frames();
  printf("\n");
  test_repeat_burst_collapses_but_new_press_gets_through();
  printf("\n");
  test_ha_command_cancels_remote_animation();

  printf("\n%d/%d checks passed\n", g_passed, g_checks);
  return g_passed == g_checks ? 0 : 1;
}
