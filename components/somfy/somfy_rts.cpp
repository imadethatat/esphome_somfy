#include "somfy_rts.h"

#ifdef USE_SOMFY_RTS

#include "esphome/core/log.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/hal.h"
#endif
#include <cinttypes>
#include <cmath>
#include <cstdlib>

namespace esphome {
namespace somfy {

static const char *TAG = "somfy.rts";

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

#ifdef USE_SOMFY_COVER_RX

void SomfyCover::on_rts_frame_(const RtsDecodedFrame &frame) {
  // Publish to the discovery text sensor regardless of the allow-list, so an
  // unknown remote can be learned.
  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16,
             frame.remote_code, rts_command_name(frame.command), frame.rolling_code);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!this->is_allowed_remote_(frame.remote_code))
    return;

  switch (frame.command) {
    case RtsCommand::Up:
    case RtsCommand::MyUp:
      this->start_rx_sync_(cover::COVER_OPERATION_OPENING);
      break;

    case RtsCommand::Down:
    case RtsCommand::MyDown:
      this->start_rx_sync_(cover::COVER_OPERATION_CLOSING);
      break;

    case RtsCommand::My:
    case RtsCommand::UpDown:
      this->stop_rx_sync_();
      break;

    case RtsCommand::StepUp:
    case RtsCommand::StepDown:
      this->apply_rx_tilt_(frame.command, frame.step_size);
      break;

    default:
      break;
  }
}

// The motor reports nothing back, so the position is dead-reckoned from the
// configured travel durations for as long as we believe it is moving.
void SomfyCover::start_rx_sync_(cover::CoverOperation op) {
  this->rx_sync_.start(op == cover::COVER_OPERATION_OPENING, this->position, millis());
  this->current_operation = op;
  this->publish_state();
}

void SomfyCover::stop_rx_sync_() {
  this->rx_sync_.stop();
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

bool SomfyCover::is_allowed_remote_(uint32_t code) const {
  return this->receive_remote_codes_.empty() ||
         std::binary_search(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
}

#endif  // USE_SOMFY_COVER_RX

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void SomfyCover::setup() {
  this->storage_ = std::make_unique<NVSRollingCodeStorage>(
      this->storage_namespace_, this->storage_key_, this->initial_rolling_code_);

#ifdef USE_SOMFY_COVER_RX
  // Register RX callback on hub (if hub has a receiver)
  this->hub_->register_rx_callback([this](const RtsDecodedFrame &frame) {
    this->on_rts_frame_(frame);
  });
#endif

  // Wire up time-based cover triggers
  automationTriggerUp_ = std::make_unique<Automation<>>(this->get_open_trigger());
  actionTriggerUp_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->open(); });
  automationTriggerUp_->add_action(actionTriggerUp_.get());

  automationTriggerDown_ = std::make_unique<Automation<>>(this->get_close_trigger());
  actionTriggerDown_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->close(); });
  automationTriggerDown_->add_action(actionTriggerDown_.get());

  automationTriggerStop_ = std::make_unique<Automation<>>(this->get_stop_trigger());
  actionTriggerStop_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->stop(); });
  automationTriggerStop_->add_action(actionTriggerStop_.get());

  this->cover_prog_button_->add_on_press_callback([=, this] { return this->program(); });

  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  SomfyTimeBasedCover::setup();
}

// ---------------------------------------------------------------------------
// Loop (RX sync animation)
// ---------------------------------------------------------------------------

void SomfyCover::loop() {
#ifdef USE_SOMFY_COVER_RX
  if (this->rx_sync_.active()) {
    const uint32_t full_duration_ms = this->rx_sync_.opening() ? this->open_duration_ : this->close_duration_;
    const RxSyncUpdate update = this->rx_sync_.update(millis(), full_duration_ms);

    this->position = update.position;
    if (update.finished) {
      this->stop_rx_sync_();
    } else if (update.publish) {
      this->publish_state();
    }
    return;
  }
#endif  // USE_SOMFY_COVER_RX

  SomfyTimeBasedCover::loop();
}

void SomfyCover::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy RTS cover");
  if (this->tilt_steps_ > 0) {
    ESP_LOGCONFIG(TAG, "  Venetian tilt: %u steps, protocol direction %s",
                  this->tilt_steps_, this->tilt_inverted_ ? "inverted" : "normal");
  }
}

cover::CoverTraits SomfyCover::get_traits() {
  auto traits = SomfyTimeBasedCover::get_traits();
  traits.set_supports_tilt(this->tilt_steps_ > 0);
  return traits;
}

void SomfyCover::control(const cover::CoverCall &call) {
#ifdef USE_SOMFY_COVER_RX
  // A command from Home Assistant supersedes a physical-remote animation. The
  // animator owns current_operation while it runs, so hand a clean IDLE state to
  // the base machine rather than letting it resume from a stale travel clock.
  if (this->rx_sync_.active()) {
    this->rx_sync_.stop();
    this->current_operation = cover::COVER_OPERATION_IDLE;
  }
#endif

  const auto requested_tilt = call.get_tilt();
  if (requested_tilt.has_value() && this->tilt_steps_ > 0)
    this->set_tilt_target_(*requested_tilt);

  SomfyTimeBasedCover::control(call);
}

// ---------------------------------------------------------------------------
// TX: frame building + send via hub
// ---------------------------------------------------------------------------

void SomfyCover::log_and_send_(const char *label, RtsCommand cmd) {
  char object_id[128];
  this->get_object_id_to(object_id);
  ESP_LOGD(TAG, "%s %s", label, object_id);
  this->send_command(cmd);
}

void SomfyCover::open()    { log_and_send_("OPEN", RtsCommand::Up);    }
void SomfyCover::close()   { log_and_send_("CLOSE", RtsCommand::Down); }
void SomfyCover::stop()    { log_and_send_("STOP", RtsCommand::My);    }
void SomfyCover::program() { log_and_send_("PROG", RtsCommand::Prog);  }

void SomfyCover::build_frame(std::array<uint8_t, 7> &bytes, RtsCommand command, uint16_t code) {
  bytes.fill(0x00);

  const uint8_t button = static_cast<uint16_t>(command) & 0x0F;
  bytes[0] = 0xA7;
  bytes[1] = button << 4;
  bytes[2] = code >> 8;
  bytes[3] = code;
  bytes[4] = this->remote_code_ >> 16;
  bytes[5] = this->remote_code_ >> 8;
  bytes[6] = this->remote_code_;

  // Checksum: XOR of all nibbles
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    checksum = checksum ^ bytes[i] ^ (bytes[i] >> 4);
  }
  checksum &= 0x0F;
  bytes[1] |= checksum;

  // Obfuscation: XOR chain
  for (uint8_t i = 1; i < 7; i++) {
    bytes[i] ^= bytes[i - 1];
  }
}

void SomfyCover::build_step_frame(std::array<uint8_t, 10> &bytes, RtsCommand command, uint8_t steps,
                                  uint16_t code) {
  std::array<uint8_t, 7> short_frame;
  this->build_frame(short_frame, RtsCommand::StepDown, code);
  std::copy(short_frame.begin(), short_frame.end(), bytes.begin());

  steps = clamp<uint8_t>(steps, 1, 0x7F);
  bytes[7] = 0x84;
  bytes[8] = static_cast<uint8_t>(0x30 | ((steps & 0x70) >> 4));
  if (command == RtsCommand::StepUp)
    bytes[8] |= 0x08;
  bytes[9] = static_cast<uint8_t>((steps & 0x0F) << 4);
  bytes[9] |= static_cast<uint8_t>((bytes[7] >> 4) ^ (bytes[8] >> 4) ^ (bytes[9] >> 4) ^
                                   (bytes[7] & 0x0F) ^ (bytes[8] & 0x0F));
}

void SomfyCover::build_long_frame_(std::array<uint8_t, 10> &bytes, RtsCommand command, uint16_t code) {
  std::array<uint8_t, 7> short_frame;
  this->build_frame(short_frame, command, code);
  std::copy(short_frame.begin(), short_frame.end(), bytes.begin());

  bytes[7] = 0xC4;
  if (command == RtsCommand::Up) {
    bytes[8] = 0x20;
    bytes[9] = 0x00;
  } else if (command == RtsCommand::Down) {
    bytes[8] = 0x2C;
    bytes[9] = 0x80;
  } else {
    bytes[8] = 0x00;
    bytes[9] = 0x10;
  }
  bytes[9] |= static_cast<uint8_t>((bytes[7] >> 4) ^ (bytes[8] >> 4) ^ (bytes[9] >> 4) ^
                                   (bytes[7] & 0x0F) ^ (bytes[8] & 0x0F));
}

void SomfyCover::send_command(RtsCommand command) {
  const uint16_t rolling_code = this->storage_->nextCode();
  if (rolling_code == 0) {
    ESP_LOGE(TAG, "TX aborted: rolling-code storage unavailable or exhausted");
    return;
  }
  if (this->tilt_steps_ > 0) {
    std::array<uint8_t, 10> frame;
    this->build_long_frame_(frame, command, rolling_code);
    ESP_LOGD(TAG, "80-bit lift TX: command=0x%X rolling=0x%04" PRIX16,
             static_cast<unsigned>(command), rolling_code);
    this->hub_->send_frame(frame, static_cast<uint8_t>(this->repeat_count_), true);
  } else {
    std::array<uint8_t, 7> frame;
    build_frame(frame, command, rolling_code);
    this->hub_->send_frame(frame, static_cast<uint8_t>(this->repeat_count_));
  }
}

bool SomfyCover::send_step_command_(RtsCommand command, uint8_t steps) {
  const uint16_t rolling_code = this->storage_->nextCode();
  if (rolling_code == 0) {
    ESP_LOGE(TAG, "Tilt TX aborted: rolling-code storage unavailable or exhausted");
    return false;
  }
  std::array<uint8_t, 10> frame;
  this->build_step_frame(frame, command, steps, rolling_code);
  ESP_LOGD(TAG, "Tilt TX: %s, %u step(s), rolling=0x%04" PRIX16,
           command == RtsCommand::StepUp ? "STEP_UP" : "STEP_DOWN", steps, rolling_code);
  this->hub_->send_frame(frame, static_cast<uint8_t>(this->repeat_count_));
  return true;
}

void SomfyCover::set_tilt_target_(float target) {
  target = clamp(target, 0.0f, 1.0f);
  const int current_step = static_cast<int>(std::lround(this->tilt * this->tilt_steps_));
  const int target_step = static_cast<int>(std::lround(target * this->tilt_steps_));
  const int delta = target_step - current_step;
  if (delta == 0) {
    this->tilt = target_step / static_cast<float>(this->tilt_steps_);
    this->publish_state();
    return;
  }

  const bool logical_up = delta > 0;
  const bool protocol_up = logical_up != this->tilt_inverted_;
  if (!this->send_step_command_(protocol_up ? RtsCommand::StepUp : RtsCommand::StepDown,
                                static_cast<uint8_t>(std::abs(delta))))
    return;
  this->tilt = target_step / static_cast<float>(this->tilt_steps_);
  this->publish_state();
}

void SomfyCover::apply_rx_tilt_(RtsCommand command, uint8_t steps) {
  if (this->tilt_steps_ == 0)
    return;
  if (steps == 0) {
    ESP_LOGW(TAG, "Ignoring RTS tilt frame with zero step magnitude");
    return;
  }
  const int direction = ((command == RtsCommand::StepUp) != this->tilt_inverted_) ? 1 : -1;
  this->tilt = clamp(this->tilt + direction * steps / static_cast<float>(this->tilt_steps_), 0.0f, 1.0f);
  ESP_LOGD(TAG, "RX tilt: %s, %u step(s) -> %.0f%%",
           command == RtsCommand::StepUp ? "STEP_UP" : "STEP_DOWN", steps, this->tilt * 100.0f);
  this->publish_state();
}

} // namespace somfy
} // namespace esphome

#endif  // USE_SOMFY_RTS
