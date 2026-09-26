#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_RTS

#include "esphome/core/component.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
// remote_base rather than remote_receiver: RemoteReceiverBase is what carries
// register_listener, and it is auto-loaded with the transmitter. ESPHome only
// copies a component's sources into the build when the YAML uses it, so
// including remote_receiver.h here would break configs that have no receiver.
#include "esphome/components/remote_base/remote_base.h"
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome {
namespace somfy {

// RTS command enum (shared between hub and devices)
enum class RtsCommand : uint16_t {
  My      = 0x1,
  Up      = 0x2,
  MyUp    = 0x3,
  Down    = 0x4,
  MyDown  = 0x5,
  UpDown  = 0x6,
  Prog    = 0x8,
  SunFlag = 0x9,
  Flag    = 0xA,
  StepDown = 0x0B,
  StepUp   = 0x8B
};

// Decoded RTS frame (from RX)
struct RtsDecodedFrame {
  uint32_t remote_code{0};
  uint16_t rolling_code{0};
  RtsCommand command{RtsCommand::My};
  uint8_t bit_length{56};
  uint8_t step_size{0};
};

// TX timing constants
struct RtsTiming {
  static constexpr int32_t SYMBOL_USEC = 640;
  static constexpr float TOLERANCE_MIN = 0.7f;
  static constexpr float TOLERANCE_MAX = 1.3f;

  static constexpr uint32_t SOFTWARE_SYNC_USEC = 4850;
  static constexpr int32_t WAKEUP_HIGH_USEC = 9415;
  static constexpr int32_t WAKEUP_LOW_USEC = 9565 + 80000;
  static constexpr int32_t SOFTWARE_SYNC_HIGH_USEC = 4550;
  static constexpr int32_t INTER_FRAME_GAP_USEC = 415 + 30000;

  static constexpr uint8_t FIRST_FRAME_SYNC_COUNT = 2;
  static constexpr uint8_t REPEAT_FRAME_SYNC_COUNT = 7;
};

// Callback type for RX frame notifications
using RtsRxCallback = std::function<void(const RtsDecodedFrame &frame)>;

// Human-readable name of an RTS command ("UP", "DOWN", ..., "UNKNOWN").
const char *rts_command_name(RtsCommand cmd);

/// RTS radio hub — owns the remote_transmitter (and optionally remote_receiver).
/// Devices register for RX callbacks and call send_frame() for TX.
class SomfyRtsHub : public Component, public remote_base::RemoteReceiverListener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration
  void set_remote_transmitter(remote_transmitter::RemoteTransmitterComponent *t) {
    this->remote_transmitter_ = t;
  }
  void set_remote_receiver(remote_base::RemoteReceiverBase *r) { this->remote_receiver_ = r; }

  // TX: encode and transmit an RTS frame
  void send_frame(const std::array<uint8_t, 7> &frame_bytes, uint8_t repeat_count);
  void send_frame(const std::array<uint8_t, 10> &frame_bytes, uint8_t repeat_count,
                  bool update_repeat_extension = false);

  // RX: register a device to receive decoded frames
  // The callback is called for every successfully decoded frame.
  // Devices should filter by remote_code themselves.
  void register_rx_callback(RtsRxCallback callback) { this->rx_callbacks_.push_back(std::move(callback)); }

  bool on_receive(remote_base::RemoteReceiveData data) override;

 protected:
  remote_transmitter::RemoteTransmitterComponent *remote_transmitter_{nullptr};

  remote_base::RemoteReceiverBase *remote_receiver_{nullptr};
  std::vector<RtsRxCallback> rx_callbacks_;

  // Last frame accepted, used to collapse a remote's repeat burst.
  uint32_t rx_last_remote_{0};
  uint16_t rx_last_rolling_{0};
  uint32_t rx_last_ms_{0};
  bool rx_last_valid_{false};

  bool decode_frame_(const remote_base::RawTimings &data, RtsDecodedFrame &decoded, bool debug_log = false);
  // True when this frame is another copy of the press we already dispatched.
  bool rx_is_duplicate_(uint32_t remote_code, uint16_t rolling_code);
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_RTS
