#pragma once

#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/component.h"

#include <vector>

namespace esphome {
namespace remote_receiver {

class RemoteReceiverComponent : public remote_base::RemoteReceiverBase, public Component {};

}  // namespace remote_receiver
}  // namespace esphome
