/*
 * @file hid_event_handler.hpp
 * @date 7/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_HID_EVENT_HANDLER_HPP
#define QUADRUPED_HID_EVENT_HANDLER_HPP

#include "input_hid/hid_event_poll.hpp"
#include "input_hid/keyboard.hpp"
#include "input_hid/joystick.hpp"

#include "quadruped/system_config.hpp"
#include "quadruped/event_handler/hid_event.hpp"
#include "quadruped/event_handler/event_queue.hpp"
#include "quadruped/event_handler/event_handler.hpp"

namespace xmotion {
class HidEventHandler final : public EventHandler {
 public:
  explicit HidEventHandler(const HidConfig& config);

  bool Initialize() override;
  void Start() override;
  void PollEvents() override;

 private:
  HidConfig config_;
  std::unique_ptr<Keyboard> keyboard_;
  std::unique_ptr<Joystick> joystick_;
  HidEventPoll hid_poll_;
  EventQueue<HidEvent> event_queue_;
};
}  // namespace xmotion

#endif  // QUADRUPED_HID_EVENT_HANDLER_HPP
