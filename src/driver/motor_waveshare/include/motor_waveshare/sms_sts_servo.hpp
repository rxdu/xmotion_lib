/*
 * @file sms_sts_servo.hpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SMS_STS_SERVO_HPP
#define XMOTION_SMS_STS_SERVO_HPP

#include <string>
#include <cstdint>
#include <memory>
#include <vector>
#include <unordered_map>

#include "interface/driver/motor_controller_interface.hpp"

namespace xmotion {
class SmsStsServo : public MotorControllerInterface {
 public:
  enum class Mode { kSpeed = 0, kPosition, kUnknown = 0xff };

  struct State {
    bool is_moving;
    float position;
    float speed;
    float load;
    float voltage;
    float temperature;
    float current;
  };

 public:
  SmsStsServo(uint8_t id);
  SmsStsServo(const std::vector<uint8_t>& ids);
  ~SmsStsServo();

  // do not allow copy
  SmsStsServo(const SmsStsServo&) = delete;
  SmsStsServo& operator=(const SmsStsServo&) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  // for single-servo control
  void SetSpeed(float step_per_sec) override;
  float GetSpeed() override;

  // position offset is used to remap the position command
  // for example, if the servo is mounted at 180 degree, set offset to 180,
  // then set position to 0 will actually move the servo to 180 degree
  void SetPositionOffset(float offset);
  void SetPosition(float position) override;
  float GetPosition() override;

  bool IsNormal() override;

  State GetState() const;

  // the following functions may not be called during normal motor operation
  // in most cases, motor id and mode should be set beforehand
  bool SetMode(Mode mode, uint32_t timeout_ms = 100);
  bool SetMotorId(uint8_t id);
  bool SetNeutralPosition();

  // for multi-servo control
  void SetPosition(std::vector<float> positions);
  std::unordered_map<uint8_t, float> GetPositions();
  std::unordered_map<uint8_t, State> GetStates() const;

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
}  // namespace xmotion

#endif  // XMOTION_SMS_STS_SERVO_HPP