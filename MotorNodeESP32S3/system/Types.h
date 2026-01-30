/*
  Types.h
  -------
  Shared fundamental types used across modules.
  This file MUST NOT include Protocol.h or any comms headers.
*/
#pragma once
#include <Arduino.h>

enum class MotorState : uint8_t {
  MOTOR_DISABLED = 0,
  IDLE,
  MOVING_POSITION,
  MOVING_VELOCITY,
  HOMING,
  FAULT
};


struct Telemetry {
  uint32_t ms = 0;
  MotorState state = MotorState::MOTOR_DISABLED;
  bool motorEnabled = false;
  bool moving = false;
  int32_t pos_steps = 0;
  int32_t target_steps = 0;
  int32_t home_steps = 0;
  int32_t enc_deg_q100 = 0;
  int32_t enc_abs_deg_q100 = 0;
  int32_t error_steps = 0;
  bool lossActive = false;
  uint16_t faultCode = 0;
  int16_t temp_c_q10 = 0;
  uint32_t config_revision = 0;
};
