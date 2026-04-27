#include "turtlebot3_conveyor_motor_driver.h"
#include <Arduino.h>

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  portHandler_(nullptr),
  packetHandler_(nullptr),
  groupSyncWriteVelocity_(nullptr),
  groupSyncWritePosition_(nullptr)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // If port was already open (e.g. re-init via 'r'), close it first so openPort() succeeds
  if (portHandler_->isPacketTimeout() || portHandler_->getBaudRate() > 0)
  {
    portHandler_->closePort();
  }

  if (!portHandler_->openPort())
  {
    Serial.println("[ERROR] Failed to open Dynamixel port.");
    return false;
  }

  if (!portHandler_->setBaudRate(baudrate_))
  {
    Serial.println("[ERROR] Failed to set Dynamixel baudrate.");
    return false;
  }

  // ── Step 1: Reboot all motors to clear any hardware error state ──────────
  // A motor in hardware error (overload, etc.) rejects EEPROM writes and
  // ignores velocity/position commands.  Rebooting clears the error register
  // and brings each motor back to a known-good state.
  // IMPORTANT: each XL430 takes ~300-500ms to reboot. We wait 600ms after
  // each individual reboot so the motor is fully online before the next one.
  uint8_t dxl_err = 0;
  uint8_t ids[8] = {WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F,
                    JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F};
  for (int i = 0; i < 8; i++) {
    packetHandler_->reboot(portHandler_, ids[i], &dxl_err);
    delay(600);   // wait for this motor to finish rebooting before next
  }

  // ── Step 2: Disable torque on ALL motors ──────────────────────────────────
  // Operating Mode is an EEPROM register; it can ONLY be written while torque
  // is OFF.  After reboot, torque is off by default, but we explicitly set it.
  setTorque(WHEEL_L_R, false); setTorque(WHEEL_R_R, false);
  setTorque(WHEEL_L_F, false); setTorque(WHEEL_R_F, false);
  setTorque(JOINT_L_R, false); setTorque(JOINT_R_R, false);
  setTorque(JOINT_L_F, false); setTorque(JOINT_R_F, false);
  delay(50);

  // ── Step 3: Set operating modes ───────────────────────────────────────────
  // Wheels → Velocity Control (mode 1)
  setOperatingMode(WHEEL_L_R, OPERATING_MODE_VELOCITY);
  setOperatingMode(WHEEL_R_R, OPERATING_MODE_VELOCITY);
  setOperatingMode(WHEEL_L_F, OPERATING_MODE_VELOCITY);
  setOperatingMode(WHEEL_R_F, OPERATING_MODE_VELOCITY);

  // Joints → Position Control (mode 3)
  setOperatingMode(JOINT_L_R, OPERATING_MODE_POSITION);
  setOperatingMode(JOINT_R_R, OPERATING_MODE_POSITION);
  setOperatingMode(JOINT_L_F, OPERATING_MODE_POSITION);
  setOperatingMode(JOINT_R_F, OPERATING_MODE_POSITION);
  delay(50);

  // ── Step 4: Enable torque ─────────────────────────────────────────────────
  setTorque(WHEEL_L_R, true); setTorque(WHEEL_R_R, true);
  setTorque(WHEEL_L_F, true); setTorque(WHEEL_R_F, true);
  setTorque(JOINT_L_R, true); setTorque(JOINT_R_R, true);
  setTorque(JOINT_L_F, true); setTorque(JOINT_R_F, true);

  // ── Step 5: Create (or recreate) GroupSyncWrite objects ───────────────────
  delete groupSyncWriteVelocity_;
  delete groupSyncWritePosition_;
  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  Serial.println("[OK] Motor driver initialized. Operating modes set.");
  return true;
}

bool Turtlebot3MotorDriver::setOperatingMode(uint8_t id, uint8_t mode)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler_->write1ByteTxRx(
    portHandler_, id, ADDR_X_OPERATING_MODE, mode, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print("[WARN] setOperatingMode comm fail, ID="); Serial.println(id);
    return false;
  }
  if (dxl_error != 0)
  {
    Serial.print("[WARN] setOperatingMode dxl error, ID="); Serial.println(id);
    return false;
  }
  return true;
}

bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS) {
    packetHandler_->getTxRxResult(dxl_comm_result);
    return false;
  }
  else if(dxl_error != 0) {
    packetHandler_->getRxPacketError(dxl_error);
    return false;
  }

  return true;
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Guard: do nothing if init() was never called
  if (portHandler_ == nullptr || packetHandler_ == nullptr) return;

  setTorque(WHEEL_L_R, false); setTorque(WHEEL_R_R, false);
  setTorque(WHEEL_L_F, false); setTorque(WHEEL_R_F, false);
  setTorque(JOINT_L_R, false); setTorque(JOINT_R_R, false);
  setTorque(JOINT_L_F, false); setTorque(JOINT_R_F, false);

  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::readByte(uint8_t id, uint16_t addr, uint8_t &value)
{
  uint8_t dxl_error = 0;
  int result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, &value, &dxl_error);
  
  if (result != COMM_SUCCESS) {
    // True communication failure (e.g., timeout, wire unplugged)
    return false;
  }
  
  if (dxl_error != 0) {
    // Communication succeeded, but the motor is reporting a hardware fault!
    Serial.print(" [DXL_ERR: "); 
    Serial.print(dxl_error); 
    Serial.print("] ");
    return true; // We still successfully read the byte, so return true!
  }
  
  return true;
}

bool Turtlebot3MotorDriver::controlJoints(int32_t *value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_L_R, (uint8_t*)&value[0]);
  if (dxl_addparam_result_ != true) { groupSyncWritePosition_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_R_R, (uint8_t*)&value[1]);
  if (dxl_addparam_result_ != true) { groupSyncWritePosition_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_L_F, (uint8_t*)&value[2]);
  if (dxl_addparam_result_ != true) { groupSyncWritePosition_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_R_F, (uint8_t*)&value[3]);
  if (dxl_addparam_result_ != true) { groupSyncWritePosition_->clearParam(); return false; }

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  groupSyncWritePosition_->clearParam();

  if (dxl_comm_result_ != COMM_SUCCESS) {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  return true;
}

bool Turtlebot3MotorDriver::controlWheels(int32_t *value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_L_R, (uint8_t*)&value[0]);
  if (dxl_addparam_result_ != true) { groupSyncWriteVelocity_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_R_R, (uint8_t*)&value[1]);
  if (dxl_addparam_result_ != true) { groupSyncWriteVelocity_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_L_F, (uint8_t*)&value[2]);
  if (dxl_addparam_result_ != true) { groupSyncWriteVelocity_->clearParam(); return false; }

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_R_F, (uint8_t*)&value[3]);
  if (dxl_addparam_result_ != true) { groupSyncWriteVelocity_->clearParam(); return false; }

  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  groupSyncWriteVelocity_->clearParam();

  if (dxl_comm_result_ != COMM_SUCCESS) {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  return true;
}
