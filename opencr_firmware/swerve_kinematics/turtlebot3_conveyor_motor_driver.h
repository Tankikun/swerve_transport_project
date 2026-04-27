#ifndef TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_
#define TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_OPERATING_MODE           11   // EEPROM — must set BEFORE enabling torque
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116

#define OPERATING_MODE_VELOCITY         1    // Velocity Control  → wheels
#define OPERATING_MODE_POSITION         3    // Position Control  → joints (factory default)

#define PROTOCOL_VERSION                2.0

// DYNAMIXEL IDs
#define WHEEL_L_R 3
#define WHEEL_R_R 1
#define WHEEL_L_F 7
#define WHEEL_R_F 5

#define JOINT_L_R 4
#define JOINT_R_R 2
#define JOINT_L_F 8
#define JOINT_R_F 6

#define BAUDRATE                        1000000
#define DEVICENAME                      ""

#define LEN_X_OPERATING_MODE            1
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define ADDR_X_HARDWARE_ERROR           70
#define LEN_X_GOAL_POSITION             4

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool controlJoints(int32_t *value);
  bool controlWheels(int32_t *value);
  bool readByte(uint8_t id, uint16_t addr, uint8_t &value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;

  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWritePosition_;

  bool setOperatingMode(uint8_t id, uint8_t mode);
};

#endif // TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_