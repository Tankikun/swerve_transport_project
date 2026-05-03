/**
 * turtlebot3_conveyor.ino
 * =======================
 * 4-Wheel Swerve Drive — OpenCR firmware with IK + ENCODER-based Odometry
 *
 * Serial protocol (USB-CDC, 115200 baud):
 *   Receive:  "x_dot y_dot gamma_dot\n"   (floats: m/s, m/s, rad/s)
 *   Example:  "0.15 0.0 0.0\n"            → forward at 0.15 m/s
 *   Reply:    "OK d:δ0,δ1,δ2,δ3 w:ω0,ω1,ω2,ω3\n"
 *   Odom:     "POSE x y theta vx vy wz\n" (~33 Hz, world-frame pose)
 *   Reset:    send "R\n"                  → zeroes odometry
 *   Error:    "ERR\n"
 *
 * Odometry (NEW): each motor cycle the firmware does a GroupSyncRead of
 * Present Position (joints) and Present Velocity (wheels) from the
 * Dynamixel bus, converts those to per-module (delta, drive_speed),
 * and integrates body velocity from FK on the MEASURED values. If a
 * read fails it falls back to the commanded values for that cycle so
 * the loop still advances.
 *
 * IMU (NEW): the OpenCR's onboard MPU-9250 is read every motor cycle
 * via the cIMU library. A separate "IMU" line is emitted every
 * IMU_DIV motor cycles (~10 Hz) carrying body-frame acceleration,
 * body-frame angular velocity, and the filtered yaw angle. The Pi
 * can fuse this with wheel odometry in the EKF for a more stable
 * heading estimate.
 *
 * Serial protocol additions:
 *   Send:  "IMU ax ay az gx gy gz yaw\n"
 *          ax,ay,az  body linear acceleration [m/s²]
 *          gx,gy,gz  body angular velocity    [rad/s]
 *          yaw       filtered yaw angle        [rad]
 *
 * Watchdog: wheels stop if no command within CMD_TIMEOUT_MS.
 */

#include "turtlebot3_conveyor.h"
#include "turtlebot3_conveyor_motor_driver.h"
#include <IMU.h>

// MPU-9250 default full-scale ranges:
//   Accel ±2 g    →   16384 LSB / g
//   Gyro  ±250 dps→     131 LSB / (deg/s)
// If you change these in the IMU library, update the conversions below.
static const float ACC_LSB_TO_MS2  = (1.0f / 16384.0f) * 9.80665f;
static const float GYRO_LSB_TO_RAD = (1.0f / 131.0f)   * (M_PI / 180.0f);
// NOTE: OpenCR's wiring_constants.h already defines DEG_TO_RAD as a macro,
// so we use a local name (DEG2RAD) to avoid the collision.
static const float DEG2RAD         = M_PI / 180.0f;

// ──────────────────────────────────────────────────────────────────────────────
// Globals
// ──────────────────────────────────────────────────────────────────────────────

Turtlebot3MotorDriver motor_driver;
cIMU imu;

// IK state for 4 modules: [0=FL, 1=FR, 2=RL, 3=RR]
// These are the COMMANDED targets — they drive the motor outputs.
static ModuleState g_modules[4] = {{0.0f, 0.0f}, {0.0f, 0.0f},
                                    {0.0f, 0.0f}, {0.0f, 0.0f}};

// Watchdog
static uint32_t g_last_cmd_ms   = 0;
static bool     g_wheels_stopped = true;
static const uint32_t CMD_TIMEOUT_MS = 5000;  // 5 s: keep last command alive before stopping

// Serial receive buffer
static char    s_rx_buf[64];
static uint8_t s_rx_idx = 0;

// Debug: print motor ticks for N cycles after a new command
static int g_debug_cycles = 0;

// Odometry state (dead-reckoning in world frame, integrated from FK)
static float    g_odom_x     = 0.0f;
static float    g_odom_y     = 0.0f;
static float    g_odom_theta = 0.0f;
static uint32_t g_odom_prev_ms = 0;

// Debug counter for encoder-read failures (for occasional reporting)
static uint32_t g_enc_read_fail_count = 0;

// POSE is sent every ODOM_DIV motor cycles. Motor cycle is 30 ms,
// so ODOM_DIV = 1 gives 33 Hz POSE rate (the previous 10 → 3 Hz was
// too slow for the EKF prediction step and for RTAB-Map's motion
// hint between camera frames).
//
// Bandwidth check: ~54 bytes per POSE line × 33 Hz ≈ 1.8 kB/s.
// 115200 baud → ~11.5 kB/s budget. We sit at ~16 % of serial bandwidth
// even with IMU emission too — plenty of headroom.
static const uint8_t ODOM_DIV = 1;
static uint8_t       g_odom_div_cnt = 0;

// IMU line is sent every IMU_DIV motor cycles (30ms × 3 = 90ms ≈ 11 Hz),
// faster than POSE so it's actually useful as a high-rate angular-rate
// source for downstream EKF fusion.
static const uint8_t IMU_DIV = 3;
static uint8_t       g_imu_div_cnt = 0;

// ──────────────────────────────────────────────────────────────────────────────
// Forward kinematics from a given module state (commanded OR measured).
//
// Least-squares FK (A^T A is diagonal for our symmetric L=W geometry):
//   Vx  = (1/4) Σ ω_i·r·cos(θ_axis_i + δ_i)
//   Vy  = (1/4) Σ ω_i·r·sin(θ_axis_i + δ_i)
//   wz  = Σ(-py_i·bx_i + px_i·by_i) / Σ(px_i² + py_i²)
//
// Pass g_modules[] for commanded-value FK; pass an array filled from
// the encoder reads for true closed-loop FK.
// ──────────────────────────────────────────────────────────────────────────────
static void fk_from_module_states(const ModuleState modules[4],
                                  float &vx, float &vy, float &wz)
{
  float sum_vx = 0.0f, sum_vy = 0.0f;
  float sum_wz_num = 0.0f, sum_wz_den = 0.0f;

  for (int i = 0; i < 4; i++) {
    float theta_axis = atan2f(MODULE_AXIS[i][1], MODULE_AXIS[i][0]);
    float dir = theta_axis + modules[i].delta;
    float bx  = modules[i].drive_speed * WHEEL_RADIUS * cosf(dir);
    float by  = modules[i].drive_speed * WHEEL_RADIUS * sinf(dir);
    float px  = MODULE_POS[i][0];
    float py  = MODULE_POS[i][1];

    sum_vx    += bx;
    sum_vy    += by;
    sum_wz_num += -py * bx + px * by;
    sum_wz_den += px * px + py * py;
  }

  vx = sum_vx / 4.0f;
  vy = sum_vy / 4.0f;
  // Empirical sign correction (verified 2026-05-04 with bench test:
  // commanded angular.z=+0.5 rad/s → physical CCW rotation → without
  // this flip, /tb3_1/odom reports CW). The IK at the top of
  // turtlebot3_conveyor.h and the FK formula above are both
  // mathematically right-handed (z-up, CCW-positive), so the
  // sign mismatch between commanded gamma_dot and reconstructed
  // wz must come from somewhere in the encoder-feedback chain
  // (likely a per-side mounting orientation or a swapped row in
  // MODULE_POS / IK_TO_MOTOR). Forward and strafe both verify
  // correct (sign symmetric for those modes), so this targeted
  // negation only affects yaw integration. Re-test rotation after
  // re-flashing: cmd_vel.angular.z=+0.5 should now produce a
  // positive yaw delta in /tb3_1/odom matching CCW physical motion.
  wz = (sum_wz_den > 1e-6f) ? -(sum_wz_num / sum_wz_den) : 0.0f;
}

// ──────────────────────────────────────────────────────────────────────────────
// Read encoder values from all 8 motors and convert to per-IK-module state.
//
// Two GroupSyncRead packets:
//   joint_raw[0..3]  in MOTOR order [L_R, R_R, L_F, R_F]   — ticks 0..4095
//   wheel_raw[0..3]  in MOTOR order [L_R, R_R, L_F, R_F]   — signed velocity ticks
//
// Then map to IK order [FL, FR, RL, RR] using the inverse of IK_TO_MOTOR.
// IK_TO_MOTOR = {2, 3, 0, 1}, which is its own inverse, so the same
// table maps motor index → IK index.
//
// Returns true on success. On false, caller should fall back to commanded.
// ──────────────────────────────────────────────────────────────────────────────
static bool read_measured_module_states(ModuleState measured[4])
{
  int32_t joint_raw[4];
  int32_t wheel_raw[4];

  if (!motor_driver.readJointPositions(joint_raw))   return false;
  if (!motor_driver.readWheelVelocities(wheel_raw))  return false;

  for (int m = 0; m < 4; m++) {
    int ik = IK_TO_MOTOR[m];   // self-inverse permutation, ok to use both ways

    // Steering tick → angle (rad), centred on STEER_CENTER (=2048).
    float delta = ((float)(joint_raw[m] - STEER_CENTER)) / RAD_TO_DXL_POS;
    // Clamp to the same δ ∈ [-π/2, π/2] window the IK enforces, so FK
    // stays consistent with the IK convention even if a joint overshoots.
    if (delta >  M_PI / 2.0f) delta =  M_PI / 2.0f;
    if (delta < -M_PI / 2.0f) delta = -M_PI / 2.0f;

    // Velocity tick → angular velocity (rad/s); already signed via int32 cast.
    float omega = ((float)wheel_raw[m]) / RADS_TO_DXL_VEL;

    measured[ik].delta       = delta;
    measured[ik].drive_speed = omega;
  }
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Motor diagnostics
// ──────────────────────────────────────────────────────────────────────────────
static void printMotorDiag()
{
  Serial.println("=== MOTOR DIAGNOSTICS ===");

  struct { uint8_t id; const char* name; uint8_t expected_mode; } motors[8] = {
    {WHEEL_L_R, "WHEEL_L_R(3)", 1},
    {WHEEL_R_R, "WHEEL_R_R(1)", 1},
    {WHEEL_L_F, "WHEEL_L_F(7)", 1},
    {WHEEL_R_F, "WHEEL_R_F(5)", 1},
    {JOINT_L_R, "JOINT_L_R(4)", 3},
    {JOINT_R_R, "JOINT_R_R(2)", 3},
    {JOINT_L_F, "JOINT_L_F(8)", 3},
    {JOINT_R_F, "JOINT_R_F(6)", 3},
  };

  bool all_ok = true;
  for (int i = 0; i < 8; i++) {
    uint8_t op_mode = 255, hw_err = 255;
    bool mode_ok = motor_driver.readByte(motors[i].id, ADDR_X_OPERATING_MODE, op_mode);
    bool err_ok  = motor_driver.readByte(motors[i].id, ADDR_X_HARDWARE_ERROR, hw_err);

    Serial.print("  ");
    Serial.print(motors[i].name);
    Serial.print(": mode=");

    if (mode_ok) {
      Serial.print(op_mode);
      if (op_mode != motors[i].expected_mode) {
        Serial.print(" *** WRONG (expected ");
        Serial.print(motors[i].expected_mode);
        Serial.print(") ***");
        all_ok = false;
      }
    } else {
      Serial.print("READ_FAIL");
      all_ok = false;
    }

    Serial.print("  hw_err=");
    if (err_ok) {
      Serial.print(hw_err);
      if (hw_err != 0) {
        Serial.print(" *** HAS ERROR ***");
        all_ok = false;
      }
    } else {
      Serial.print("READ_FAIL");
      all_ok = false;
    }
    Serial.println();
  }

  if (all_ok) Serial.println("=== ALL MOTORS OK ===");
  else        Serial.println("=== SOME MOTORS HAVE ISSUES ===");
}

// ──────────────────────────────────────────────────────────────────────────────
// Serial command parser: "x_dot y_dot gamma_dot\n"
// ──────────────────────────────────────────────────────────────────────────────
static bool try_parse_command(float &x_dot, float &y_dot, float &gamma_dot)
{
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (s_rx_idx == 0) continue;
      s_rx_buf[s_rx_idx] = '\0';
      s_rx_idx = 0;

      if (s_rx_buf[0] == 'd') { printMotorDiag(); continue; }

      // 'R' — reset odometry to origin
      if (s_rx_buf[0] == 'R') {
        g_odom_x = g_odom_y = g_odom_theta = 0.0f;
        Serial.println("ODOM_RESET");
        continue;
      }

      float a, b, g;
      if (sscanf(s_rx_buf, "%f %f %f", &a, &b, &g) == 3) {
        x_dot = a; y_dot = b; gamma_dot = g;
        return true;
      }
      Serial.println("ERR");
    } else {
      if (s_rx_idx < (uint8_t)(sizeof(s_rx_buf) - 1))
        s_rx_buf[s_rx_idx++] = c;
      else {
        s_rx_idx = 0;
        Serial.println("ERR");
      }
    }
  }
  return false;
}

// ──────────────────────────────────────────────────────────────────────────────
// Stop all drive wheels (steering stays in place)
// ──────────────────────────────────────────────────────────────────────────────
static void stop_all_wheels()
{
  if (g_wheels_stopped) return;
  int32_t vel[4] = {0, 0, 0, 0};
  motor_driver.controlWheels(vel);
  for (int i = 0; i < 4; i++) g_modules[i].drive_speed = 0.0f;
  g_wheels_stopped = true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Arduino entry points
// ──────────────────────────────────────────────────────────────────────────────

void setup()
{
  Serial.begin(115200);

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(300);

  motor_driver.init();
  init_module_axes();

  // ── IMU (MPU-9250 onboard the OpenCR) ────────────────────────────────────
  // begin() spins up SPI, configures sensor ranges and starts the
  // Madgwick attitude filter. Reasonably fast — couple hundred ms.
  Serial.println("Initializing onboard IMU (MPU-9250)...");
  imu.begin();
  Serial.println("IMU ready.");

  // Homing
  Serial.println("Homing all joints to center (tick=2048)...");
  int32_t home_joints[4] = {STEER_CENTER, STEER_CENTER, STEER_CENTER, STEER_CENTER};
  int32_t zero_wheels[4] = {0, 0, 0, 0};
  for (int i = 0; i < 50; i++) {
    motor_driver.controlJoints(home_joints);
    motor_driver.controlWheels(zero_wheels);
    delay(30);
  }
  Serial.println("Homing done.");

  printMotorDiag();

  g_last_cmd_ms    = millis();
  g_odom_prev_ms   = millis();
  g_wheels_stopped = true;

  Serial.println("Ready. Send: x_dot y_dot gamma_dot  (m/s, m/s, rad/s)");
  Serial.println("  'd' = motor diag | 'R' = reset odometry");
}

void loop()
{
  static uint32_t prev_motor_time = 0;
  uint32_t now = millis();

  float x_dot = 0.0f, y_dot = 0.0f, gamma_dot = 0.0f;

  if (try_parse_command(x_dot, y_dot, gamma_dot)) {
    g_last_cmd_ms    = now;
    g_wheels_stopped = false;
    g_debug_cycles   = 3;

    for (int i = 0; i < 4; i++)
      compute_module_ik(x_dot, y_dot, gamma_dot, i, g_modules[i]);

    // Echo IK result
    Serial.print("OK d:");
    for (int i = 0; i < 4; i++) {
      Serial.print(g_modules[i].delta, 3);
      if (i < 3) Serial.print(",");
    }
    Serial.print(" w:");
    for (int i = 0; i < 4; i++) {
      Serial.print(g_modules[i].drive_speed, 3);
      if (i < 3) Serial.print(",");
    }
    Serial.println();
  }

  // Watchdog
  if ((now - g_last_cmd_ms) > CMD_TIMEOUT_MS)
    stop_all_wheels();

  // Motor cycle: 30 ms
  if ((now - prev_motor_time) >= 30) {
    // Update the IMU once per motor cycle (~33 Hz), so the Madgwick
    // attitude filter sees a stable sample rate. The send to serial
    // is gated separately by IMU_DIV below.
    imu.update();

    int32_t joint_vals[4], wheel_vals[4];
    ik_to_dynamixel(g_modules, joint_vals, wheel_vals);

    bool j_ok = motor_driver.controlJoints(joint_vals);
    bool w_ok = motor_driver.controlWheels(wheel_vals);

    // ── Odometry from ENCODER feedback ───────────────────────────────────────
    // GroupSyncRead present position (joints) + present velocity (wheels)
    // from the Dynamixel bus; convert to per-module (delta, drive_speed);
    // run FK on those measured states.
    //
    // If either read fails (bus glitch, motor not responding) fall back to
    // FK on commanded values so the integrator keeps advancing instead of
    // stalling. Increment a counter so we can warn occasionally.
    float vx, vy, wz;
    ModuleState measured[4];
    bool used_encoders = read_measured_module_states(measured);
    if (used_encoders) {
      fk_from_module_states(measured, vx, vy, wz);
    } else {
      g_enc_read_fail_count++;
      fk_from_module_states(g_modules, vx, vy, wz);
    }

    uint32_t dt_ms = now - g_odom_prev_ms;
    float dt = (float)dt_ms * 0.001f;
    float c  = cosf(g_odom_theta);
    float s  = sinf(g_odom_theta);
    g_odom_x     += (c * vx - s * vy) * dt;
    g_odom_y     += (s * vx + c * vy) * dt;
    g_odom_theta += wz * dt;
    while (g_odom_theta >  M_PI) g_odom_theta -= 2.0f * M_PI;
    while (g_odom_theta < -M_PI) g_odom_theta += 2.0f * M_PI;

    // Send POSE every ODOM_DIV cycles (now 33 Hz with ODOM_DIV=1).
    if (++g_odom_div_cnt >= ODOM_DIV) {
      g_odom_div_cnt = 0;
      Serial.print("POSE ");
      Serial.print(g_odom_x, 4);
      Serial.print(" ");
      Serial.print(g_odom_y, 4);
      Serial.print(" ");
      Serial.print(g_odom_theta, 4);
      Serial.print(" ");
      Serial.print(vx, 4);
      Serial.print(" ");
      Serial.print(vy, 4);
      Serial.print(" ");
      Serial.print(wz, 4);
      Serial.println();
      // (Encoder vs commanded source isn't appended to the POSE line —
      //  conveyor_base_node.py expects exactly 7 space-separated fields.
      //  See the [WARN] line below for fallback reporting instead.)

      // Occasional encoder-fail summary so a degraded bus is visible without
      // spamming. ~3 Hz POSE rate × 10 = ~30 cycles → roughly every 3 s.
      static uint8_t s_pose_count = 0;
      if (++s_pose_count >= 10) {
        s_pose_count = 0;
        if (g_enc_read_fail_count > 0) {
          Serial.print("[WARN] encoder reads failed ");
          Serial.print(g_enc_read_fail_count);
          Serial.println(" times in last ~3 s; using commanded fallback.");
          g_enc_read_fail_count = 0;
        }
      }
    }
    // ─────────────────────────────────────────────────────────────────────────

    // ── IMU send (every IMU_DIV motor cycles, ~11 Hz) ────────────────────────
    // Format: "IMU ax ay az gx gy gz yaw\n"
    //   ax,ay,az  body linear acceleration [m/s²]
    //   gx,gy,gz  body angular velocity    [rad/s]
    //   yaw       Madgwick-filtered yaw    [rad]
    if (++g_imu_div_cnt >= IMU_DIV) {
      g_imu_div_cnt = 0;
      // accData / gyroData are int16 raw LSB values.
      // angle[2] is filtered yaw in DEGREES (cIMU library convention).
      float ax = (float)imu.accData[0]  * ACC_LSB_TO_MS2;
      float ay = (float)imu.accData[1]  * ACC_LSB_TO_MS2;
      float az = (float)imu.accData[2]  * ACC_LSB_TO_MS2;
      float gx = (float)imu.gyroData[0] * GYRO_LSB_TO_RAD;
      float gy = (float)imu.gyroData[1] * GYRO_LSB_TO_RAD;
      float gz = (float)imu.gyroData[2] * GYRO_LSB_TO_RAD;
      float yaw = imu.angle[2] * DEG2RAD;

      Serial.print("IMU ");
      Serial.print(ax, 4);  Serial.print(" ");
      Serial.print(ay, 4);  Serial.print(" ");
      Serial.print(az, 4);  Serial.print(" ");
      Serial.print(gx, 4);  Serial.print(" ");
      Serial.print(gy, 4);  Serial.print(" ");
      Serial.print(gz, 4);  Serial.print(" ");
      Serial.print(yaw, 4);
      Serial.println();
    }
    // ─────────────────────────────────────────────────────────────────────────

    if (g_debug_cycles > 0) {
      g_debug_cycles--;
      Serial.print("[DBG] joints=[");
      for (int i = 0; i < 4; i++) {
        Serial.print(joint_vals[i]);
        if (i < 3) Serial.print(",");
      }
      Serial.print("] ");
      Serial.print(j_ok ? "J:OK" : "J:FAIL");
      Serial.print("  wheels=[");
      for (int i = 0; i < 4; i++) {
        Serial.print(wheel_vals[i]);
        if (i < 3) Serial.print(",");
      }
      Serial.print("] ");
      Serial.println(w_ok ? "W:OK" : "W:FAIL");
    }

    g_odom_prev_ms  = now;
    prev_motor_time = now;
  }
}
