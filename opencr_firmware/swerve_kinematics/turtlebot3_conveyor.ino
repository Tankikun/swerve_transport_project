/**
 * turtlebot3_conveyor.ino
 * =======================
 * 4-Wheel Swerve Drive — OpenCR firmware with IK + Commanded-Value Odometry
 *
 * Serial protocol (USB-CDC, 115200 baud):
 *   Receive:  "x_dot y_dot gamma_dot\n"   (floats: m/s, m/s, rad/s)
 *   Example:  "0.15 0.0 0.0\n"            → forward at 0.15 m/s
 *   Reply:    "OK d:δ0,δ1,δ2,δ3 w:ω0,ω1,ω2,ω3\n"
 *   Odom:     "POSE x y theta vx vy wz\n" (~33 Hz, world-frame pose)
 *   Reset:    send "R\n"                  → zeroes odometry
 *   Error:    "ERR\n"
 *
 * Odometry uses commanded IK values (g_modules) — no Dynamixel reads needed.
 * Watchdog: wheels stop if no command within CMD_TIMEOUT_MS.
 */

#include "turtlebot3_conveyor.h"
#include "turtlebot3_conveyor_motor_driver.h"

// ──────────────────────────────────────────────────────────────────────────────
// Globals
// ──────────────────────────────────────────────────────────────────────────────

Turtlebot3MotorDriver motor_driver;

// IK state for 4 modules: [0=FL, 1=FR, 2=RL, 3=RR]
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

// Odometry state (dead-reckoning in world frame, integrated from commanded IK)
static float    g_odom_x     = 0.0f;
static float    g_odom_y     = 0.0f;
static float    g_odom_theta = 0.0f;
static uint32_t g_odom_prev_ms = 0;

// POSE is sent every ODOM_DIV motor cycles (30ms × 10 = 300ms ≈ 3 Hz).
// This keeps serial traffic and RPi CPU load low — identical to backup firmware
// during normal motor operation, preventing brownout when wheels spin up.
static const uint8_t ODOM_DIV = 10;
static uint8_t       g_odom_div_cnt = 0;

// ──────────────────────────────────────────────────────────────────────────────
// Forward kinematics from commanded module states
//
// Uses g_modules[] (already computed by IK — no Dynamixel reads required).
// MODULE_AXIS[] and MODULE_POS[] are from turtlebot3_conveyor.h.
//
// Least-squares FK (A^T A is diagonal for our symmetric L=W geometry):
//   Vx  = (1/4) Σ ω_i·r·cos(θ_axis_i + δ_i)
//   Vy  = (1/4) Σ ω_i·r·sin(θ_axis_i + δ_i)
//   wz  = Σ(-py_i·bx_i + px_i·by_i) / Σ(px_i² + py_i²)
// ──────────────────────────────────────────────────────────────────────────────
static void fk_from_commanded(float &vx, float &vy, float &wz)
{
  float sum_vx = 0.0f, sum_vy = 0.0f;
  float sum_wz_num = 0.0f, sum_wz_den = 0.0f;

  for (int i = 0; i < 4; i++) {
    float theta_axis = atan2f(MODULE_AXIS[i][1], MODULE_AXIS[i][0]);
    float dir = theta_axis + g_modules[i].delta;
    float bx  = g_modules[i].drive_speed * WHEEL_RADIUS * cosf(dir);
    float by  = g_modules[i].drive_speed * WHEEL_RADIUS * sinf(dir);
    float px  = MODULE_POS[i][0];
    float py  = MODULE_POS[i][1];

    sum_vx    += bx;
    sum_vy    += by;
    sum_wz_num += -py * bx + px * by;
    sum_wz_den += px * px + py * py;
  }

  vx = sum_vx / 4.0f;
  vy = sum_vy / 4.0f;
  wz = (sum_wz_den > 1e-6f) ? (sum_wz_num / sum_wz_den) : 0.0f;
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
    int32_t joint_vals[4], wheel_vals[4];
    ik_to_dynamixel(g_modules, joint_vals, wheel_vals);

    bool j_ok = motor_driver.controlJoints(joint_vals);
    bool w_ok = motor_driver.controlWheels(wheel_vals);

    // ── Odometry (no Dynamixel reads — uses commanded IK values) ─────────────
    // Always integrate at full 30ms rate for accuracy.
    float vx, vy, wz;
    fk_from_commanded(vx, vy, wz);

    uint32_t dt_ms = now - g_odom_prev_ms;
    float dt = (float)dt_ms * 0.001f;
    float c  = cosf(g_odom_theta);
    float s  = sinf(g_odom_theta);
    g_odom_x     += (c * vx - s * vy) * dt;
    g_odom_y     += (s * vx + c * vy) * dt;
    g_odom_theta += wz * dt;
    while (g_odom_theta >  M_PI) g_odom_theta -= 2.0f * M_PI;
    while (g_odom_theta < -M_PI) g_odom_theta += 2.0f * M_PI;

    // Send POSE only every ODOM_DIV cycles (~3 Hz) to keep serial
    // traffic minimal — prevents RPi CPU spike / brownout on wheel spin-up.
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
