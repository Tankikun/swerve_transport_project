#ifndef TURTLEBOT3_CONVEYOR_H_
#define TURTLEBOT3_CONVEYOR_H_

/**
 * 4-Wheel Swerve Drive — Inverse Kinematics
 * ==========================================
 *
 * Coordinate frame (ROS convention):
 * X — forward        Y — left        γ̇ — yaw rate, CCW positive
 *
 * Module layout (top-view):
 *
 * +X (forward)
 * [FL]----------[FR]
 * |              |
 * +Y   |    (0,0)     |
 * |              |
 * [RL]----------[RR]
 *
 * At X-home (Dynamixel position 2048), wheels are perpendicular to the
 * outward diagonal — i.e. tangent to the rotation circle.  This is why
 * pure pivot works with all joints at center.
 *
 * Serial protocol:  "x_dot y_dot gamma_dot\n"  (m/s, m/s, rad/s)
 * Example:          "0.1 0.0 0.0\n"            (forward at 0.1 m/s)
 * Reply:            "OK d:δ0,δ1,δ2,δ3 w:ω0,ω1,ω2,ω3\n"
 */

#include <Arduino.h>
#include <math.h>

// ============================================================
// Robot geometry (metres) — MEASURE YOUR ROBOT AND UPDATE!
// L = half the distance between front and rear wheel centres
// W = half the distance between left and right wheel centres
// r = drive wheel radius
// ============================================================
#define HALF_WHEELBASE    0.15f   // L (metres)
#define HALF_TRACK_WIDTH  0.15f   // W (metres)
#define WHEEL_RADIUS      0.033f  // r (metres)

// ============================================================
// Dynamixel unit conversions (XL430-W250-T)
//   Position: 4096 ticks = 360° = 2π rad
//   Velocity: 1 tick ≈ 0.229 RPM
// ============================================================
static const float RAD_TO_DXL_POS  = 4096.0f / (2.0f * M_PI);   // ~651.9 ticks/rad
static const float RADS_TO_DXL_VEL = (60.0f / (2.0f * M_PI)) / 0.229f;  // ~41.7 ticks/(rad/s)

// ============================================================
// Safety limits
// ============================================================
//
// XL430-W250-T physical maximum output shaft speed: ~57–61 RPM = ~6.0–6.35 rad/s
//   MAX_DRIVE_SPEED = 6.0 rad/s  →  max robot linear speed ≈ 6.0 × 0.033 = 0.198 m/s
//
// teleop_twist_keyboard sends cmd_vel in m/s. The IK converts:
//   drive_speed [rad/s] = cmd_speed [m/s] / WHEEL_RADIUS
//   e.g. 0.10 m/s → 3.03 rad/s  (controllable range)
//        0.20 m/s → 6.06 rad/s  (at motor limit, effectively max)
//        0.50 m/s → 15.2 rad/s  (clamped back to 6.0 — no faster than 0.20 m/s)
//
// Useful teleop speed range:  0.05 m/s  →  0.20 m/s
// Anything above 0.20 m/s is clamped — speed control keys ('w'/'x') only
// make a noticeable difference BELOW this value.
static const float MAX_DRIVE_SPEED = 6.0f;    // rad/s — XL430 physical limit (~57 RPM)
static const int32_t STEER_CENTER  = 2048;    // Dynamixel tick at δ=0 (X-home)

// ============================================================
// Module positions [x, y] in robot frame
// Order: [0=FL, 1=FR, 2=RL, 3=RR]
// ============================================================
static const float MODULE_POS[4][2] = {
    { HALF_WHEELBASE,  HALF_TRACK_WIDTH},   // 0: Front-Left
    { HALF_WHEELBASE, -HALF_TRACK_WIDTH},   // 1: Front-Right
    {-HALF_WHEELBASE,  HALF_TRACK_WIDTH},   // 2: Rear-Left
    {-HALF_WHEELBASE, -HALF_TRACK_WIDTH}    // 3: Rear-Right
};

// Module local axes — computed in init_module_axes()
// At X-home (δ=0), the wheel rolls along this direction.
static float MODULE_AXIS[4][2];

// ============================================================
// Module state
// ============================================================
struct ModuleState {
    float delta;        // steering angle in local frame [rad], [-π/2, π/2]
    float drive_speed;  // wheel angular velocity [rad/s]
};

// ============================================================
// Math helpers
// ============================================================
inline float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

inline void rotate2(float x, float y, float theta, float &rx, float &ry) {
    float c = cosf(theta), s = sinf(theta);
    rx = c * x - s * y;
    ry = s * x + c * y;
}

// ============================================================
// Initialize module axes
//
// At X-home, each wheel is PERPENDICULAR to the outward diagonal
// (tangent to the rotation circle).  This is computed as the 90°
// CCW rotation of the outward diagonal: perp(-y, x) of (px, py).
// ============================================================
static void init_module_axes() {
    for (int i = 0; i < 4; ++i) {
        float px = MODULE_POS[i][0];
        float py = MODULE_POS[i][1];
        // 90° CCW rotation of outward diagonal: (-py, px)
        float ax = -py;
        float ay =  px;
        float norm = sqrtf(ax * ax + ay * ay);
        MODULE_AXIS[i][0] = ax / norm;
        MODULE_AXIS[i][1] = ay / norm;
    }
}

// ============================================================
// Inverse kinematics for one module
//
//  1. Compute velocity at wheel contact in robot frame
//  2. Rotate into module local frame
//  3. Compute steering angle δ = atan2(vl_y, vl_x)
//  4. Pick k∈{0,1} to keep δ in [-π/2, π/2] (flip steering
//     + reverse drive when needed)
//  5. Smooth transition: pick k that minimises steering change
// ============================================================
static void compute_module_ik(float x_dot, float y_dot, float gamma_dot,
                              int idx, ModuleState &state)
{
    const float *pos  = MODULE_POS[idx];
    const float *axis = MODULE_AXIS[idx];

    // Step 1: velocity at wheel contact point (robot frame)
    float vx = x_dot - gamma_dot * pos[1];
    float vy = y_dot + gamma_dot * pos[0];
    float speed = sqrtf(vx * vx + vy * vy);

    if (speed < 1e-4f) {
        state.drive_speed = 0.0f;
        // Keep state.delta unchanged — joints hold their current position.
        // Previously resetting delta to 0 caused joints to snap back to X-home
        // every time the robot stopped, creating a visible jerk.
        return;
    }

    // Step 2: rotate into module local frame
    float theta_i = atan2f(axis[1], axis[0]);
    float vl_x, vl_y;
    rotate2(vx, vy, -theta_i, vl_x, vl_y);

    // Step 3: raw steering angle
    float delta_raw = atan2f(vl_y, vl_x);

    // Step 4 & 5: pick k ∈ {0,1} to keep δ in [-π/2, π/2]
    float delta_k0 = delta_raw;
    float delta_k1 = wrap_pi(delta_raw + M_PI);

    bool in0 = (fabsf(delta_k0) <= M_PI / 2.0f + 1e-4f);
    bool in1 = (fabsf(delta_k1) <= M_PI / 2.0f + 1e-4f);

    int best_k;
    if (in0 && in1) {
        // Both valid — pick closest to current angle (smooth transition)
        float err0 = fabsf(wrap_pi(delta_k0 - state.delta));
        float err1 = fabsf(wrap_pi(delta_k1 - state.delta));
        best_k = (err0 <= err1) ? 0 : 1;
    } else if (in0) {
        best_k = 0;
    } else if (in1) {
        best_k = 1;
    } else {
        // Edge case — pick closest
        float err0 = fabsf(wrap_pi(delta_k0 - state.delta));
        float err1 = fabsf(wrap_pi(delta_k1 - state.delta));
        best_k = (err0 <= err1) ? 0 : 1;
    }

    float target_delta = (best_k == 0) ? delta_k0 : delta_k1;
    float drive        = (best_k == 0) ? (speed / WHEEL_RADIUS)
                                       : -(speed / WHEEL_RADIUS);

    // Hard clamp δ to [-π/2, π/2]
    if (target_delta >  M_PI / 2.0f) target_delta =  M_PI / 2.0f;
    if (target_delta < -M_PI / 2.0f) target_delta = -M_PI / 2.0f;

    // Clamp drive speed
    if (drive >  MAX_DRIVE_SPEED) drive =  MAX_DRIVE_SPEED;
    if (drive < -MAX_DRIVE_SPEED) drive = -MAX_DRIVE_SPEED;

    state.delta       = target_delta;
    state.drive_speed = drive;
}

// ============================================================
// Convert IK results → Dynamixel values
//
// IK module order:    [0=FL, 1=FR, 2=RL, 3=RR]
// Motor driver order: [0=L_R, 1=R_R, 2=L_F, 3=R_F]
//
// Mapping:
//   IK[0]=FL → motor[2]=L_F
//   IK[1]=FR → motor[3]=R_F
//   IK[2]=RL → motor[0]=L_R
//   IK[3]=RR → motor[1]=R_R
// ============================================================
static const int IK_TO_MOTOR[4] = {2, 3, 0, 1};

static void ik_to_dynamixel(ModuleState modules[4],
                            int32_t joint_values[4],
                            int32_t wheel_values[4])
{
    for (int ik = 0; ik < 4; ++ik) {
        int m = IK_TO_MOTOR[ik];

        // Steering: δ [rad] → absolute position tick
        int32_t pos_tick = STEER_CENTER + (int32_t)(modules[ik].delta * RAD_TO_DXL_POS);
        if (pos_tick < 0)    pos_tick = 0;
        if (pos_tick > 4095) pos_tick = 4095;
        joint_values[m] = pos_tick;

        // Drive: ω [rad/s] → velocity tick
        wheel_values[m] = (int32_t)(modules[ik].drive_speed * RADS_TO_DXL_VEL);
    }
}

#endif // TURTLEBOT3_CONVEYOR_H_