/**
 * @file Throttle_test.ino
 * @brief Comprehensive Unity test suite for the AirYak closed-loop Throttle governor.
 * @author Luke Leber
 * @version 1.0.0
 */

#include <unity.h>
#include <Arduino.h>
#include <Tachometer.h>
#include <Throttle.h>

// Test Configuration Constants
#define T_TACH_PIN 3
#define T_INPUT_PIN 2
#define T_OUTPUT_PIN 4
#define T_KP 2.0
#define T_KI 0.5
#define T_KD 0.1
#define T_START_POS 15
#define T_DANGER_SPEED 4000
#define T_IDLE_SPEED 1200
#define T_MAX_SPEED 3600
#define T_SERVO_MIN 5
#define T_SERVO_MAX 85
#define T_SWEEP_DELAY_MILLIS 100

/**
 * Helper: Simulates a steady engine speed (RPM) using microsecond flywheel periods
 * and converges the tachometer's 75/25 EMA filter to the exact target speed.
 */
static void simulate_engine_speed(Tachometer& tachometer, uint16_t target_rpm) {
    if (target_rpm == 0) {
        delayMicroseconds(500001UL); // Exceed 500 ms stall timeout
        tachometer.resample();
        return;
    }

    uint32_t period_us = 60000000UL / (uint32_t)target_rpm;
    mock_isr_callback();
    delayMicroseconds(period_us);
    mock_isr_callback();

    // Converge the 75/25 integer EMA filter to the exact target RPM
    for (int i = 0; i < 20; i++) {
        tachometer.resample();
    }
}

void setUp(void) {
    mock_millis = 1000;
    mock_micros = 1000000UL;
    mock_analog_value = 0;
    mock_isr_callback = nullptr;
}

void tearDown(void) {
}

/**
 * 1. Initial & Stationary Baseline (0 RPM)
 */
void test_throttle_initial_and_stationary_baseline(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());

    // Stationary update keeps starting angle
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());
}

/**
 * 2. Electric Cranking Gate (< 1,000 RPM)
 * Verifies speeds below 1,000 RPM (e.g., 600-800 RPM electric starter) keep the
 * governor locked in MANUAL mode at T_START_POS regardless of pilot throttle input.
 */
void test_throttle_cranking_gate_holds_start_position(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    simulate_engine_speed(tachometer, 800);
    mock_analog_value = 1023;
    throttle.update();

    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());
}

/**
 * 3. Input Deadband Filter (<= 90 RPM)
 * Verifies minor potentiometer jitter near idle does not shift target_speed.
 */
void test_throttle_input_deadband_rejects_minor_jitter(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    // Engine running at exact idle target (1200 RPM)
    simulate_engine_speed(tachometer, T_IDLE_SPEED);

    // ADC = 25 -> map(25, 0, 1023, 1200, 3600) = 1258 RPM (delta = 58 RPM <= 90 RPM deadband)
    mock_analog_value = 25;
    throttle.update();

    // Target remains 1200 RPM, process error is 0 -> throttle stays at starting position
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());
}

/**
 * 4. Process Error Deadband (<= 100 RPM)
 * Verifies normal carbureted RPM fluctuations within +/- 100 RPM of target
 * do not trigger PID adjustments.
 */
void test_throttle_process_error_deadband(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    mock_analog_value = 0; // Target = 1200 RPM

    // +80 RPM above target (within 100 RPM deadband)
    simulate_engine_speed(tachometer, 1280);
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());

    // -80 RPM below target (within 100 RPM deadband)
    simulate_engine_speed(tachometer, 1120);
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());
}

/**
 * 5. Governed Upward Tracking & Positive Slew-Rate Clamping (+1.0 deg/cycle)
 */
void test_throttle_governed_upward_slew_clamping(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    simulate_engine_speed(tachometer, 1200);
    mock_analog_value = 1023; // Max demand (3600 RPM)

    // Mock PID adds +2.0 deg per compute; slew limiter clamps to +1.0 deg per cycle
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 1, throttle.getPosition());

    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 2, throttle.getPosition());

    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 3, throttle.getPosition());
}

/**
 * 6. Governed Downward Tracking & Negative Slew-Rate Clamping (-1.0 deg/cycle)
 */
void test_throttle_governed_downward_slew_clamping(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    // Engine overshooting idle target (1500 RPM vs 1200 RPM target, below 4000 RPM danger limit)
    mock_analog_value = 0;
    simulate_engine_speed(tachometer, 1500);

    // Mock PID subtracts -2.0 deg per compute; slew limiter clamps to -1.0 deg per cycle
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS - 1, throttle.getPosition());

    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS - 2, throttle.getPosition());
}

/**
 * 7. Mechanical Servo Limits (Min & Max Bounds)
 */
void test_throttle_respects_servo_min_max_limits(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    // Drive upward past max angle (15 -> 85 requires 70 steps of +1 deg)
    simulate_engine_speed(tachometer, 1200);
    mock_analog_value = 1023;
    for (int i = 0; i < 100; i++) {
        throttle.update();
    }
    TEST_ASSERT_EQUAL_UINT8(T_SERVO_MAX, throttle.getPosition());

    // Drive downward to min angle without breaching dangerous speed (3500 RPM vs 1200 target)
    mock_analog_value = 0;
    simulate_engine_speed(tachometer, 3500);
    for (int i = 0; i < 100; i++) {
        throttle.update();
    }
    TEST_ASSERT_EQUAL_UINT8(T_SERVO_MIN, throttle.getPosition());
}

/**
 * 8. Emergency Overspeed Cutout (>= dangerous_engine_speed)
 */
void test_throttle_emergency_overspeed_cutout(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    // Open throttle slightly during normal operation
    simulate_engine_speed(tachometer, 1200);
    mock_analog_value = 1023;
    throttle.update();
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 2, throttle.getPosition());

    // Overspeed breach (4100 RPM > 4000 RPM danger limit) must immediately slam to T_SERVO_MIN
    simulate_engine_speed(tachometer, T_DANGER_SPEED + 100);
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_SERVO_MIN, throttle.getPosition());}

/**
 * 9. Stall Recovery & Explicit reset()
 */
void test_throttle_stall_recovery_and_reset(void) {
    Tachometer tachometer(T_TACH_PIN);
    Throttle throttle(
        tachometer, T_INPUT_PIN, T_OUTPUT_PIN,
        T_KP, T_KI, T_KD,
        T_START_POS, T_DANGER_SPEED,
        T_IDLE_SPEED, T_MAX_SPEED,
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );

    // Move throttle away from starting position
    simulate_engine_speed(tachometer, 1200);
    mock_analog_value = 1023;
    throttle.update();
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 2, throttle.getPosition());

    // Engine stalls -> Mode 1 restores starting angle
    simulate_engine_speed(tachometer, 0);
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());

    // Move throttle again and verify explicit reset() restores starting baseline
    simulate_engine_speed(tachometer, 1200);
    throttle.update();
    throttle.update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 2, throttle.getPosition());

    throttle.reset();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle.getPosition());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_initial_and_stationary_baseline);
    RUN_TEST(test_throttle_cranking_gate_holds_start_position);
    RUN_TEST(test_throttle_input_deadband_rejects_minor_jitter);
    RUN_TEST(test_throttle_process_error_deadband);
    RUN_TEST(test_throttle_governed_upward_slew_clamping);
    RUN_TEST(test_throttle_governed_downward_slew_clamping);
    RUN_TEST(test_throttle_respects_servo_min_max_limits);
    RUN_TEST(test_throttle_emergency_overspeed_cutout);
    RUN_TEST(test_throttle_stall_recovery_and_reset);
    return UNITY_END();
}