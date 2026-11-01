#include <unity.h>
#include <Arduino.h>
#include <Tachometer.h>
#include <Throttle.h>

// Environment mocks backed by tests/mock_arduino/Arduino.cpp
extern uint16_t mock_analog_value;

// Test Configuration Constants
#define T_INPUT_PIN 2
#define T_OUTPUT_PIN 3
#define T_KP 2.0
#define T_KI 0.5
#define T_KD 0.1
#define T_START_POS 15
#define T_DANGER_SPEED 4000
#define T_WARMUP_DUR 3000
#define T_IDLE_SPEED 1200
#define T_MAX_SPEED 3600
#define T_SERVO_MIN 5
#define T_SERVO_MAX 85
#define T_SWEEP_DELAY_MILLIS 100
// Subjects Under Test
Tachometer* tachometer;
Throttle* throttle;

/**
 * Helper function to simulate a specific engine speed.
 * Generates the precise amount of flywheel pulses over a 1000ms duration window.
 * Formula: Pulses = (RPM * 1000ms) / 60000ms
 */
void simulate_engine_speed(uint16_t target_rpm) {
    // Force a baseline resample step to flush out any old accumulated time drift 
    // from prior delay updates before generating a fresh snapshot interval
    tachometer->resample();

    uint32_t pulses_needed = ((uint32_t)target_rpm * 1000UL) / 60000UL;
    
    for (uint32_t i = 0; i < pulses_needed; i++) {
        if (mock_isr_callback != nullptr) {
            mock_isr_callback();
        }
    }
    delay(1000); // Step mock_millis forward precisely by 1000ms
    tachometer->resample(); // Process the fresh 1-second data frame
}

void setUp(void) {
    mock_millis = 1000;
    mock_analog_value = 0;
    mock_isr_callback = nullptr;

    // Pin 2 is bound to interrupt 0 in our mock framework
    tachometer = new Tachometer(2); 
    throttle = new Throttle(
        *tachometer, T_INPUT_PIN, T_OUTPUT_PIN, 
        T_KP, T_KI, T_KD, 
        T_START_POS, T_DANGER_SPEED, T_WARMUP_DUR, 
        T_IDLE_SPEED, T_MAX_SPEED, 
        T_SERVO_MIN, T_SERVO_MAX, T_SWEEP_DELAY_MILLIS
    );
}

void tearDown(void) {
    delete throttle;
    delete tachometer;
}

/**
 * 1. Cranking State (Stall Condition)
 */
void test_throttle_cranking_baseline(void) {
    // No pulses generated means tachometer speed defaults to 0
    throttle->update();
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle->getPosition());
}

/**
 * 2. Warmup Hold Condition
 */
void test_throttle_warmup_hold_behavior(void) {
    // Engine fires up and runs at 1200 RPM
    simulate_engine_speed(1200); 
    throttle->update(); // Captures the throttle initialization start_time anchor (elapsed = 0ms)

    // Advance time by half the warmup threshold window
    delay(T_WARMUP_DUR / 2); 
    
    // Simulate pilot input demand shifting to maximum scale parameters
    mock_analog_value = 1023; 
    throttle->update();

    // The throttle must ignore the input changes and hold position during warmup
    TEST_ASSERT_EQUAL_UINT8(T_START_POS, throttle->getPosition());
}

/**
 * 3. Governed Autopilot Updates
 */
void test_throttle_governed_autopilot_tracking(void) {
    simulate_engine_speed(1200); 
    throttle->update(); // Captures the start_time anchor (elapsed = 0ms)

    // Advance time completely past the warmup threshold
    delay(T_WARMUP_DUR + 500); 
    
    // Set pilot demand potentiometer to exactly 50% scale
    mock_analog_value = 512; 
    throttle->update();

    // Target Calculation: map(512, 0, 1023, 1200, 3600) = 2401 RPM reference target
    // Since current speed (1200) < target (2401), the real PID library logic outputs
    // a correction signal increasing throttle position (+2 degrees via our PID mock engine)
    TEST_ASSERT_EQUAL_UINT8(T_START_POS + 2, throttle->getPosition());
}

/**
 * 4. Emergency Overspeed Cutout
 */
void test_throttle_emergency_overspeed_safety_cut(void) {
    simulate_engine_speed(1200);
    throttle->update();
    
    delay(T_WARMUP_DUR + 500);
    mock_analog_value = 512;
    throttle->update(); // Throttle opens slightly to track demand

    // CRITICAL TRIGGER: Simulate flywheel spinning past danger threshold (e.g., 4500 RPM)
    simulate_engine_speed(T_DANGER_SPEED + 1000);
    throttle->update();

    // Overspeed protection circuit must instantly drop output signals to absolute zero
    TEST_ASSERT_EQUAL_UINT8(T_SERVO_MIN, throttle->getPosition());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_cranking_baseline);
    RUN_TEST(test_throttle_warmup_hold_behavior);
    RUN_TEST(test_throttle_governed_autopilot_tracking);
    RUN_TEST(test_throttle_emergency_overspeed_safety_cut);
    return UNITY_END();
}