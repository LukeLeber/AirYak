#include <unity.h>
#include <Arduino.h>
#include <FlywheelMagnetInterrupt.h>

void setUp(void) {
    mock_millis = 1000;
    mock_isr_callback = nullptr;
}

void tearDown(void) {

}

/**
 * Verifies that an invalid pin input handles initialization safely and 
 * does not register structural hooks.
 */
void test_invalid_pin_binding_rejection(void) {
    FlywheelMagnetInterrupt invalidSensor(99); // Pin 99 is unmapped in mock
    
    // An invalid sensor instantiation must safely return 0 pulses
    TEST_ASSERT_EQUAL_UINT8(0, invalidSensor.getPulses());
}

/**
 * Asserts that the instance safely binds its structural tracking loops 
 * and processes simulated hardware edge events accurately.
 */
void test_pulse_accumulation_and_flush(void) {
    FlywheelMagnetInterrupt sensor(2);
    // Baseline validation: The loop register should be completely empty initially
    TEST_ASSERT_EQUAL_UINT8(0, sensor.getPulses());

    // Verify the hardware callback was successfully captured during initialization
    TEST_ASSERT_NOT_NULL(mock_isr_callback);

    // Simulate exactly 4 physical revolutions of the crankshaft
    mock_isr_callback();
    mock_isr_callback();
    mock_isr_callback();
    mock_isr_callback();

    // Verify getPulses() correctly extracts the cumulative event data
    TEST_ASSERT_EQUAL_UINT8(4, sensor.getPulses());

    // CRITICAL REQUIREMENT: A read statement must flush the register back to zero
    TEST_ASSERT_EQUAL_UINT8(0, sensor.getPulses());
}

/**
 * Validates that asynchronous event increments keep logging cleanly 
 * across subsequent sequential reading boundaries.
 */
void test_subsequent_reading_cycles(void) {
    FlywheelMagnetInterrupt sensor(2);

    // Cycle 1 simulation
    mock_isr_callback();
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT8(2, sensor.getPulses());

    // Cycle 2 simulation immediately following the first sample flush
    mock_isr_callback();
    mock_isr_callback();
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT8(3, sensor.getPulses());
}

/**
 * Entry point framework portal mapping for the Linux host machine.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_invalid_pin_binding_rejection);
    RUN_TEST(test_pulse_accumulation_and_flush);
    RUN_TEST(test_subsequent_reading_cycles);
    return UNITY_END();
}