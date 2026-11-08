#include <unity.h>
#include <Arduino.h>
#include <FlywheelMagnetInterrupt.h>

void setUp(void) {
    mock_millis = 1000;
    mock_micros = 1000000UL;
    mock_isr_callback = nullptr;
}

void tearDown(void) {
}

/**
 * Verifies that an invalid pin input handles initialization safely and
 * does not register an ISR callback.
 */
void test_invalid_pin_binding_rejection(void) {
    FlywheelMagnetInterrupt invalidSensor(99);
    TEST_ASSERT_NULL(mock_isr_callback);
    TEST_ASSERT_EQUAL_UINT32(0, invalidSensor.getPeriodMicros());
}

/**
 * Verifies that before two pulses arrive, getPeriodMicros() safely returns 0.
 */
void test_requires_two_pulses_for_period(void) {
    FlywheelMagnetInterrupt sensor(3);
    TEST_ASSERT_NOT_NULL(mock_isr_callback);

    // Zero pulses: must return 0
    TEST_ASSERT_EQUAL_UINT32(0, sensor.getPeriodMicros());

    // First pulse only sets the initial timestamp anchor; period is still 0
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT32(0, sensor.getPeriodMicros());
}

/**
 * Verifies exact microsecond period calculation across consecutive pulses.
 */
void test_period_measurement_accuracy(void) {
    FlywheelMagnetInterrupt sensor(3);

    // Pulse 1 at t = 1,000,000 us
    mock_isr_callback();

    // Advance 30,000 us (2,000 RPM) and trigger Pulse 2
    delayMicroseconds(30000UL);
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT32(30000UL, sensor.getPeriodMicros());

    // Advance 16,666 us (~3,600 RPM) and trigger Pulse 3
    delayMicroseconds(16666UL);
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT32(16666UL, sensor.getPeriodMicros());
}

/**
 * Verifies that exceeding the 500,000 us (500 ms) stall timeout returns 0.
 */
void test_stall_timeout_detection(void) {
    FlywheelMagnetInterrupt sensor(3);

    mock_isr_callback();
    delayMicroseconds(40000UL); // 1,500 RPM
    mock_isr_callback();
    TEST_ASSERT_EQUAL_UINT32(40000UL, sensor.getPeriodMicros());

    // Advance exactly 500,000 us (boundary: still valid)
    delayMicroseconds(500000UL);
    TEST_ASSERT_EQUAL_UINT32(40000UL, sensor.getPeriodMicros());

    // Advance 1 more microsecond past 500,000 us -> stalled
    delayMicroseconds(1UL);
    TEST_ASSERT_EQUAL_UINT32(0, sensor.getPeriodMicros());
}

/**
 * Verifies 32-bit unsigned rollover handling (~71.58 minutes uptime).
 */
void test_micros_32bit_rollover(void) {
    FlywheelMagnetInterrupt sensor(3);

    // Position 10,000 us before uint32_t max (4,294,967,295)
    mock_micros = 4294967295UL - 10000UL;
    mock_isr_callback();

    // Advance 30,000 us across the overflow boundary
    mock_micros = 19999UL;
    mock_isr_callback();

    TEST_ASSERT_EQUAL_UINT32(30000UL, sensor.getPeriodMicros());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_invalid_pin_binding_rejection);
    RUN_TEST(test_requires_two_pulses_for_period);
    RUN_TEST(test_period_measurement_accuracy);
    RUN_TEST(test_stall_timeout_detection);
    RUN_TEST(test_micros_32bit_rollover);
    return UNITY_END();
}