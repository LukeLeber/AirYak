#include <unity.h>
#include <Arduino.h>
#include <Tachometer.h>

const uint8_t TARGET_PIN = 3;

void setUp(void) {
    mock_millis = 1000;
    mock_micros = 1000000UL;
    mock_isr_callback = nullptr;
}

void tearDown(void) {
}

/**
 * Asserts the initial baseline zero metrics configuration.
 */
void test_tachometer_initial_state(void) {
    Tachometer tachometer(TARGET_PIN);
    TEST_ASSERT_EQUAL_UINT16(0, tachometer.getSpeed());
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(0, tachometer.getSpeed());
}

/**
 * Validates initial RPM calculation from a 40,000 us period (1,500 RPM).
 * When starting from 0 RPM, the filter seeds directly to raw_rpm without lag.
 */
void test_tachometer_initial_speed_calculation(void) {
    Tachometer tachometer(TARGET_PIN);
    TEST_ASSERT_NOT_NULL(mock_isr_callback);

    // Pulse 1 sets the initial timestamp anchor
    mock_isr_callback();
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(0, tachometer.getSpeed());

    // Pulse 2 arrives 40,000 us later -> 60,000,000 / 40,000 = 1,500 RPM
    delayMicroseconds(40000UL);
    mock_isr_callback();
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(1500, tachometer.getSpeed());
}

/**
 * Validates the 75/25 EMA smoothing calculation when engine speed changes.
 */
void test_tachometer_ema_smoothing(void) {
    Tachometer tachometer(TARGET_PIN);

    // Seed initial speed at 1,500 RPM (40,000 us period)
    mock_isr_callback();
    delayMicroseconds(40000UL);
    mock_isr_callback();
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(1500, tachometer.getSpeed());

    // Step to 3,000 RPM (20,000 us period)
    // First resample: ((1500 * 3) + 3000) / 4 = 7500 / 4 = 1875 RPM
    delayMicroseconds(20000UL);
    mock_isr_callback();
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(1875, tachometer.getSpeed());

    // Second resample on same period: ((1875 * 3) + 3000) / 4 = 8625 / 4 = 2156 RPM
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(2156, tachometer.getSpeed());
}

/**
 * Validates that exceeding the 500 ms stall timeout drops reported speed to 0 RPM.
 */
void test_tachometer_stall_detection(void) {
    Tachometer tachometer(TARGET_PIN);

    // Establish 2,000 RPM (30,000 us period)
    mock_isr_callback();
    delayMicroseconds(30000UL);
    mock_isr_callback();
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(2000, tachometer.getSpeed());

    // Advance past the 500,000 us (500 ms) stall threshold without new pulses
    delayMicroseconds(500001UL);
    tachometer.resample();
    TEST_ASSERT_EQUAL_UINT16(0, tachometer.getSpeed());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_tachometer_initial_state);
    RUN_TEST(test_tachometer_initial_speed_calculation);
    RUN_TEST(test_tachometer_ema_smoothing);
    RUN_TEST(test_tachometer_stall_detection);
    return UNITY_END();
}