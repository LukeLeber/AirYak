#include <unity.h>
#include <Arduino.h>
#include <Tachometer.h>

// Subject under test
Tachometer* tachometer;
const uint8_t TARGET_PIN = 2; // Digital pin 2 is mapped to interrupt 0 in our mock framework

void setUp(void) {
    mock_millis = 1000;
    mock_isr_callback = nullptr;
    // Instantiating tachometer creates its private internal FlywheelMagnetInterrupt object
    tachometer = new Tachometer(TARGET_PIN);
}

void tearDown(void) {
    delete tachometer;
}

/**
 * Asserts the initial baseline zero metrics configuration.
 */
void test_tachometer_initial_state(void) {
    TEST_ASSERT_EQUAL_UINT16(0, tachometer->getSpeed());
}

/**
 * Validates dynamic speed calculations by simulating hardware edge-trigger executions.
 */
void test_tachometer_resampling_calculations(void) {
    TEST_ASSERT_EQUAL_UINT16(0, tachometer->getSpeed());
    TEST_ASSERT_NOT_NULL(mock_isr_callback); // Verify underlying sensor bound the ISR correctly

    // 1. Simulate an interval with 25 pulses over 1000ms (25 RPS * 60 = 1500 RPM)
    for (int i = 0; i < 25; i++) {
        mock_isr_callback();
    }
    delay(1000); // Steps mock_millis forward to 2000
    tachometer->resample();
    TEST_ASSERT_EQUAL_UINT16(1500, tachometer->getSpeed());

    // 2. Simulate a stationary engine state (0 pulses over 1050ms loop cycle time)
    delay(1050); // Steps mock_millis forward to 3050 (Time elapsed = 1050ms)
    tachometer->resample();
    TEST_ASSERT_EQUAL_UINT16(0, tachometer->getSpeed());

    // 3. Simulate high-speed operation with jitter (50 pulses over 1025ms loop cycle time)
    // Precise Calculation: (50 * 60,000) / 1025 = 2926 RPM
    for (int i = 0; i < 50; i++) {
        mock_isr_callback();
    }
    delay(1025); 
    tachometer->resample();
    TEST_ASSERT_EQUAL_UINT16(2926, tachometer->getSpeed());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_tachometer_initial_state);
    RUN_TEST(test_tachometer_resampling_calculations);
    return UNITY_END();
}