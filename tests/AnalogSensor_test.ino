#include <unity.h>
#include <Arduino.h>
#include <AnalogSensor.h>


void setUp(void) {
}

void tearDown(void) {
}

/**
 * Asserts the initial baseline zero metrics configuration.
 */
void test_analog_sensor_initial_state(void) {
  AnalogSensor sensor(2);
  TEST_ASSERT_EQUAL_UINT16(0, sensor.read());
}

/**
 * Verifies that the sensor accurately reads.
 */
void test_analog_sensor_reading(void) {
  AnalogSensor sensor(2);
  mock_analog_value = 1024;  
  TEST_ASSERT_EQUAL_UINT16(1024, sensor.read());
}

/**
 * Entry point framework portal mapping for the Linux host machine.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_analog_sensor_initial_state);
    RUN_TEST(test_analog_sensor_reading);
    return UNITY_END();
}