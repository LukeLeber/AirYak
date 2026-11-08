#include <unity.h>
#include <Arduino.h>
#include <AnalogSensor.h>

void setUp(void) {
  mock_analog_value = 0;
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
 * Verifies that the first read seeds the filter directly without lag.
 */
void test_analog_sensor_initial_seed_reading(void) {
  AnalogSensor sensor(2);
  mock_analog_value = 1000;
  TEST_ASSERT_EQUAL_UINT16(1000, sensor.read());
}

/**
 * Verifies upward exponential smoothing with sensitivity_factor = 2 (shift by 2, alpha = 0.25).
 */
void test_analog_sensor_ema_upward_smoothing(void) {
  AnalogSensor sensor(2, 2);
  mock_analog_value = 100;
  TEST_ASSERT_EQUAL_UINT16(100, sensor.read()); // Seed: 100

  mock_analog_value = 500;
  // 100 + ((500 - 100) >> 2) = 100 + 100 = 200
  TEST_ASSERT_EQUAL_UINT16(200, sensor.read());
}

/**
 * Verifies 50/50 exponential smoothing with sensitivity_factor = 1 (shift by 1, alpha = 0.5).
 */
void test_analog_sensor_ema_factor_one(void) {
  AnalogSensor sensor(2, 1);
  mock_analog_value = 200;
  TEST_ASSERT_EQUAL_UINT16(200, sensor.read()); // Seed: 200

  mock_analog_value = 600;
  // 200 + ((600 - 200) >> 1) = 200 + 200 = 400
  TEST_ASSERT_EQUAL_UINT16(400, sensor.read());

  mock_analog_value = 100;
  // 400 + ((100 - 400) >> 1) = 400 + (-300 >> 1) = 400 - 150 = 250
  TEST_ASSERT_EQUAL_UINT16(250, sensor.read());
}

/**
 * Regression test: verifies downward smoothing does not underflow unsigned integers.
 */
void test_analog_sensor_ema_downward_no_underflow(void) {
  AnalogSensor sensor(2, 2);
  mock_analog_value = 500;
  TEST_ASSERT_EQUAL_UINT16(500, sensor.read()); // Seed: 500

  mock_analog_value = 100;
  // 500 + ((100 - 500) >> 2) = 500 + (-400 >> 2) = 500 - 100 = 400
  TEST_ASSERT_EQUAL_UINT16(400, sensor.read());
}

/**
 * Verifies sensitivity_factor = 0 acts as an unsmoothed raw pass-through.
 */
void test_analog_sensor_raw_passthrough(void) {
  AnalogSensor sensor(2, 0);
  mock_analog_value = 800;
  TEST_ASSERT_EQUAL_UINT16(800, sensor.read());

  mock_analog_value = 200;
  TEST_ASSERT_EQUAL_UINT16(200, sensor.read());
}

/**
 * Entry point framework portal mapping for the Linux host machine.
 */
int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_analog_sensor_initial_state);
  RUN_TEST(test_analog_sensor_initial_seed_reading);
  RUN_TEST(test_analog_sensor_ema_upward_smoothing);
  RUN_TEST(test_analog_sensor_ema_factor_one);
  RUN_TEST(test_analog_sensor_ema_downward_no_underflow);
  RUN_TEST(test_analog_sensor_raw_passthrough);
  return UNITY_END();
}