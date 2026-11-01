#include <unity.h>
#include <Arduino.h>
#include <EcoServo.h>

void setUp(void) {
    mock_millis = 1000;
    mock_isr_callback = nullptr;
}

void tearDown(void) {

}

void test_eco_servo(void) {
  EcoServo servo(1, 0, 100);

  // Eco-mode should be disabled by default with the default position.
  TEST_ASSERT_FALSE(servo.isEcoMode());
  TEST_ASSERT_EQUAL_UINT16(0, servo.getPosition());

  // Advance time 50 milliseconds.
  mock_millis += 50;
  servo.revalidateEcoMode();

  // Eco mode should still be disabled.
  TEST_ASSERT_FALSE(servo.isEcoMode());

  // Advance time 50 milliseconds.
  mock_millis += 50;
  servo.revalidateEcoMode();

  // Eco mode should now be enabled.
  TEST_ASSERT_TRUE(servo.isEcoMode());

  // Update the position to disable eco mode.
  servo.setPosition(90);
  TEST_ASSERT_FALSE(servo.isEcoMode());
  TEST_ASSERT_EQUAL_UINT16(90, servo.getPosition());

  // Advance time 50 milliseconds.
  mock_millis += 50;
  servo.revalidateEcoMode();
  TEST_ASSERT_FALSE(servo.isEcoMode());

  // Update the position to reset the eco mode timer.
  servo.setPosition(45);

  // Advance time 50 milliseconds, eco mode still disabled.
  mock_millis += 50;
  servo.revalidateEcoMode();
  TEST_ASSERT_FALSE(servo.isEcoMode());

  // Advance time 50 milliseconds, eco mode is now enabled.
  mock_millis += 50;
  servo.revalidateEcoMode();
  TEST_ASSERT_TRUE(servo.isEcoMode());

  // Setting the position as a no-op should not disable eco mode.
  servo.setPosition(45);
  TEST_ASSERT_TRUE(servo.isEcoMode());
}

/**
 * Entry point framework portal mapping for the Linux host machine.
 */
int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_eco_servo);
  return UNITY_END();
}