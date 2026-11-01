#include <unity.h>
#include <Arduino.h>
#include <Servo.h>
#include <Choke.h>

// Digital pin 2 is used for the test.
#define TEST_CHOKE_PWM_OUT_PIN 2

// Position 0 should be fairly close to fully engaged.
#define TEST_CHOKE_FULL_POSITION 0

// Position 90 should be fairly close to fully off.
#define TEST_CHOKE_OFF_POSITION 90

// Simulate a 5 second warmup duration.
#define TEST_WARMUP_DURATION 5000

#define TEST_SWEEP_DURATION 0

/**
 * Unit test setup method.
 */
void setUp(void) {
  // The infamous timestamp when the Y2k38 bug destroys the world.
  mock_millis = 2147483647;
}

/**
 * Unit test teardown method.
 */
void tearDown(void) {

}

/**
 * Initially, and by default, the choke position must be OFF. This acts as a
 * fail-safe to prevent dumping excess fuel into the engine in the event of an
 * unexpected failure.
 */
void test_choke_initial_state(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * At time zero, the choke should move to the full position.
 */
void test_choke_starts_at_zero(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  choke.update(); 
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());
}

/**
 * As time passes, the choke should gradually turn off.
 */
void test_choke_lifecycle_completion(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // The choke should be off by default.
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  // At time zero, the choke moves to full.
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Delay for half of the warmup duration.
  delay(TEST_WARMUP_DURATION / 2);

  // Half way through the warmup duration should result in midpoint engagement.
  choke.update(); 
  TEST_ASSERT_EQUAL_UINT8((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) / 2, choke.getPosition());

  // Delay for the other half of the warmup duration.
  delay(TEST_WARMUP_DURATION / 2);

  // The choke should now be all the way off.
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  // Simulate an engine stall and re-start condition.
  choke.reset();

  // The choke should now be fully engaged.
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Delay for half of the warmup duration.
  delay(TEST_WARMUP_DURATION / 2);

  // Half way through the warmup duration should result in midpoint engagement.
  choke.update(); 
  TEST_ASSERT_EQUAL_UINT8((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) / 2, choke.getPosition());

  // Delay for the other half of the warmup duration.
  delay(TEST_WARMUP_DURATION / 2);

  // The choke should now be all the way off.
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * Edge case for when a consumer defines no warmup duration. The expectation is
 * that the choke will immediately transition to fully off after one update
 *  frame.
 */
void test_choke_zero_warmup_duration(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, 0, TEST_SWEEP_DURATION);
  
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * Tests 32 bit overflow handling.
 */
void test_choke_millis_overflow_handling(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // Stage the clock 2 seconds before the absolute maximum limit of uint32_t.
  mock_millis = 4294967295UL - 2000;
  
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Simulate an overflow (the next update happens 3 seconds later).
  mock_millis = 999;
  choke.update();

  // Ensure the math holds up across the wrap-around boundary without breaking.
  uint8_t expected_midpoint = TEST_CHOKE_FULL_POSITION + 
    ((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) * 3000 / TEST_WARMUP_DURATION);
  
  TEST_ASSERT_EQUAL_UINT8(expected_midpoint, choke.getPosition());
}

/**
 * Tests that redundant writes are not made to the output servo.
 */
void test_choke_avoids_redundant_servo_writes(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  delay(TEST_WARMUP_DURATION);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());  

  // Wait for another warmup cycle.
  delay(TEST_WARMUP_DURATION);
  choke.update();

  // Ensure the position has not moved.
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());  
}

void test_choke_mid_cycle_reset_retrigger(void) {
  Choke choke(TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Simulate mid-sequence.
  delay(TEST_WARMUP_DURATION / 2);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) / 2, choke.getPosition());

  // Trigger an unexpected mid-sequence reset.
  choke.reset();

  // The next update frame should reset the choke to fully engaged.
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());
}

/**
 * Entry-point for the unit test.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_choke_initial_state);
    RUN_TEST(test_choke_starts_at_zero);
    RUN_TEST(test_choke_lifecycle_completion);
    RUN_TEST(test_choke_starts_at_zero);
    RUN_TEST(test_choke_millis_overflow_handling);
    RUN_TEST(test_choke_avoids_redundant_servo_writes);
    RUN_TEST(test_choke_mid_cycle_reset_retrigger);
    return UNITY_END();
}
