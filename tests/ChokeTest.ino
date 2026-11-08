#include <unity.h>
#include <Arduino.h>
#include <Servo.h>
#include <Tachometer.h>
#include <Choke.h>

// Digital pin 3 maps to INT1 in our mock framework.
#define TEST_TACH_DIGITAL_IN_PIN 3

// Digital pin 4 is used for the choke servo output.
#define TEST_CHOKE_PWM_OUT_PIN 4

// Position 0 is fully engaged (closed).
#define TEST_CHOKE_FULL_POSITION 0

// Position 90 is fully off (open).
#define TEST_CHOKE_OFF_POSITION 90

// Simulate a 5 second warmup duration.
#define TEST_WARMUP_DURATION 5000

// Instant servo sweep for deterministic unit assertions.
#define TEST_SWEEP_DURATION 0

/**
 * Helper: Simulates a running engine at 2,000 RPM (30,000 us period)
 * and updates the tachometer cache.
 */
static void simulateEngineRunning(Tachometer& tachometer) {
  mock_isr_callback();
  delayMicroseconds(30000UL);
  mock_isr_callback();
  tachometer.resample();
}

/**
 * Unit test setup method.
 */
void setUp(void) {
  mock_micros = 1000000UL;
  mock_isr_callback = nullptr;
}

/**
 * Unit test teardown method.
 */
void tearDown(void) {
}

/**
 * Initially, and by default, the choke position must be OFF while the engine is stationary.
 */
void test_choke_initial_state(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // Calling update() while engine speed is 0 RPM must keep the choke OFF
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * Once the engine starts spinning, the first update() engages full choke.
 */
void test_choke_engages_when_engine_starts(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  simulateEngineRunning(tachometer);
  choke.update();

  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());
}

/**
 * As time passes while the engine is running, the choke gradually opens to OFF.
 */
void test_choke_lifecycle_completion(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // Stationary baseline
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  // Engine starts -> full choke
  simulateEngineRunning(tachometer);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Advance halfway through warmup (keep tachometer cached at running speed)
  delay(TEST_WARMUP_DURATION / 2);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) / 2, choke.getPosition());

  // Advance through the remaining warmup duration -> choke fully OFF
  delay(TEST_WARMUP_DURATION / 2);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * Verifies that if the engine stalls mid-warmup, the choke immediately resets to OFF
 * and clears its warmup timer so a subsequent restart re-engages full choke.
 */
void test_choke_resets_on_engine_stall(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // Start engine and advance 1 second into warmup
  simulateEngineRunning(tachometer);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  delay(1000);
  // Exceed 500 ms stall timeout and resample -> speed drops to 0 RPM
  tachometer.resample();
  TEST_ASSERT_EQUAL_UINT16(0, tachometer.getSpeed());

  // Choke must return to OFF fail-safe position on stall
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());

  // Restart engine -> choke must re-engage FULL from the beginning of the curve
  simulateEngineRunning(tachometer);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());
}

/**
 * Edge case: zero warmup duration transitions immediately to OFF.
 */
void test_choke_zero_warmup_duration(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, 0, TEST_SWEEP_DURATION);

  simulateEngineRunning(tachometer);
  choke.update(); // Seeds engine_start_time at full
  choke.update(); // 0 elapsed is not < 0 warmup_duration -> transitions to OFF
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_OFF_POSITION, choke.getPosition());
}

/**
 * Tests 32-bit unsigned millis() rollover handling.
 */
void test_choke_millis_overflow_handling(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  // Start the engine first so delayMicroseconds() doesn't overwrite mock_millis
  simulateEngineRunning(tachometer);

  // Stage clock 2 seconds before uint32_t max
  mock_millis = 4294967295UL - 2000UL;
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  // Simulate overflow 3 seconds later (without resampling tach as stalled)
  mock_millis = 999UL;
  choke.update();

  uint8_t expected_midpoint = TEST_CHOKE_FULL_POSITION +
    ((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) * 3000 / TEST_WARMUP_DURATION);

  TEST_ASSERT_EQUAL_UINT8(expected_midpoint, choke.getPosition());
}

/**
 * Tests explicit mid-cycle reset() retriggering full choke on next update().
 */
void test_choke_mid_cycle_reset_retrigger(void) {
  Tachometer tachometer(TEST_TACH_DIGITAL_IN_PIN);
  Choke choke(tachometer, TEST_CHOKE_PWM_OUT_PIN, TEST_CHOKE_OFF_POSITION, TEST_CHOKE_FULL_POSITION, TEST_WARMUP_DURATION, TEST_SWEEP_DURATION);

  simulateEngineRunning(tachometer);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());

  delay(TEST_WARMUP_DURATION / 2);
  choke.update();
  TEST_ASSERT_EQUAL_UINT8((TEST_CHOKE_OFF_POSITION - TEST_CHOKE_FULL_POSITION) / 2, choke.getPosition());

  choke.reset();
  choke.update();
  TEST_ASSERT_EQUAL_UINT8(TEST_CHOKE_FULL_POSITION, choke.getPosition());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_choke_initial_state);
  RUN_TEST(test_choke_engages_when_engine_starts);
  RUN_TEST(test_choke_lifecycle_completion);
  RUN_TEST(test_choke_resets_on_engine_stall);
  RUN_TEST(test_choke_zero_warmup_duration);
  RUN_TEST(test_choke_millis_overflow_handling);
  RUN_TEST(test_choke_mid_cycle_reset_retrigger);
  return UNITY_END();
}