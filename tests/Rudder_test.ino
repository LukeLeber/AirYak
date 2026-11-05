#include <unity.h>
#include <Arduino.h>
#include <Servo.h>
#include <Rudder.h>

// Global mock variable provided by tests/mock_arduino/Arduino.cpp
extern uint16_t mock_analog_value;

#define HARD_PORT 0
#define HALF_PORT 45
#define STRAIGHT_ON 90
#define HALF_STARBOARD 135
#define HARD_STARBOARD 180


void setUp(void) {
  mock_analog_value = 512; 
}

void tearDown(void) {

}

void test_rudder_default_position(void) {
  Rudder rudder(4, 5, 0);
  TEST_ASSERT_EQUAL_UINT8(STRAIGHT_ON, rudder.getPosition());
}

void test_rudder_hard_port(void) {
  Rudder rudder(4, 5, 0);

  // map(HARD_PORT, 0, 180, 0, 1023) = 0
  mock_analog_value = 0;

  // Steering should be gradual over 8 samples due to moving average filter
  const uint8_t expected_path[] = { 79, 68, 56, 45, 34, 23, 11, 0 };
  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  // Ensure that subsequent updates do not attempt to oversteer
  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HARD_PORT, rudder.getPosition());  
}

void test_rudder_half_port(void) {
  Rudder rudder(4, 5, 0);

  // map(HALF_PORT, 0, 180, 0, 1023) = 255
  mock_analog_value = 255;
  const uint8_t expected_path[] = { 84, 79, 73, 68, 62, 56, 51, 45 };

  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  // Verify subsequent updates do not oversteer
  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HALF_PORT, rudder.getPosition());
}

void test_rudder_hard_starboard(void) {
  Rudder rudder(4, 5, 0);

  // map(HARD_STARBOARD, 0, 180, 0, 1023) = 1023
  mock_analog_value = 1023;

  // Steering should be gradual over 8 samples
  const uint8_t expected_path[] = { 101, 113, 124, 135, 146, 158, 169, 180 };
  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  // Ensure subsequent updates do not attempt to oversteer
  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HARD_STARBOARD, rudder.getPosition());
}

void test_rudder_half_starboard(void) {
  Rudder rudder(4, 5, 0);
  // map(HALF_STARBOARD, 0, 180, 0, 1023) = 767
  mock_analog_value = 767;
  const uint8_t expected_path[] = { 96, 101, 107, 113, 118, 124, 129, 135 };

  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  // Verify subsequent updates do not oversteer
  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HALF_STARBOARD, rudder.getPosition());
}

void test_rudder_full_port_to_full_starboard(void) {
  Rudder rudder(4, 5, 0);
  mock_analog_value = 0; // HARD_PORT
  for(uint8_t i = 0; i < 8; ++i) {
    rudder.update();
  }
  TEST_ASSERT_EQUAL_UINT8(HARD_PORT, rudder.getPosition());
  
  mock_analog_value = 1023; // HARD_STARBOARD
  const uint8_t expected_path[] = { 23, 45, 68, 90, 113, 135, 158, 180 };

  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HARD_STARBOARD, rudder.getPosition());
}

void test_rudder_full_starboard_to_full_port(void) {
  Rudder rudder(4, 5, 0);
  mock_analog_value = 1023; // HARD_STARBOARD
  for(uint8_t i = 0; i < 8; ++i) {
    rudder.update();
  }
  TEST_ASSERT_EQUAL_UINT8(HARD_STARBOARD, rudder.getPosition());
  
  mock_analog_value = 0; // HARD_PORT
  const uint8_t expected_path[] = { 158, 135, 113, 90, 68, 45, 23, 0 };

  for (uint8_t expected_position : expected_path) {
    rudder.update();
    TEST_ASSERT_EQUAL_UINT8(expected_position, rudder.getPosition());
  }

  rudder.update();
  TEST_ASSERT_EQUAL_UINT8(HARD_PORT, rudder.getPosition());
}

// --- Dynamic Host Switching Portal ---
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rudder_default_position);
    RUN_TEST(test_rudder_hard_port);
    RUN_TEST(test_rudder_half_port);
    RUN_TEST(test_rudder_half_starboard);
    RUN_TEST(test_rudder_hard_starboard);
    RUN_TEST(test_rudder_full_port_to_full_starboard);
    RUN_TEST(test_rudder_full_starboard_to_full_port);
    return UNITY_END();
}