#include <gtest/gtest.h>
#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace config_utilities {
namespace test {

// Helper functions.
// Run a unix command and get the output as string.
std::string execute_command(const std::string& command) {
  char buffer[128];
  std::string result;

  // Also get stderr.
  const std::string full_command = command + " 2>&1";

  // Open pipe to file.
  FILE* pipe = popen(full_command.c_str(), "r");
  if (!pipe) {
    return "popen failed!";
  }

  // Read till end of process:
  while (!feof(pipe)) {
    if (fgets(buffer, 128, pipe) != NULL) result += buffer;
  }

  pclose(pipe);
  return result;
}

// Wrapper to run an executable if no ROS environment is needed.
std::string run_node(const std::string& node_name) {
  return execute_command("rosrun config_utilities " + node_name);
}

// Wrapper to roslaunch an executable if ROS environment is needed.
std::string launch_node(const std::string& node_name) {
  return execute_command("roslaunch config_utilities test.launch name:=" +
                         node_name);
}

// Check for complicated strings whether they contain all expected substrings.
void expect_contains(const std::string& result,
                     const std::vector<std::string>& expected_results) {
  for (const std::string& expected : expected_results) {
    if (result.find(expected) == std::string::npos) {
      FAIL() << "Not Found:\n" + expected;
    }
  }
}

TEST(Demos, ConfigChecker) {
  const std::string result = run_node("demo_config_checker");
  const std::vector<std::string> expected_results = {
      "Result: 'config' was valid",
      "============================== IndependentConfig "
      "===============================\nWarning: Param 'a' is expected >= '0' "
      "(is: '-1').\nWarning: Param 'c' is expected to be 'this is c' (is: "
      "'test').\n=============================================================="
      "==================",
      "Check failed: isValid(true)"};
  expect_contains(result, expected_results);
}

TEST(Demos, Config) {
  const std::string result = run_node("demo_config");
  const std::vector<std::string> expected_results = {
      "================================ MyClass-Config "
      "================================\na:                            1 "
      "(default)\nb:                            2.34 (default)\nb_half:        "
      "               1.17 (default)\nc:                            this is c "
      "(default)\nAn_extremely_unecessarily_and_unreasonably_long_param_name: "
      "\n                              "
      "A_similarly_unreasonably_long_param_value. (defaul\n                    "
      "          t)\nAnd a custom "
      "message.\n=============================================================="
      "==================",
      "================================ MyClass-Config "
      "================================\nWarning: Param 'a' is expected >= '0' "
      "(is: '-1').\nWarning: Param 'c' is expected to be 'this is c' (is: "
      "'test').\nWarning: b is expected > "
      "a.\n===================================================================="
      "============",
      "Check failed: isValid(true)"};
  expect_contains(result, expected_results);
}

TEST(Demos, RosParam) {
  const std::string result = launch_node("demo_ros_param");
  const std::vector<std::string> expected_results = {
      "=========================== Config (from ROS params) "
      "===========================\na:                            123\nb:      "
      "                      45.6\nc:                            "
      "seven-eight-nine\nvec:                          [1, 2, 3]\nmap:         "
      "                 {m1: 1, m2: 2}\nT:                            t: [0, "
      "0, 0] RPY°: [-0, 0, -0] (default)\nnamespace:                    "
      "/test\n============================================================"
      "====================",
      "'printField()' calls are only allowed within the 'printFields()' "};
  expect_contains(result, expected_results);
}

TEST(Demos, Inheritance) {
  const std::string result = launch_node("demo_inheritance");
  const std::vector<std::string> expected_results = {
      "===================== MyDerivedConfig ======================\ne:        "
      "          Bananas are yellow.\nf:                  6\nother_config:\n   "
      "a:               11.1\n   b:               222\nbase_config:\n   c:     "
      "          False\n   d:               3.45\n   other_config:\n      a:   "
      "         1\n      b:            "
      "2\n============================================================",
      "======================= OtherConfig ========================\nWarning: "
      "Param 'a' is expected >= '0' (is: "
      "'-1').\n============================================================",
      "========================== MyBase "
      "==========================\nWarning: Member config 'OtherConfig' is not "
      "valid.\n============================================================",
      "===================== MyDerivedConfig "
      "======================\nWarning: Member config 'MyBase' is not "
      "valid.\n============================================================",
      "Check failed: isValid(true)"};
  expect_contains(result, expected_results);
}

TEST(Demos, Factory) {
  const std::string result = run_node("demo_factory");
  const std::vector<std::string> expected_results = {
      "This is a DerivedA with i=0, f=0.\nThis is a DerivedB with i=1, f=2.",
      "No module of type 'DerivedC' registered to the factory for base '4Base' "
      "and constructor arguments 'i, f'. Registered are: DerivedB, DerivedA.",
      "'object' is invalid."};
  expect_contains(result, expected_results);
}

TEST(Demos, VariableConfig) {
  const std::string result = launch_node("demo_variable_config");
  const std::vector<std::string> expected_results = {
      "================================ Object Config "
      "=================================\ni:                            5 "
      "(default)\nbase_config:                  Uninitialized Variable "
      "Config.\n==============================================================="
      "=================\nConfig is valid: "
      "false\n================================ Object Config "
      "=================================\ni:                            "
      "10\nbase_config (Variable Config: DerivedB):\n   s:                     "
      "    text for derived "
      "B.\n===================================================================="
      "============\nConfig is valid: true\nThis is a DerivedB with s='text "
      "for derived B.' and base data='10'."};
  expect_contains(result, expected_results);
}

TEST(Demos, GlobalSettings) {
  const std::string result = run_node("demo_global_settings");
  const std::vector<std::string> expected_results = {
      "=================================== ConfigC "
      "====================================\nconfig_a:\n   a [m]:              "
      "       123 (default)\n   aa:                        config a text "
      "(default)\nconfig_b:\n   b [Hz]:                    45\n   bb:          "
      "              varied config b "
      "text\n=================================================================="
      "==============\n=============== ConfigC ================\nconfig_a:\n   "
      "       a:   123\n          aa:  config a text\nconfig_b:\n          b:  "
      " 45\n          bb:  varied config b "
      "text\n========================================\n\nValue of all existing "
      "params: \n=============== ConfigA ================\na:             "
      "123\naa:            config a text\n=============== ConfigB "
      "================\nb:             30\nbb:            config b "
      "text\n=============== ConfigC ================\nconfig_a:\n          a: "
      "  123\n          aa:  config a text\nconfig_b:\n          b:   45\n     "
      "     bb:  varied config b "
      "text\n========================================"};
  expect_contains(result, expected_results);
}

void print_command_output() {
  // launch_node, run_node
  std::cout << "TEST-START\n"
            << launch_node("demo_ros_param") << "TEST-END" << std::endl;
}

}  // namespace test
}  // namespace config_utilities

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  // config_utilities::test::print_command_output();
}