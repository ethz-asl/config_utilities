#include <gtest/gtest.h>
#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace config_utilities {
namespace test {

// Helper functions.

std::string execute_command(std::string command) {
  char buffer[128];
  std::string result = "";

  // Open pipe to file
  FILE* pipe = popen(command.c_str(), "r");
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

class RosCore {
 public:
  RosCore() { ros_core_process_ = popen("roscore", "r"); }
  ~RosCore() { pclose(ros_core_process_); }

 private:
  FILE* ros_core_process_;
};

TEST(Demos, ConfigChecker) {
  std::string result = execute_command("rosrun config_utilities demo_config");
  const std::string expected_result(
      "================= MyClass-Config =================\na:             "
      "1\nb:             2.34\nb_half:        1.17\nc:             this is "
      "c\nAn_extremely_unecessarily_and_unreasonably_long_pa\nram_name:      "
      "A_similarly_unreasonably_long_param\n               _value.\nAnd a "
      "custom message.\n==================================================\n");

  EXPECT_EQ(result, expected_result);
}

void test() {
  RosCore core();
  std::string result =
      execute_command("roscore & rosrun config_utilities demo_ros_param");
  std::cout << "TEEEEEEEST\n" << result << std::endl;
  // std::cout << "Test: " << (result == expected_result) << std::endl;
}

}  // namespace test
}  // namespace config_utilities

int main(int argc, char** argv) {
  // testing::InitGoogleTest(&argc, argv);
  // return RUN_ALL_TESTS();
  config_utilities::test::test();
}