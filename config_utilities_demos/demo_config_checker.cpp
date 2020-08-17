/**
 * This demo showcases the use of config_utilities to check independent structs
 * for compatibility in a clean and reabable way.
 */

#include <iostream>
#include <string>

#include "../config_utilities.hpp"

// Define a struct that is not a config_utilities::Config.
struct IndependentConfig {
  int a = 0;
  double b = 0.0;
  std::string c = "this is c";

  // Use the config checker to check all params of the config.
  [[nodiscard]] bool isValid(bool print_warnings = false) const {
    // Create the checker with the config name.
    config_utilities::ConfigChecker checker("IndependentConfig");

    // Use the checker to verify various conditions.
    checker.checkGE(a, 0, "a");
    checker.checkEq(c, std::string("this is c"), "c");

    // Any condition can be implemented using checkCond with a corresponding
    // error message.
    checker.checkCond(static_cast<int>(b) >= a, "b is expected >= a.");

    // Return the summary.
    return checker.isValid(print_warnings);
  };

  // A function that guarantees validity and exits the program otherwise.
  void checkValid() const { CHECK(isValid(true)); };
};

int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Create a valid (default) config.
  IndependentConfig config;

  config.checkValid();  // This should simply pass.

  // Print the result.
  std::cout << "Result: 'config' was "
            << (config.isValid() ? "valid" : "invalid") << std::endl;

  // Now change the config s.t. it is invalid.
  config.a = -1;
  config.b = 0;
  config.c = "test";

  config.checkValid();  // This should exit with a failed check, but raise a
  // warning for every wrong paramter first.

  return 0;
}
