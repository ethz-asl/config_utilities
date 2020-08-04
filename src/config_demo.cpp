#include <iostream>
#include <string>

#include "config_utilities/config_utilities.hpp"

// Define a class that uses a config.
class MyClass {
 public:
  // Config definition that inherits from config_utility and templates itself.
  struct Config : public config_utilities::Config<Config> {
    int a = 0;
    double b = 123.4;
    double b_half = -1;
    std::string c = "this is c";

    // Check params is mandatory to implement but can be left empty if any
    // config is valid. Otherwise use the checkParam tools for verification.
    void checkParams() const override {
      checkParamGE(a, 0, "a");
      checkParamEq(c, std::string("this is c"), "c");
      checkParamCond(static_cast<int>(b) > a, "b is expected > a.");
    }

    // Optionally printing can be implemented using the printX tools.
    void printFields() const override {
      printField(a, "a");
      printField(b, "b");
      printField(b_half, "b_half");
      printField(c, "c");
      printText("And a custom message.");
    }

    // Dependent default arguments can be set by overriding this function.
    void initializeDependentVariableDefaults() override {
      if (b_half == -1) {
        b_half = b / 2.0;
      }
    }

    // Optional other fields can be set in the constructor.
    Config() {
      setName("MyClass-Config");
      setPrintWidth(40);
      setPrintIndent(20);
    }
  };

  // Use the config for construction.
  explicit MyClass(const Config& config) : config_(config.checkValid()) {}

  void print() const {
    std::cout << config_.toString() << std::endl;
  }

 private:
  const Config config_;
};

int main(int argc, char** argv) {
  // Warnings are printed using GLOG, make sure to run with "-alsologtostderr".
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Create a valid (default) config.
  MyClass::Config valid_config;

  MyClass my_class(valid_config);
  my_class.print();

  // Create an invalid config.
  MyClass::Config invalid_config;
  invalid_config.a = -1;
  invalid_config.b = -3;
  invalid_config.c = "test";

  MyClass another_instance(invalid_config);
  return 0;
}
