/**
 * This demo showcases the use of config_utilities to configure global settings
 * and printing.
 */

#include <iostream>
#include <string>

#include "config_utilities.hpp"

// Define several dummy configs.
struct ConfigA : public config_utilities::Config<ConfigA> {
  float a = 123.f;
  std::string aa = "config a text";

  ConfigA() { setConfigName("ConfigA"); }

 protected:
  void setupParamsAndPrinting() override {
    // The unit of a is in meters [m]. This can be specified as optional third
    // argument.
    const std::string unit = "m";
    setupParam("a", &a, unit);
    setupParam("aa", &aa);
  }
};

struct ConfigB : public config_utilities::Config<ConfigB> {
  int b = 30;
  std::string bb = "config b text";

  ConfigB() { setConfigName("ConfigB"); }

 protected:
  void setupParamsAndPrinting() override {
    setupParam("b", &b, "Hz");
    setupParam("bb", &bb);
  }
};

struct ConfigC : public config_utilities::Config<ConfigC> {
  bool c = true;

  ConfigA config_a;
  ConfigB config_b;

  ConfigC() { setConfigName("ConfigC"); }

 protected:
  void setupParamsAndPrinting() override {
    setupParam("config_a", &config_a);
    setupParam("config_b", &config_b);
  }
};

int main(int argc, char** argv) {
  // Setup logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Create several configs.
  ConfigA config_a;
  ConfigB config_b;
  ConfigC config_c;
  config_c.config_b.b = 45;
  config_c.config_b.bb = "varied config b text";

  // print the values in config C.
  std::cout << config_c.toString() << std::endl;

  // General settings can be set dynamically in the global settings.
  auto& settings = config_utilities::Global::Settings();
  settings.print_width = 40;                 // default: 80
  settings.print_indent = 15;                // default: 3
  settings.subconfig_indent = 10;            // default: 30
  settings.indicate_default_values = false;  // default: true
  settings.indicate_units = false;           // default: true

  // Reprint the config and see it take effect.
  std::cout << config_c.toString() << std::endl;

  // We can collect information about all configs that exist using 'Global'.
  // 'printAllConfigs()' will create a compact list of all currently existing
  // configs. Each config (such as the member configs of config_c) will only be
  // printed once. This tool gives quick information about the realized params
  // in all configs or can be used for logging.
  const std::string global_configuration =
      config_utilities::Global::printAllConfigs();
  std::cout << "\nValue of all existing params: \n"
            << global_configuration << std::endl;

  return 0;
}
