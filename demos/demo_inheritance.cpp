/**
 * This demo showcases the use of config_utilities create a Config from ros
 * parameter server. This functionality is only included if ros/node_handle.h
 * is included, i.e. in a ROS library.
 */

#include <iostream>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ros-tools
// (i.e. getConfigFromRos()) are enabled.
#include <ros/node_handle.h>

#include "config_utilities.hpp"

// The method introduced below also works for other configs, so we declare
// another config.
struct OtherConfig : public config_utilities::Config<OtherConfig> {
  float a = 0.f;
  int b = 0;

  OtherConfig() { setConfigName("OtherConfig"); }

 protected:
  // fromRosParam() and printFields() can be combined in
  // setupParamsAndPrinting() to save code duplication. setupParamsAndPrinting()
  // precedes fromRosParam() and printFields() in order of execution, special
  // requirements can thus still be implemented using these.
  void setupParamsAndPrinting() override {
    setupParam("a", &a);
    setupParam("b", &b);
  }

  void checkParams() const override { checkParamGE(a, 0.0f, "a"); }
};

// Define a base class that uses a base config.
class MyBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    bool c = true;
    double d = 0.0;

    // Configs are allowed to contain other configs.
    OtherConfig other_config;

    Config() { setConfigName("MyBase"); }

   protected:
    void setupParamsAndPrinting() override {
      setupParam("c", &c);
      setupParam("d", &d);
      setupParam("other_config", &other_config);
    }

    void checkParams() const override {
      checkParamGE(d, 0.0, "d");

      // Use checkParamConfig() to validate entire configs.
      checkParamConfig(other_config);
    }
  };

  explicit MyBase(const Config& config) : config_(config.checkValid()) {}
  virtual ~MyBase() = default;

 private:
  const Config config_;
};

// Define a derived class that uses its own config.
class MyDerived : public MyBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string e = "e";
    int f = 0;

    // Configs are allowed to contain other configs, so we add the base config.
    MyBase::Config base_config;
    OtherConfig other_config;

    Config() { setConfigName("MyDerivedConfig"); }

   protected:
    void setupParamsAndPrinting() override {
      setupParam("e", &e);
      setupParam("f", &f);
      setupParam("other_config", &other_config);

      // Here we use a sub_namespace 'base' to create the base config from.
      // Leading "/" can be used to specify a global namespace.
      setupParam("base_config", &base_config, "base");
    }

    void checkParams() const override {
      checkParamConfig(base_config);
      checkParamConfig(other_config);
    }
  };

  // We can use the member config to initialize the base class.
  explicit MyDerived(const Config& config)
      : MyBase(config.base_config), config_(config.checkValid()) {}

  void printConfig() const { std::cout << config_.toString() << std::endl; }

 private:
  const Config config_;
};

int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // General settings for all configs that are created after this statement.
  config_utilities::GlobalSettings().default_print_width = 60;
  config_utilities::GlobalSettings().default_print_indent = 20;

  // Setup ros and add some params to the parameter server
  ros::init(argc, argv, "demo_inheritance");
  ros::NodeHandle nh_private("~");
  nh_private.setParam("base/a", 1.0);
  nh_private.setParam("base/b", 2);
  nh_private.setParam("base/c", false);
  nh_private.setParam("base/d", 3.45);
  nh_private.setParam("a", 11.1);
  nh_private.setParam("b", 222);
  nh_private.setParam("e", "Bananas are yellow.");
  nh_private.setParam("f", 6.78);

  ros::NodeHandle nh_global("/");
  nh_global.setParam("test/a", 7.0);
  nh_global.setParam("test/b", 7);
  nh_global.setParam("test/d", 7.0);

  // We can as usual get the derived config and instantiate a derived object.
  MyDerived::Config config =
      config_utilities::getConfigFromRos<MyDerived::Config>(nh_private);
  MyDerived derived(config);
  derived.printConfig();

  // Invalidate a member config to provoke an invalid check.
  config.base_config.other_config.a = -1.0f;
  config.checkValid();

  return 0;
}
