/**
 * This demo showcases the use of config_utilities variable configs to create
 * variable downstream modules.
 */

#include <iostream>
#include <memory>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ROS-tools,
// i.e. FactoryROS::create(), are enabled.
#include <ros/node_handle.h>

#include "config_utilities.hpp"

// Define a common base class.
class Base {
 public:
  // Base and derived classes are allowed to have arbitrary constructor
  // arguments.
  explicit Base(int data) : data_(data) {}

  // Interface for the operation that this component implements.
  virtual void print() const = 0;

 protected:
  const int data_;
};

// Define derived classes.
class DerivedA : public Base {
 public:
  // Define the config. Only classes having a 'Config' member can be created
  // using the ROS factory.
  struct Config : public config_utilities::Config<Config> {
    int i = 0;

    Config() { setConfigName("DerivedA Config"); }

   protected:
    void setupParamsAndPrinting() override { setupParam("i", &i); }
  };

  // The constructor that is registered to the factory is required to take the
  // config as the first argument.
  DerivedA(const Config& config, int data)
      : Base(data), config_(config.checkValid()) {}

  void print() const override {
    std::cout << "This is a DerivedA with i='" << config_.i
              << "' and base data='" << data_ << "'." << std::endl;
  }

 private:
  const Config config_;

  // Register to the factory using the 'RegistrationRos' struct. Notice that the
  // config does not need to be specified, the order of template arguments is
  // the Base, itself, followed by all constructor arguments.  Multiple
  // registrations can be used to register different constructors if necessary.
  static config_utilities::Factory::RegistrationRos<Base, DerivedA, int>
      registration;
};

config_utilities::Factory::RegistrationRos<Base, DerivedA, int>
    DerivedA::registration("DerivedA");

// Similarly, a seond derived class using a different config.
class DerivedB : public Base {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string s = "default text";

    Config() { setConfigName("DerivedB Config"); }

   protected:
    void setupParamsAndPrinting() override { setupParam("s", &s); }
  };

  DerivedB(const Config& config, int data)
      : Base(data), config_(config.checkValid()) {}

  void print() const override {
    std::cout << "This is a DerivedB with s='" << config_.s
              << "' and base data='" << data_ << "'." << std::endl;
  }

 private:
  const Config config_;

  static config_utilities::Factory::RegistrationRos<Base, DerivedB, int>
      registration;
};

config_utilities::Factory::RegistrationRos<Base, DerivedB, int>
    DerivedB::registration("DerivedB");

// Finally, we define the upstream object that contains a variable config of
// base.
class Object {
 public:
  struct Config : public config_utilities::Config<Config> {
    int i = 5;
    config_utilities::VariableConfig<Base> base_config;

    Config() { setConfigName("Object Config"); }

   protected:
    void setupParamsAndPrinting() override {
      setupParam("i", &i);
      // Setting up variable configs can be done identically to other configs,
      // using a sub-namespace if desired.
      setupParam("base_config", &base_config, "sub_conf");
    }

    void checkParams() const override {
      checkParamGE(i, 0, "i");
      // Regular checkParamConfig() can be used to guarantee the variable config
      // is valid. Configs that are not setup (e.g. have no type param set) are
      // invalid. If this is undesired behavior, base_config.isSetup() can be
      // used to check for this case.
      checkParamConfig(base_config);
    }
  };

  explicit Object(const Config& config) : config_(config.checkValid()) {
    // Use the variable config to create an object of type base, as specified by
    // the variable config arguments. Other constructor arguments can be passed
    // here.
    base_ptr_ = config.base_config.create(config.i);
  }

  void print() const {
    // Delegate the print request.
    if (base_ptr_) {
      base_ptr_->print();
    }
  }

 private:
  const Config config_;
  std::unique_ptr<Base> base_ptr_;
};

int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server.
  ros::init(argc, argv, "demo_variable_config");
  ros::NodeHandle nh_private("~");

  // The paramter 'type' is used to identify the object to be created.
  nh_private.setParam("sub_conf/type", "DerivedB");
  nh_private.setParam("sub_conf/i", 1);
  nh_private.setParam("sub_conf/s", "text for derived B.");
  nh_private.setParam("i", 10);

  // Create the target object from ROS params.
  Object::Config config;

  // Print what is in the config, the variable config should be unset.
  std::cout << config.toString() << std::endl;
  std::cout << "Config is valid: " << (config.isValid() ? "true" : "false")
            << std::endl;

  // Now let's setup the config from ROS.
  config = config_utilities::getConfigFromRos<Object::Config>(nh_private);
  std::cout << config.toString() << std::endl;
  std::cout << "Config is valid: " << (config.isValid() ? "true" : "false")
            << std::endl;

  // Create the Object that uses a Base-component and verify the correct base
  // component was created.
  Object object(config);
  object.print();

  return 0;
}
