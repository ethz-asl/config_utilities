/**
 * This demo showcases the use of config_utilities factory to create varying
 * config-independent types.
 */

#include <iostream>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ROS-tools,
// i.e. FactoryROS::create(), are enabled.
#include <ros/node_handle.h>

#include "../config_utilities.hpp"

// Define a common base class.
class Base {
 public:
  virtual void print() = 0;
};

// Define derived classes.
class DerivedA : public Base {
 public:
  // Define the config. Only classes having a 'Config' member can be created
  // using the ROS factory.
  struct Config : public config_utilities::Config<Config> {
    int i = 0;
    float f = 0.f;

   protected:
    void setupParamsAndPrinting() override {
      setupParam("i", &i);
      setupParam("f", &f);
    }
  };

  // The constructor that is registered to the factory is required to take the
  // config as the first argument.
  DerivedA(const Config& config, const std::string& info) : config_(config.checkValid()), info_(info) {}

  void print() override {
    std::cout << "This is a DerivedA with i=" << config_.i << ", f="<< config_.f << ", and info '" << info_ << "'." <<std::endl;
  }

 private:
  const Config config_;
  const std::string info_;

  // Register to the factory using the 'RegistrationRos' struct. Notice that the config does not need to
  // be specified, the order of template arguments is the Base, itself, followed
  // by all constructor arguments.  Multiple registrations can be used to
  // register different constructors if necessary.
  static config_utilities::Factory::RegistrationRos<Base, DerivedA, std::string> registration;
};

config_utilities::Factory::RegistrationRos<Base, DerivedA, std::string> DerivedA::registration("DerivedA") ;


// Similarly, a seond derived class using completely different config and
// printing is implemented.
class DerivedB : public Base {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string s = "default text";
    float f = 0.f;

    Config() {
      setConfigName("DerivedB Config");
    }

   protected:
    void setupParamsAndPrinting() override {
      setupParam("s", &s);
      setupParam("f", &f);
    }
  };

  DerivedB(const Config& config, const std::string& info) : config_(config.checkValid()), info_(info) {}

  void print() override {
    std::cout << "This is a DerivedB with info '" << info_ << "'.\n" << config_.toString() <<std::endl;
  }

 private:
  const Config config_;
  const std::string info_;

  static config_utilities::Factory::RegistrationRos<Base, DerivedB, std::string> registration;
};

config_utilities::Factory::RegistrationRos<Base, DerivedB, std::string> DerivedB::registration("DerivedB") ;



int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ros and add some params to the parameter server.
  ros::init(argc, argv, "demo_ros_param");
  ros::NodeHandle nh_private("~");

  // The paramter 'type' is used to identify the object to be created.
  nh_private.setParam("type", "DerivedA");
  nh_private.setParam("i", 1);
  nh_private.setParam("f", 2.345);
  nh_private.setParam("s", "param text.");

  // Create the object specified in the param server using the ros factory.
  std::string info = "How to create a DerivedA";
  std::unique_ptr<Base> object = config_utilities::FactoryRos::create<Base>(nh_private, info);
  object->print();

  // If the 'type' param was different another object would be created.
  nh_private.setParam("type", "DerivedB");
  info = "Now the type param has changed";
  object = config_utilities::FactoryRos::create<Base>(nh_private, info);
  object->print();

  return 0;
}
