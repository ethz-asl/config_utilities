/**
 * This demo showcases the use of config_utilities factory to create varying
 * config-independent types.
 */

#include <iostream>
#include <string>

// NOTE: This is required before config_utilities, s.t. the ROS-tools,
// i.e. getConfigFromRos(), are enabled.
#include <ros/node_handle.h>

#include "../config_utilities.hpp"

// Define a common base class.
class Base {
 public:
  Base() = default;
  Base(int i, float f) : i_(i), f_(f) {}

  virtual void print() = 0;

 protected:
  int i_ = 0;
  float f_ = 0.f;
};

// Define a derived classes.
class DerivedA : public Base {
 public:
  DerivedA(int i, float f) : Base(i, f) {}
  DerivedA() = default;

  void print() override {
    std::cout << "This is a DerivedA with i=" << i_ << ", f=" << f_ << "."
              << std::endl;
  }

 private:
  // Each derived class can register itself using a static registration struct.
  // The registration is templated on the Base, itself, followed by all
  // constructor arguments. Multiple registrations can be used to register
  // different constructors if necessary.
  static config_utilities::Factory::Registration<Base, DerivedA, int, float>
      registration;
  static config_utilities::Factory::Registration<Base, DerivedA>
      registration_default;
};

// The argument to the registration constructor is the name by which classes are
// retrieved. These are required to be unique within a set of template arguments
// but may repeat for different template arguments.
config_utilities::Factory::Registration<Base, DerivedA, int, float>
    DerivedA::registration("DerivedA");
config_utilities::Factory::Registration<Base, DerivedA>
    DerivedA::registration_default("DerivedA");

// Create a second derived class. This one without a default registration.
class DerivedB : public Base {
 public:
  DerivedB(int i, float f) : Base(i, f) {}
  DerivedB() = default;

  void print() override {
    std::cout << "This is a DerivedB with i=" << i_ << ", f=" << f_ << "."
              << std::endl;
  }

 private:
  static config_utilities::Factory::Registration<Base, DerivedB, int, float>
      registration;
  static config_utilities::Factory::Registration<Base, DerivedB>
      registration_default;
};

config_utilities::Factory::Registration<Base, DerivedB, int, float>
    DerivedB::registration("DerivedB");
config_utilities::Factory::Registration<Base, DerivedB>
    DerivedB::registration_default("DerivedB");

int main(int argc, char** argv) {
  // Setup Logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Objects can be created using the create<Base> function. The arguments are
  // required to match the templates of the registration or lookup will fail.
  std::unique_ptr<Base> object =
      config_utilities::Factory::create<Base>("DerivedA");
  object->print();

  // Create a DerivedB using the specific constructor.
  object = config_utilities::Factory::create<Base>("DerivedB", 1, 2.f);
  object->print();

  // If the lookup fails a nullptr is returned and a warning is raised.
  object = config_utilities::Factory::create<Base>("DerivedC", 1, 2.f);
  if (object) {
    object->print();
  } else {
    std::cout << "'object' is invalid." << std::endl;
  }

  return 0;
}
