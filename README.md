# config_utilities
Utility tools to make working with config structs for (ROS) C++ libraries more uniform, readable, and convenient.

* **Author:** Lukas Schmid <schmluk@mavt.ethz.ch>.
* **Affiliation:** Autonomous Systems Lab (ASL), ETH Zürich.
* **Version:** 1.1.3
* **License:** BSD-3-Clause.

### Table of contents
* [**Why config_utilities**](#Why-config_utilities)
* [**Installation**](#Installation)
* [**Interfaces and Tools** (The config_utilities How To...)](#Interfaces-and-Tools)
* [**Demos**](#Demos)
  * [config_checker](#Config-Checker)
  * [config](#Config)
  * [ros_param](#ROS-Param)
  * [inheritance](#Inheritance)
  * [factory](#factory)
  * [ros_factory](#ROS-Factory)

  
# Why config_utilities
This library was developed to make working with config structs for object-oriented C++ libraries as simple as possible.
Using config_utilities-based configs has the following advantages:

* Having all parameters in a config struct, rather than with other variables, makes code clearer more readable:
  ```c++
  if (x_ < config_.x_max) { doMagic(); }
  ```
* Configs can be easily checked for validity with verbose warnings to avoid runtime issues:
  ```c++
  MyClass::MyClass(const Config& config) : config_(config.checkValid()) {}
  ```
* For projects consisting of a library and a ROS-package, the configs don't have any ROS dependency and can be used in the library.
  In the ROS-package, configs can be created from NodeHandles without requiring additional code:
  ```c++
  MyConfig c = config_utilities::getConfigFromRos<MyConfig>(nh_private);
  ```
* Verbose and clear printing for debugging or verification can be setup for the entire project:
  ```c++
  config_utilities::GlobalSettings().default_print_width = 80;
  std::cout << config.toString() << std::endl;
  ```
* Everything related to a config is located at its definition/implementation, making all its properties clear and easy to change.
  No need for additional code in other files where changes could be overlooked.
  ```c++
  my_class_using_configs.h / my_class_using_configs.cpp {
    // Contains *all* variables, defaults, valid values, printing, ROS-creation, factory registration, ...
  }
    ```
* Easy registration and factory creation for arbitrary classes with and without configs:
  ```c++
  static config_utilities::Factory::Registration<Base, Derived> registration("MyDerived");
  std::shared_ptr<Base> object = config_utilities::Factory::create<Base>("MyDerived");
  ```

# Installation
* **Header-Only**

  This mini-library can be used as a header only library by simply copying `config_utilities.hpp` into your project.
   Requires [glog](https://github.com/google/glog) and [xmlrpc++](http://xmlrpc.com/).
  * Dependencies:
    ```sh
    # As System Install:
    sudo apt update
    sudo apt install libxmlrpc-c++8-dev
    sudo apt-get install libgoogle-glog-dev
    ```
    ```sh
    # Alternatively, as Catkin Package:
    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/glog_catkin.git
    catkin build glog_catkin
    ```

* **Demos Package**

  To run the demos, the package can be conveniently installed via catkin: 
  ```sh
  cd ~/catkin_ws/src
  git clone https://github.com/Schmluk/config_utilities.git
  
  # If not already done so, install dependencies:
  git clone git@github.com:ethz-asl/glog_catkin.git
  git clone git@github.com:ethz-asl/gflags_catkin.git
  git clone git@github.com:ethz-asl/minkindr.git
  git clone git@github.com:ethz-asl/eigen_catkin.git
  
  cd config_utilities_demos
  catkin build --this
  ```
  
# Interfaces and Tools
Briefly describes the interfaces available and how to use them.

### Settings
Set default settings for the entire project. Set these before instantiating a config.
```c++
config_utilities::GlobalSettings().default_print_width = 80;
config_utilities::GlobalSettings().default_print_indent = 30;
```
### Configs
Define configs by inheriting from the provided `config_utilities::Config` and templating itself. 
All following interfaces are part of such a `Config`.
```c++
struct MyConfig : public config_utilities::Config<MyConfig> {
  double x_max = 1.0;
};
```
#### Public Member Functions
Use these to interact with a `Config`.
```c++
bool isValid(bool print_warnings=false) const;  // Validity information.
Config checkValid() const;  // Enforce validity.
Config& checkValid(); 
string toString() const;  // Printing.
```

#### Virtual Member Functions
Override these functions to implement the corresponding behavior.
```c++
  virtual void initializeDependentVariableDefaults();  // Initialization.
  virtual void checkParams() const;  // Param validity checks.
  virtual void printFields() const;  // Printing behavior.
  virtual void fromRosParam();  // ROS-creation behavior.
  virtual void setupParamsAndPrinting();  // Combines fromRosParam() and printFields() in a single call. Precedes but does not exclude these functions if implemented. 
```

#### Protected Member Functions
Use these tools within the virtual functions to create the desired behavior.
```c++
// General settings.
void setConfigName(const std::string& name);
void setPrintWidth(int width);
void setPrintIndent(int indent);

// Set these values in the constructor.
MyConfig::MyConfig() {
  setConfigName("MyConfig");
  ...
}
```
```c++
// Parameter validity constraints.
void checkParamGT<T>(const T& param, const T& value, const std::string& name) const;
void checkParamGE<T>(const T& param, const T& value, const std::string& name) const;
void checkParamLT<T>(const T& param, const T& value, const std::string& name) const;
void checkParamLE<T>(const T& param, const T& value, const std::string& name) const;
void checkParamEq<T>(const T& param, const T& value, const std::string& name) const;
void checkParamNE<T>(const T& param, const T& value, const std::string& name) const;
// Any condition can be checked using checkParamCond().
void checkParamCond(bool condition, const std::string &warning) const;
// Validity of member configs can be checked using checkParamConfig().
void checkParamConfig(const Config& config) const;

// Use these checks within checkParams().
MyConfig::checkParams() const {
  checkParamGT(x_max, 0.0, "x_max");
  ...
}
```
```c++
// Printing.
void printField<T>(const std::string& name, const T& field) const;
void printText(const std::string& text) const;

// Use these tools within printFields().
MyConfig::printFields() const {
  printField("x_max", x_max);
  ...
}
```
```c++
// Creation from ROS params.
void rosParam<T>(const std::string& name, T* param);
// Also works for configs, these don't require a name but an optional sub_namespace.
void rosParam(Config* config, const std::string& sub_namespace = "");
// The namespace of the creating nodehandle can be querried via rosParamNameSpace().
string rosParamNameSpace();

// Use these tools within fromRosParam(). Defaults should be set at variable declaration.
MyConfig::fromRosParam() {
  rosParam("x_max", &x_max);
  ...
}
```
```c++
// Merged param and printing setup. Internally uses the same tools as printField() and rosParam() to avoid code duplication.
void setupParam<T>(const std::string& name, T* param);

// Use these tools within setupParamsAndPrinting().
MyConfig::setupParamsAndPrinting() {
  setupParam("x_max", &x_max);
  ...
}
```

### Factory
Use these tools to let derived classes register themselves to the factory and create them based on a string or from the ROS parameter server.
```c++
// Register any class to the factory using a static struct.
static config_utilities::Factory::Registration<BaseT, DerivedT, ConstructorArgs...> registration("IdentifierString");
// Register a class that has a Config struct as a member to enable ROS creation.
static config_utilities::Factory::RegistrationRos<BaseT, DerivedT, ConstructorArgs...> registration("IdentifierString");
// Create any class registered to the factory.
std::unique_ptr<BaseT> config_utilities::Factory::create<BaseT>("IdentifierString", constructor_args);
// Create a that uses a Config from ros params. The param 'type' is expected to provide the identifier string.
// The constructors of each DerivedT is expected to take as first argument a DerivedT::Config.
std::unique_ptr<BaseT> config_utilities::FactoryRos::create<BaseT>(const ros::NodeHandle& nh, constructor_args);
```

 
# Demos
Verbose examples of the most important functionalities are given in the demos folder. They can easily be run after building the config_utilities ROS-package.

## Config Checker
This demo describes how to use the `ConfigChecker` class to verify non-config_utilities configs in a readable way:
```sh
rosrun config_utilities demo_config_checker
```
Runs a validity check and prints all warnings to console:
```
============================== IndependentConfig ===============================
Warning: Param 'a' is expected >= '0' (is: '-1').
Warning: Param 'c' is expected to be 'this is c' (is: 'test').
================================================================================
``` 

## Config
This demo describes how to define custom classes that utilize a `Config` struct:
```sh
rosrun config_utilities demo_config
```
This will setup a class using a valid config and print it to console, as well as a creation attempt with an invalid config:
```
============ MyClass-Config ============
a:             1
b:             2.34
b_half:        1.17
c:             this is c
An_extremely_unecessarily_and_unreasonab
ly_long_param_name: 
               A_similarly_unreasonably_
               long_param_value.
And a custom message.
========================================

============ MyClass-Config ============
Warning: Param 'a' is expected >= '0' (i
         s: '-1').
Warning: Param 'c' is expected to be 'th
         is is c' (is: 'test').
Warning: b is expected > a.
========================================
```

## ROS Param
This demo describes how to use the `config_utilities::getConfigFromRos<Config>()` function to setup configs via the ROS parameter server:
```sh
roscore & rosrun config_utilities demo_ros_param
```
Sets config params from ros and prints them to console:
```
================= Config (from ROS params) =================
a:             123
b:             45.6
c:             seven-eight-nine
vec:           [1, 2, 3]
map:           {m1: 1, m2: 2}
T:             t: [0, 0, 0] RPY°: [-0, 0, -0]
namespace:     /demo_ros_param
============================================================
``` 

## Inheritance
This demo describes how to use nested configs, which can be used to setup derived and base classes:
```sh
roscore & rosrun config_utilities demo_inheritance
```
Sets up a derived class from ROS, prints its nested config, and check for validity:
```
===================== MyDerivedConfig ======================
e:                  Bananas are yellow.
f:                  6
other_config:
   a:               11.1
   b:               222
base_config:
   c:               False
   d:               3.45
   other_config:
      a:            1
      b:            2
============================================================

======================= OtherConfig ========================
Warning: Param 'a' is expected >= '0' (is: '-1').
============================================================
========================== MyBase ==========================
Warning: Member config 'OtherConfig' is not valid.
============================================================
===================== MyDerivedConfig ======================
Warning: Member config 'MyBase' is not valid.
============================================================
``` 
## Factory
This demo describes how to use the `config_utilities::Factory::Registration()` and `config_utilities::Factory::create()` tools to instantiate various objects.
```sh
rosrun config_utilities demo_factory
```
Defines two derived classes and registers them statically to the factory.
```
This is a DerivedA with i=0, f=0.
This is a DerivedB with i=1, f=2.
E1104 20:45:29.080973  6629 config_utilities.hpp:1152] No module with name 'DerivedC' registered to the factory for base '4Base' and constructor arguments 'i, f'. Registered are: DerivedB, DerivedA.
'object' is invalid.
```

## ROS Factory
This demo describes how to use the `config_utilities::Factory::RegistrationRos()` and `config_utilities::FactoryRos::create()` tools to create different objects that use varying custom configs from the parameter server.
```sh
roscore & rosrun config_utilities demo_ros_factory
```

Defines two derived classes that use different configs and creates them from Ros.

```
This is a DerivedA with i=1, f=2.345, and info 'How to create a DerivedA'.
This is a DerivedB with info 'Now the type param has changed'.
=============================== DerivedB Config ================================
s:                            param text.
f:                            2.345
================================================================================
```