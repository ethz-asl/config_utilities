/**
AUTHOR: Lukas Schmid <schmluk@mavt.ethz.ch>

Copyright 2020 Autonomous Systems Lab (ASL), ETH Zürich.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Depending on which headers are available, ROS dependencies are included in
 * the library. Make sure to include config_utilities.hpp after these headers.
 */

#include <ros/node_handle.h>
#include <kindr/minimal/quat-transformation.h>

// <ros/node_handle.h>
#ifdef ROSCPP_NODE_HANDLE_H
#define CONFIG_UTILITIES_ROS_ENABLED
#endif

// <kindr/minimal/quat-transformation.h>
#ifdef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#define CONFIG_UTILITIES_TRANSFORMATION_ENABLED
#endif

#ifndef CONFIG_UTILITIES_CORE_HPP_
#define CONFIG_UTILITIES_CORE_HPP_

#include <string>
#include <utility>
#include <memory>
#include <typeinfo>
#include <type_traits>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <map>

#include <glog/logging.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace config_utilities {

/**
 * ==================== Defaults ====================
 */

namespace internal {
struct Settings {
  // Settings
  unsigned int default_print_width = 80;
  unsigned int default_print_indent = 30;
  unsigned int default_subconfig_indent = 3;

  static Settings& instance() {
    static Settings settings;
    return settings;
  }
 private:
  Settings() = default;
};
} // namespace internal

// Access.
internal::Settings& Settings() { return internal::Settings::instance(); }

/**
 * ==================== ConfigChecker Interface ====================
 */

// Utility tool to make checking configs easier and more readable.
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name);
  ~ConfigChecker() = default;

  // Get the result of all checks.
  [[nodiscard]] bool isValid(bool print_warnings=false) const;
  void checkValid() const;

  // Reset checks.
  void reset();

  // Checking interface.
  template<typename T>
  void checkGT(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkGE(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkLT(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkLE(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkEq(const T& param, const T& value, const std::string& name);
  template<typename T>
  void checkNE(const T& param, const T& value, const std::string& name);
  void checkCond(bool condition, const std::string& warning);

  // Printing interfaces.
  void setPrintWidth(int width);

 private:
  void print(const std::string& severity) const;

 private:
  const std::string name_;
  std::vector<std::string> warnings_;
  int print_width_;
};

namespace internal {
// Base class for internal use.
struct ConfigBase {
  [[nodiscard]] virtual std::string toStringInternal(int indent,
                                             int print_width,
                                             int print_indent) const =0 ;
};
// This is a dummy operator, configs provide toString().
std::ostream& operator<< (std::ostream& os, const ConfigBase&) {return os;}
}  // namespace internal

/**
 * ==================== Utilities ====================
 */
namespace internal {
std::string printCenter(const std::string& text, int width, char symbol = ' ') {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width -
      static_cast<int>(result.length()), 0), symbol);
  return result;
}

template<typename T>
bool isConfig(const T* candidate) {
  try { throw candidate; }
  catch (const ConfigBase*) {
    return true;
  }
  catch (...) {
  }
  return false;
}

} // namespace internal

/**
 * ==================== Config ====================
 */
template<class ConfigT>
struct Config : public internal::ConfigBase {
 public:
  Config() : name_(typeid(ConfigT).name()),
             meta_data_(new MetaData()) {}
  Config(const Config& other) : name_(other.name_),
                                meta_data_(new MetaData(*(other.meta_data_))) {

  }
  Config& operator=(const Config& other) {
    name_ = other.name_;
    meta_data_.reset(new MetaData(*other.meta_data_));
    return *this;
  }

  [[nodiscard]] bool isValid(bool print_warnings=false) const {
    meta_data_->checker = std::make_unique<ConfigChecker>(name_);
    meta_data_->checker->setPrintWidth(meta_data_->print_width);
    checkParams();
    bool result = meta_data_->checker->isValid(print_warnings);
    meta_data_->checker.reset(nullptr);
    return result;
  }

  virtual void initializeDependentVariableDefaults() {}

  ConfigT checkValid() const {
    // Returns a copy of the config in the const case.
    ConfigT result(*static_cast<const ConfigT*>(this));
    result.initializeDependentVariableDefaults();
    result.meta_data_->checker = std::make_unique<ConfigChecker>(name_);
    result.meta_data_->checker->setPrintWidth(meta_data_->print_width);
    result.checkParams();
    result.meta_data_->checker->checkValid();
    result.meta_data_->checker.reset(nullptr);
    return result;
  }
  ConfigT& checkValid() {
    // Returns a mutable reference.
    initializeDependentVariableDefaults();
    meta_data_->checker = std::make_unique<ConfigChecker>(name_);
    meta_data_->checker->setPrintWidth(meta_data_->print_width);
    checkParams();
    meta_data_->checker->checkValid();
    meta_data_->checker.reset(nullptr);
    return *static_cast<ConfigT*>(this);
  }

  [[nodiscard]] std::string toString() const {
    meta_data_->messages.reset(new std::vector<std::string>());
    printFields();
    std::string result = internal::printCenter(name_, meta_data_->print_width, '=');
    for (const std::string& msg : *(meta_data_->messages)) {
      result.append("\n" + msg);
    }
    result.append("\n" + std::string(meta_data_->print_width, '='));
    meta_data_->messages.reset(nullptr);
    return result;
  };

 protected:
  // Implementable methods.
  virtual void checkParams() const {
    // By default everything is valid, i.e. no checks.
  };
  virtual void printFields() const {
    meta_data_->messages->emplace_back(
        "\nThe 'printFields()' method is not implemented.");
  }
  virtual void fromRosParam() {
    LOG(WARNING) << "fromRosParam() is not implemented for '" << name_
                 << "', no parameters will be loaded.";
  }

  // General Tools.
  void setName(const std::string& name) { name_ = name; }
  void setPrintWidth(int width) { meta_data_->print_width = width; }
  void setPrintIndent(int indent) { meta_data_->print_indent = indent; }

  // Checker Tools.
  template<typename T>
  void checkParamGT(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGT(param, value, name);
  }

  template<typename T>
  void checkParamGE(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGE(param, value, name);
  }

  template<typename T>
  void checkParamLT(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLT(param, value, name);
  }

  template<typename T>
  void checkParamLE(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLE(param, value, name);
  }

  template<typename T>
  void checkParamEq(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamEq()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkEq(param, value, name);
  }

  template<typename T>
  void checkParamNE(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamNE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkNE(param, value, name);
  }

  void checkParamCond(bool condition, const std::string& warning) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamCond()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkCond(condition, warning);
  }

  // Printing Tools.
  template<typename T>
  void printField(const std::string& name, const T& field) const {
    if (!meta_data_->messages) {
      LOG(WARNING) << "'printField()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    if (internal::isConfig(&field)) {
      printConfigInternal(name, (const internal::ConfigBase*)&field);
    } else {
      printFieldInternal(name, field);
    }
  }

  void printText(const std::string& text) const {
    if (!meta_data_->messages) {
      LOG(WARNING) << "'printText()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    meta_data_->messages->emplace_back(text);
  }

 private:
  // Implementation for printing fields.
  template<typename T>
  void printFieldInternal(const std::string& name, const T& field) const {

    // The data.
    std::stringstream ss;
    ss << field;
    std::string f = ss.str();

    // The header is the field name.
    std::string header = std::string(meta_data_->indent, ' ') + name + ": ";
    while (header.length() > meta_data_->print_width) {
      // Linebreaks for too long lines.
      meta_data_->messages->emplace_back(header.substr(0,
                                                       meta_data_->print_width));
      header = header.substr(meta_data_->print_width);
    }
    if (header.length() < meta_data_->print_indent) {
      header.append(std::string(meta_data_->print_indent - header.length(), ' '));
    } else if (meta_data_->print_width - header.length() < f.length()){
      meta_data_->messages->emplace_back(header);
      header = std::string(meta_data_->print_indent, ' ');
    }

    // First line could be shorter.
    int length = meta_data_->print_width - header.length();
    if (f.length() > length) {
      meta_data_->messages->emplace_back(header + f.substr(0, length));
      f = f.substr(length);

      // Fill the rest.
      length = meta_data_->print_width - meta_data_->print_indent;
      while (f.length() > length) {
        meta_data_->messages->emplace_back(
            std::string(meta_data_->print_indent, ' ')
                + f.substr(0, length));
        f = f.substr(length);
      }
      meta_data_->messages->emplace_back(
          std::string(meta_data_->print_indent, ' ')
              + f.substr(0, length));    } else {
      meta_data_->messages->emplace_back(header.append(f));
    }
  }

  // Specialized function to print configs.
  void printConfigInternal(const std::string& name,
                          const internal::ConfigBase* field) const {
    meta_data_->messages->emplace_back(std::string(meta_data_->indent, ' ') + name + ":");
    meta_data_->messages->emplace_back(field->toStringInternal(
    meta_data_->indent + Settings().default_subconfig_indent,
    meta_data_->print_width,
    meta_data_->print_indent));
  }

  [[nodiscard]] std::string toStringInternal(int indent,
                               int print_width,
                               int print_indent) const override {
    int print_width_prev = meta_data_->print_width;
    int print_indent_prev = meta_data_->print_indent;
    int indent_prev = meta_data_->indent;
    meta_data_->print_width = print_width;
    meta_data_->print_indent = print_indent;
    meta_data_->indent = indent;

    meta_data_->messages.reset(new std::vector<std::string>());
    printFields();
    std::string result;
    for (const std::string& msg : *(meta_data_->messages)) {
      result.append("\n" + msg);
    }
    result = result.substr(1);
    meta_data_->messages.reset(nullptr);
    meta_data_->print_width = print_width_prev;
    meta_data_->print_indent = print_indent_prev;
    meta_data_->indent = indent_prev;
    return result;
  };

 private:
  // Implementation of getting any param from ros param server. Make this
  // private and explicitly implement the param types that are allowed within
  // ros node_handle getParam.
  template<typename T>
  void rosParamImpl(const std::string& name, T* param) {
//    auto param_factory =
//        impl::FactoryGetter::instance().getParamFactory<T>();
//    param_factory->requestParam(name, param);
    T test = *param;
  }

 protected:
  // Param Tools. These are explicitely overloaded to assure compatibility with
  // ROS and warn the user during coding.
  void rosParam(const std::string& name, int* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, float* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, double* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, bool* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::string* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::vector<int>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::vector<double>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::vector<float>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::vector<bool>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::vector<std::string>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::map<std::string, int>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::map<std::string, double>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::map<std::string, float>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name, std::map<std::string, bool>* param) {
    this->rosParamImpl(name, param);
  }
  void rosParam(const std::string& name,
                std::map<std::string, std::string>* param) {
    this->rosParamImpl(name, param);
  }

 private:
  struct MetaData {
    std::unique_ptr<ConfigChecker> checker;
    std::unique_ptr<std::vector<std::string>> messages;
    int print_width = Settings().default_print_width;
    int print_indent = Settings().default_print_indent;
    int indent = 0;  // Only used for nested printing.

    MetaData() = default;
    MetaData(const MetaData& other) {
      print_width = other.print_width;
      print_indent = other.print_indent;
      indent = other.indent;
    }
  };
//  template<typename DerivedConfig>
//  void rosParam(Config<DerivedConfig>* param) {
//    DerivedConfig* other = dynamic_cast<DerivedConfig*>(param);
//    if (!other) {
//      LOG(ERROR) << "Can not 'rosParam()' for <DerivedConfig>='"
//                 << typeid(ConfigT).name()
//                 << "' that does not inherit from "
//                    "'config_utilities::Config<DerivedConfig>'.";
//      return;
//    }
//    auto config_factory =
//        impl::FactoryGetter::instance().getConfigFactory<DerivedConfig>();
//    config_factory->requestConfig(other);
//    *param = *other;
//  }
//#ifdef CONFIG_UTILITIES_XMLRPCVALUE_ENABLED
//  void rosParam(const std::string& name, XmlRpc::XmlRpcValue* param) {
//    this->rosParamImpl(name, param);
//  }
//#endif  // CONFIG_UTILITIES_XMLRPCVALUE_ENABLED
//#ifdef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
//  template<typename Scalar>
//  void rosParam(const std::string& name,
//                kindr::minimal::QuatTransformationTemplate<Scalar>* param) {
//    this->rosParamImpl(name, param);
//  }
//#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED
 private:
  // friend class impl::ConfigCreator;
  void setupFromFactory() {
    fromRosParam();
  }

 private:
  std::string name_;
  std::unique_ptr<MetaData> meta_data_;
};

/**
 * ==================== ConfigChecker Implementation ====================
 */

ConfigChecker::ConfigChecker(std::string module_name) :
    name_(std::move(module_name)),
    print_width_(Settings().default_print_width) {}


bool ConfigChecker::isValid(bool print_warnings) const {
  if (warnings_.empty()) {
    return true;
  }
  if (print_warnings) {
    print("Warning");
  }
  return false;
}

void ConfigChecker::checkValid() const {
  if(!isValid(false)){
      print("Error");
    const bool config_params_are_valid = false;
    CHECK(config_params_are_valid);
  }
}

void ConfigChecker::reset() { warnings_.clear(); }

template<typename T>
void ConfigChecker::checkGT(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param <= value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected > '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkGE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param < value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected >= '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkLT(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param >= value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected < '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkLE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param > value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected <= '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkEq(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param != value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected to be '" << value << "' (is: '"
       << param << "').";
    warnings_.emplace_back(ss.str());
  }
}

template<typename T>
void ConfigChecker::checkNE(const T& param,
                            const T& value,
                            const std::string& name) {
  if (param == value) {
    std::stringstream ss;
    ss << "Param '" << name << "' is expected to be different from '" << value
       << "'.";
    warnings_.emplace_back(ss.str());
  }
}

void ConfigChecker::checkCond(bool condition, const std::string& warning) {
  if (!condition) {
    warnings_.emplace_back(warning);
  }
}

void ConfigChecker::setPrintWidth(int width) { print_width_ = width; }

void ConfigChecker::print(const std::string& severity) const {
  std::string sev = "";
  if (!severity.empty()) {
    sev.append(severity + ": ");
  }
  int length = print_width_-sev.length();
  std::string warning = "\n" + internal::printCenter(name_, print_width_, '=');
  for (std::string w : warnings_) {
    std::string line = sev;
    while (w.length() > length){
      line.append(w.substr(0,length));
      w=w.substr(length);
      warning.append("\n" + line);
      line = std::string(sev.length(), ' ');
    }
    warning.append("\n" + line + w);
  }
  warning = warning + "\n" + std::string(print_width_, '=');
  LOG(ERROR) << warning;
}

//#ifdef CONFIG_UTILITIES_ROS_ENABLED
//// ROS tools
//
//namespace impl {
//// implementation of the ConfigCreator method now that config is declared.
//template<typename ConfigT>
//void ConfigCreator::setupConfig(Config<ConfigT>* config) {
//  config->setupFromFactory();
//}
//}  // namespace impl
//
///**
// * The creation tool to be used by the user.
// */
//template<typename ConfigT>
//ConfigT getConfigFromRos(const ros::NodeHandle& nh) {
//  ConfigT config;
//  Config<ConfigT>* config_ptr = dynamic_cast<Config<ConfigT>*>(&config);
//  if (!config_ptr) {
//    LOG(ERROR) << "Can not 'getConfigFromRos()' for <ConfigType>='"
//               << typeid(ConfigT).name()
//               << "' that does not inherit from "
//                  "'config_utilities::Config<ConfigType>'.";
//    return config;
//  }
//  impl::ConfigCreator::instance().setNodeHandle(nh);
//  impl::ConfigCreator::instance().setupConfig(config_ptr);
//  return config;
//}
//
//#endif  // CONFIG_UTILITIES_ROS_ENABLED

}  // namespace config_utilities

#endif  // CONFIG_UTILITIES_CORE_HPP_
