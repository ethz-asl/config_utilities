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

#ifndef CONFIG_UTILITIES_CONFIG_HPP_
#define CONFIG_UTILITIES_CONFIG_HPP_

#include <string>
#include <utility>
#include <memory>
#include <typeinfo>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <glog/logging.h>

namespace config_utilities {

/**
 * Utility tool to make checking configs easier and more readable.
 */
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name)
      : name_(std::move(module_name)), is_valid_(true) {}
  ConfigChecker(const ConfigChecker& other) : ConfigChecker(other.name_) {}
  ~ConfigChecker() = default;

  [[nodiscard]] bool isValid() const { return is_valid_; }

  template<typename T>
  void checkGT(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected > '"
                   << value << "' (is: '" << param << "').";
      is_valid_ = false;
    }
  }

  template<typename T>
  void checkGE(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected >= '"
                   << value << "' (is: '" << param << "').";
      is_valid_ = false;
    }
  }

  template<typename T>
  void checkLT(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected < '"
                   << value << "' (is: '" << param << "').";
      is_valid_ = false;
    }
  }

  template<typename T>
  void checkLE(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected <= '"
                   << value << "' (is: '" << param << "').";
      is_valid_ = false;
    }
  }

  template<typename T>
  void checkEq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected to be '"
                   << value << "' (is: '" << param << "').";
      is_valid_ = false;
    }
  }

  void checkCond(bool condition, const std::string& warning) {
    if (!condition) {
      LOG(WARNING) << name_ << ": " << warning;
      is_valid_ = false;
    }
  }

 private:
  const std::string name_;
  bool is_valid_;
};

/**
 * Base class that defines interfaces and provides tools for configs.
 */
template<class ConfigT>
struct Config {
 public:
  Config() : name_(typeid(ConfigT).name()),
             print_width_(60),
             print_indent_(30),
             config_checker_(new ConfigChecker*),
             stream_buffer_(new std::stringstream*) {
    *config_checker_ = nullptr;
    *stream_buffer_ = nullptr;
  }
  Config(const Config& other) : name_(other.name_),
                                print_width_(other.print_width_),
                                print_indent_(other.print_indent_),
                                config_checker_(new ConfigChecker*),
                                stream_buffer_(new std::stringstream*) {
    *config_checker_ = nullptr;
    *stream_buffer_ = nullptr;
  }
  virtual ~Config() = default;

  [[nodiscard]] bool isValid() const {
    ConfigChecker checker(name_);
    *config_checker_ = &checker;
    checkParams();
    bool result = (*config_checker_)->isValid();
    *config_checker_ = nullptr;
    return result;
  }

  virtual void initializeDependentVariableDefaults() {}

  ConfigT checkValid() const {
    // Returns a copy of the config in the const case.
    ConfigT result(*static_cast<const ConfigT*>(this));
    result.initializeDependentVariableDefaults();
    CHECK(result.isValid());
    return result;
  }
  ConfigT& checkValid() {
    // Returns a mutable reference.
    initializeDependentVariableDefaults();
    CHECK(isValid());
    return *static_cast<ConfigT*>(this);
  }

  [[nodiscard]] std::string toString() const {
    std::stringstream ss;
    *stream_buffer_ = &ss;
    int width = std::max((print_width_ - (int) name_.length() - 2) / 2, 0);
    auto header = std::string(width, '=') + " " + name_ + " ";
    header +=
        std::string(std::max(print_width_ - (int) header.length(), 0), '=');
    ss << header;
    printFields();
    ss << "\n" << std::string(print_width_, '=');
    std::string result = ss.str();
    *stream_buffer_ = nullptr;
    return result;
  };

 protected:
  virtual void checkParams() const = 0;
  virtual void printFields() const {
    **stream_buffer_ << "\nThe 'printFields()' method is not implemented.";
  }
  void setName(const std::string& name) { name_ = name; }
  void setPrintWidth(int width) { print_width_ = width; }
  void setPrintIndent(int indent) { print_indent_ = indent; }

  template<typename T>
  void checkParamGT(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamGT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkGT(param, value, name);
  }

  template<typename T>
  void checkParamGE(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamGE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkGE(param, value, name);
  }

  template<typename T>
  void checkParamLT(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamLT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkLT(param, value, name);
  }

  template<typename T>
  void checkParamLE(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamLE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkLE(param, value, name);
  }

  template<typename T>
  void checkParamEq(const T& param,
                    const T& value,
                    const std::string& name) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamEq()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkEq(param, value, name);
  }

  void checkParamCond(bool condition, const std::string& warning) const {
    if (!*config_checker_) {
      LOG(WARNING) << "'checkParamCond()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    (*config_checker_)->checkCond(condition, warning);
  }

  template<typename T>
  void printField(const T& field, const std::string& name) const {
    if (!*stream_buffer_) {
      LOG(WARNING) << "'printField()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    **stream_buffer_ << "\n"
                     << std::string(name + ": ").append(std::max(
                         print_indent_ - (int) name.length() - 2, 0), ' ')
                     << field;
  }

  void printText(const std::string& text) const {
    if (!*stream_buffer_) {
      LOG(WARNING) << "'printText()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    **stream_buffer_ << "\n" << text;
  }

 private:
  int print_width_;
  int print_indent_;
  std::string name_;
  std::unique_ptr<ConfigChecker*> config_checker_;
  std::unique_ptr<std::stringstream*> stream_buffer_;
};

}  // namespace config_utilities

#endif  // CONFIG_UTILITIES_CONFIG_HPP_
