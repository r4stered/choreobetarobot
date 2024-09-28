// Copyright (c) Choreo contributors

#pragma once

#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <frc2/command/CommandPtr.h>

namespace choreo {

class AutoBindings {
 public:
  AutoBindings() = default;
  AutoBindings(const AutoBindings&) = delete;
  AutoBindings& operator=(const AutoBindings&) = delete;
  AutoBindings(AutoBindings&&) = default;
  AutoBindings& operator=(AutoBindings&&) = default;

  AutoBindings Bind(std::string_view name,
                    std::function<frc2::CommandPtr()> cmdFactory) && {
    bindings.emplace(name, std::move(cmdFactory));
    return std::move(*this);
  }

  void Merge(AutoBindings&& other) {
    for (auto& [key, value] : other.bindings) {
      bindings.emplace(std::move(key), std::move(value));
    }
    other.bindings.clear();
  }

  const std::unordered_map<std::string, std::function<frc2::CommandPtr()>>&
  GetBindings() const {
    return bindings;
  }

 private:
  std::unordered_map<std::string, std::function<frc2::CommandPtr()>> bindings;
};
}  // namespace choreo
