#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include "tools/runtime/yaml.hpp"

namespace tools
{
// A read-only view of one section of the already-loaded configuration.
class YamlConfig
{
public:
  explicit YamlConfig(const std::filesystem::path & path)
      : node_(tools::load(path.string())), path_(path)
  {
    if (!node_.IsMap())
      fail("", "expected a mapping");
  }

  bool has(const std::string & key) const
  {
    return node_ && node_[key].IsDefined();
  }

  YamlConfig section(const std::string & key) const
  {
    if (!has(key))
      return {YAML::Node(YAML::NodeType::Undefined), path_, prefix_ + key + "."};
    const auto child = node_[key];
    if (!child.IsMap())
      fail(key, "expected a mapping");
    return {child, path_, prefix_ + key + "."};
  }

  template <typename T> T get(const std::string & key) const
  {
    if (!has(key))
      fail(key, "missing required field");
    try {
      return node_[key].as<T>();
    } catch (const YAML::Exception & error) {
      fail(key, error.what());
    }
  }

  template <typename T> T get(const std::string & key, T fallback) const
  {
    return has(key) ? get<T>(key) : std::move(fallback);
  }

  // Preserve working-directory paths in existing robot configs. Also allow model files
  // alongside a config, so a self-contained deployment can be launched from elsewhere.
  std::string input_path(const std::string & key) const
  {
    const std::filesystem::path value = get<std::string>(key);
    if (value.empty())
      fail(key, "expected a file path");
    if (std::filesystem::is_regular_file(value))
      return value.string();
    if (!value.is_absolute()) {
      const auto relative =
        std::filesystem::absolute(path_.parent_path() / value).lexically_normal();
      if (std::filesystem::is_regular_file(relative))
        return relative.string();
    }
    fail(key, "file not found: " + value.string());
  }

  const YAML::Node & node() const
  {
    return node_;
  }

  [[noreturn]] void fail(const std::string & key, const std::string & message) const
  {
    const auto field = prefix_ + key;
    throw std::invalid_argument(path_.string() + (field.empty() ? "" : " [" + field + "]") + ": " +
                                message);
  }

private:
  YamlConfig(YAML::Node node, std::filesystem::path path, std::string prefix)
      : node_(std::move(node)), path_(std::move(path)), prefix_(std::move(prefix))
  {
  }

  YAML::Node node_;
  std::filesystem::path path_;
  std::string prefix_;
};
}  // namespace tools
