#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "tools/logger.hpp"

namespace tools
{
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    throw std::runtime_error("Failed to load YAML file: " + path);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    throw std::runtime_error("Failed to parse YAML file: " + path);
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  throw std::runtime_error("Missing YAML key: " + key);
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP
