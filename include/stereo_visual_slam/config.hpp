#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <any>
#include <iostream>
#include <unordered_map>

namespace StereoSLAM {
class Config {
public:
  template <typename T> void set(const std::string &key, T value) {
    data_[key] = std::make_any<T>(value);
  }

  template <typename T> T get(const std::string &key) const {
    if (data_.find(key) != data_.end()) {
      return std::any_cast<T>(data_.at(key));
    } else {
      throw std::runtime_error("Key not found or type mismatch");
    }
  }

private:
  std::unordered_map<std::string, std::any> data_;
};
} // namespace StereoSLAM

#endif // __CONFIG_H__