#pragma once
#include <json/json.h>
#include <vector>
#include <boost/format.hpp>
#include <string>
#include <sstream>
#include "utils/macros.h"
#include <type_traits>
#include "yaml-cpp/yaml.h"

namespace json_marshal {

TRAJOPT_API void fromJson(const Json::Value& v, bool& ref);
TRAJOPT_API void fromJson(const Json::Value& v, int& ref);
TRAJOPT_API void fromJson(const Json::Value& v, double& ref);
TRAJOPT_API void fromJson(const Json::Value& v, std::string& ref);
template <class T>
inline void fromJson(const Json::Value& v, T& ref) {
  ref.fromJson(v);
}
template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref) {
  ref.clear();
  ref.reserve(parent.size());
  for (Json::Value::const_iterator it = parent.begin(); it != parent.end(); ++it) {
    T t;
    fromJson(*it, t);
    ref.push_back(t);
  }
}
template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref, int size) {
  if (parent.size() != size) {
    PRINT_AND_THROW(boost::format("expected list of size size %i. got: %s\n")%size%parent);
  }
  else {
    fromJsonArray(parent, ref);
  }
}
template <class T>
inline void fromJson(const Json::Value& v, std::vector<T>& ref) {
  fromJsonArray(v, ref);
}

template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name, const T& df) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else {
    ref = df;
  }
}
template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else {
    PRINT_AND_THROW(boost::format("missing field: %s")%name);
  }
}
}

namespace yaml_marshal {

template <typename T>
void fromYamlArray(const YAML::Node& parent, std::vector<T>& ref);

template <class T>
void fromYamlArray(const YAML::Node& parent, std::vector<T>& ref, int size);

template <class T>
inline void fromYaml(const YAML::Node& v, T& ref) {
  ref.fromYaml(v);
}

template <class T>
inline void fromYaml(const YAML::Node& v, std::vector<T>& ref) {
  fromYamlArray(v, ref);
}

template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, bool& ref){ ref = v.as<bool>(); }
template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, int& ref){ ref = v.as<int>(); }
template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, float& ref){ ref = v.as<float>(); }
template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, double& ref){ ref = v.as<double>(); }
template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, char& ref){ ref = v.as<char>(); }
template <>
inline TRAJOPT_API void fromYaml(const YAML::Node& v, std::string& ref){ ref = v.as<std::string>(); }


template <typename T>
void fromYamlArray(const YAML::Node& parent, std::vector<T>& ref) {
  ref.clear();
  ref.reserve(parent.size());
  for (YAML::Node::const_iterator it = parent.begin(); it != parent.end(); ++it) {
    T t;
    fromYaml(*it, t);
    ref.push_back(t);
  }
}

template <class T>
void fromYamlArray(const YAML::Node& parent, std::vector<T>& ref, int size) {
  if (parent.size() != size) {
    PRINT_AND_THROW(boost::format("expected list of size size %i. got: %s\n")%size%parent);
  }
  else {
    fromYamlArray(parent, ref);
  }
}

template <class T>
void childFromYaml(const YAML::Node& parent, T& ref, const char* name, const T& df) {
  if (parent[name]) {
    const YAML::Node& v = parent[name];
    fromYaml(v, ref);
  }
  else {
    ref = df;
  }
}
template <class T>
void childFromYaml(const YAML::Node& parent, T& ref, const char* name) {
    if (parent[name]) {
    const YAML::Node& v = parent[name];
    fromYaml(v, ref);
  }
  else {
    PRINT_AND_THROW(boost::format("missing field: %s")%name);
  }
}
}

namespace trajopt{

template <class T>
inline void fromJsonOrYaml(const YAML::Node& v, T& ref) {
  ref.fromYaml(v);
}

template <class T>
inline void fromJsonOrYaml(const Json::Value& v, T& ref) {
  ref.fromJson(v);
}

template <class T>
inline void fromJsonOrYamlArray(const YAML::Node& v, std::vector<T>& ref) {
  yaml_marshal::fromYamlArray(v, ref);
}

template <class T>
inline void fromJsonOrYamlArray(const Json::Value& v, std::vector<T>& ref) {
  json_marshal::fromJsonArray(v, ref);
}

//template <typename T>
//inline TRAJOPT_API void fromJsonOrYaml(const YAML::Node& v, T& ref){ yaml_marshal::fromYaml(v, ref); }

//template <typename T>
//inline TRAJOPT_API void fromJsonOrYaml(const json_marshal& v, T& ref){ json_marshal::fromJson(v, ref); }


template <class T>
void childFromJsonOrYaml(const YAML::Node& parent, T& ref, const char* name, const T& df) {
  yaml_marshal::childFromYaml(parent, ref, name, df);
}
template <class T>
void childFromJsonOrYaml(const Json::Value& parent, T& ref, const char* name, const T& df) {
  json_marshal::childFromJson(parent, ref, name, df);
}

template <class T>
void childFromJsonOrYaml(const YAML::Node& parent, T& ref, const char* name) {
    yaml_marshal::childFromYaml(parent, ref, name);
}

template <class T>
void childFromJsonOrYaml(const Json::Value& parent, T& ref, const char* name) {
    yaml_marshal::childFromYaml(parent, ref, name);
}
}
