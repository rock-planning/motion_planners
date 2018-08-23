#include "trajopt/json_marshal.hpp"
#include <json/json.h>
#include <stdexcept>
using namespace Json;
using namespace std;

namespace json_marshal {

#define IMPLEMENT_READ_PRIMITIVE(T, jsonT, cvtFunc)\
    void fromJson(const Json::Value& v, T& ref) {\
  try {\
    ref = v.cvtFunc();\
  }\
  catch (const std::runtime_error&) {\
    PRINT_AND_THROW( boost::format("expected: %s, got %s")%(#T)%(v) );\
  }}


IMPLEMENT_READ_PRIMITIVE(bool, boolValue, asBool)
IMPLEMENT_READ_PRIMITIVE(int, intValue, asInt)
IMPLEMENT_READ_PRIMITIVE(double, realValue, asDouble)
IMPLEMENT_READ_PRIMITIVE(string, stringValue, asString)



}

namespace yaml_marshal {

//#define IMPLEMENT_READ_PRIMITIVE(T, jsonT)\
//    void fromYaml(const Json::Value& v, T& ref) {\
//  try {\
//    ref = v.as<T>();\
//  }\
//  catch (const std::runtime_error&) {\
//    PRINT_AND_THROW( boost::format("expected: %s, got %s")%(#T)%(v) );\
//  }}


//IMPLEMENT_READ_PRIMITIVE(bool, boolValue)
//IMPLEMENT_READ_PRIMITIVE(int, intValue)
//IMPLEMENT_READ_PRIMITIVE(double, realValue)
//IMPLEMENT_READ_PRIMITIVE(string, stringValue)



}
