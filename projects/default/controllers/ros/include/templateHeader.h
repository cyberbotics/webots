/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WEBOTS_ROS_MESSAGE_%SERVICE_NAME%_H
#define WEBOTS_ROS_MESSAGE_%SERVICE_NAME%_H

#include "ros/service_traits.h"

#include "%service_name%Request.h"
#include "%service_name%Response.h"

namespace webots_ros
{

struct %classname%
{

typedef %classname%Request Request;
typedef %classname%Response Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

};
} // namespace webots_ros

namespace ros
{
namespace service_traits
{

template<>
struct MD5Sum< ::webots_ros::%classname% > {
  static const char* value()
  {
    return "%md5sum%";
  }

  static const char* value(const ::webots_ros::%classname%&) { return value(); }
};

template<>
struct DataType< ::webots_ros::%classname% > {
  static const char* value()
  {
    return "webots_ros/%classname%";
  }

  static const char* value(const ::webots_ros::%classname%&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::%classname%Request>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::%classname% >::value();
  }
  static const char* value(const ::webots_ros::%classname%Request&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::%classname%Request>
{
  static const char* value()
  {
    return DataType< ::webots_ros::%classname% >::value();
  }
  static const char* value(const ::webots_ros::%classname%Request&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::%classname%Response>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::%classname% >::value();
  }
  static const char* value(const ::webots_ros::%classname%Response&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::%classname%Response>
{
  static const char* value()
  {
    return DataType< ::webots_ros::%classname% >::value();
  }
  static const char* value(const ::webots_ros::%classname%Response&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_%SERVICE_NAME%_H
