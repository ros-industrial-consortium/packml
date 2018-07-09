/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdarg.h>
#include <stdio.h>
#include "packml_sm/dlog.h"
#include "packml_sm/boost/packml_state_machine_continuous.h"
#include "ros/console.h"

namespace packml_sm
{
void DLog::LogInfo(const char* format, ...)
{
  char buffer[0x1FF];
  va_list ap;
  va_start(ap, format);
  vsprintf(buffer, format, ap);
  ROS_INFO("%s", buffer);
  va_end(ap);
}

void DLog::LogError(const char* format, ...)
{
  char buffer[0x1FF];
  va_list ap;
  va_start(ap, format);
  vsprintf(buffer, format, ap);
  ROS_ERROR("%s", buffer);
  va_end(ap);
}

void DLog::LogWarning(const char* format, ...)
{
  char buffer[0x1FF];
  va_list ap;
  va_start(ap, format);
  vsprintf(buffer, format, ap);
  ROS_WARN("%s", buffer);
  va_end(ap);
}

void DLog::LogDebug(const char* format, ...)
{
  char buffer[0x1FF];
  va_list ap;
  va_start(ap, format);
  vsprintf(buffer, format, ap);
  ROS_DEBUG("%s", buffer);
  va_end(ap);
}
}
