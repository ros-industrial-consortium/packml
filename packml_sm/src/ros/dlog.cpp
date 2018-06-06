/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
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
#include "ros/console.h"

namespace packml_sm
{
void DLog::LogInfo(const char* format, ...)
{
  va_list ap;
  va_start(ap, format);
  ROS_INFO(format, ap);
  va_end(ap);
}

void DLog::LogError(char* format, ...)
{
  va_list ap;
  va_start(ap, format);
  ROS_ERROR(format, ap);
  va_end(ap);
}

void DLog::LogWarning(char* format, ...)
{
  va_list ap;
  va_start(ap, format);
  ROS_WARN(format, ap);
  va_end(ap);
}

void DLog::LogDebug(char* format, ...)
{
  va_list ap;
  va_start(ap, format);
  ROS_DEBUG(format, ap);
  va_end(ap);
}
}
