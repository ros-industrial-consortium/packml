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
#pragma once

#include <boost/mpl/map/map50.hpp>

namespace packml_sm
{
struct clear_event
{
};

struct state_complete_event
{
};

struct abort_event
{
};

struct reset_event
{
};

struct stop_event
{
};

struct start_event
{
};

struct hold_event
{
};

struct suspend_event
{
};

struct unhold_event
{
};

struct unsuspend_event
{
};

struct error_event
{
};
}
