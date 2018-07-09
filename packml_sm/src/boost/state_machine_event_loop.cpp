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
#include "packml_sm/boost/state_machine_event_loop.h"
#include "packml_sm/make_unique.h"

#include <chrono>

namespace packml_sm
{
StateMachineEventLoop::StateMachineEventLoop(int interval) : interval_(interval)
{
}

StateMachineEventLoop::~StateMachineEventLoop()
{
  if (thread_ != nullptr)
  {
    stop_thread_ = true;
    thread_->join();
  }
}

void StateMachineEventLoop::start()
{
  if (thread_ == nullptr)
  {
    thread_ = std::make_unique<std::thread>(&StateMachineEventLoop::updateLoop, this);
  }
}

void StateMachineEventLoop::stop()
{
  if (thread_ != nullptr)
  {
    stop_thread_ = true;
    thread_->join();
    thread_ = nullptr;
  }
}

void StateMachineEventLoop::updateLoop()
{
  while (!stop_thread_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
    if (!stop_thread_)
    {
      updateTickEvent.invoke(*this, EventArgs::EMPTY);
    }
  }
}
}
