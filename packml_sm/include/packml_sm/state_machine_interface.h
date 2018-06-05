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
#pragma once
#include <functional>

namespace packml_sm
{
/**
 * @brief The StateMachineInterface class defines a implementation independent interface
 * to a PackML state machine.
 */
class StateMachineInterface
{
public:
  virtual bool activate() = 0;
  virtual bool setStarting(std::function<int()> state_method) = 0;
  virtual bool setExecute(std::function<int()> state_method) = 0;
  virtual bool setCompleting(std::function<int()> state_method) = 0;
  virtual bool setAborting(std::function<int()> state_method) = 0;
  virtual bool setClearing(std::function<int()> state_method) = 0;
  virtual bool setStopping(std::function<int()> state_method) = 0;
  virtual bool setResetting(std::function<int()> state_method) = 0;
  virtual bool setSuspending(std::function<int()> state_method) = 0;
  virtual bool setUnsuspending(std::function<int()> state_method) = 0;
  virtual bool setHolding(std::function<int()> state_method) = 0;
  virtual bool setUnholding(std::function<int()> state_method) = 0;
  virtual bool isActive() = 0;
  virtual int getCurrentState() = 0;

  virtual bool start();
  virtual bool clear();
  virtual bool reset();
  virtual bool hold();
  virtual bool unhold();
  virtual bool suspend();
  virtual bool unsuspend();
  virtual bool stop();
  virtual bool abort();

protected:
  virtual void _start() = 0;
  virtual void _clear() = 0;
  virtual void _reset() = 0;
  virtual void _hold() = 0;
  virtual void _unhold() = 0;
  virtual void _suspend() = 0;
  virtual void _unsuspend() = 0;
  virtual void _stop() = 0;
  virtual void _abort() = 0;
};
}
