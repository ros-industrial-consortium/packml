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

#ifndef EVENT_HPP
#define EVENT_HPP

#include <functional>
#include <vector>

struct EventArgs
{
  static const EventArgs EMPTY;
};

template <class TOwningType, class TEventArgs>
class EventHandler
{
public:
  typedef std::function<void(TOwningType&, const TEventArgs&)> EventFunction;

  EventHandler() = default;

  friend TOwningType;

  template <typename TSubscribingType>
  void bind_member_func(TSubscribingType* caller,
                        void (TSubscribingType::*member_func)(TOwningType&, const TEventArgs&))
  {
    callbacks_.push_back(std::bind(member_func, caller, std::placeholders::_1, std::placeholders::_2));
  }

  template <typename TSubscribingType>
  void unbind_member_func(TSubscribingType* caller,
                          void (TSubscribingType::*member_func)(TOwningType&, const TEventArgs&))
  {
    std::function<void(TOwningType&, const TEventArgs&)> callback =
        std::bind(member_func, caller, std::placeholders::_1, std::placeholders::_2);
    for (auto iter = callbacks_.begin(); iter != callbacks_.end(); iter++)
    {
      if (callback.target_type() == iter->target_type() &&
          iter->template target<std::function<void(TOwningType&, const TEventArgs&)>>() ==
              callback.template target<std::function<void(TOwningType&, const TEventArgs&)>>())
      {
        callbacks_.erase(iter);
        callbacks_.shrink_to_fit();
        break;
      }
    }
  }

private:
  std::vector<std::function<void(TOwningType&, const TEventArgs&)>> callbacks_;

  void invoke(TOwningType& sender, TEventArgs a)
  {
    const std::vector<EventFunction> callbacks(callbacks_);
    for (auto it = callbacks.begin(); it != callbacks.end(); ++it)
    {
      (*it)(sender, a);
    }
  }
};

#endif /* EVENT_HPP */
