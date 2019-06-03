/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Joshua Hatzenbuehler
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

#include "packml_stacklight/utils.h"

namespace packml_stacklight
{
Utils::Utils()
{
  return;
}

Utils::~Utils()
{
  return;
}

std::vector<Action> Utils::initDefaultStatusActions()
{
  std::vector<Action> temp_vec;
  for (int8_t state = packml_msgs::State::UNDEFINED; state <= packml_msgs::State::COMPLETE; state++)
  {
    Action action;
    action.state_ = state;

    for (int8_t light_value = Light::Value::UNDEFINED; light_value <= Light::Value::BLUE; light_value++)
    {
      Light light;
      light.current_ = (Light::Value)light_value;
      action.light_vec_.push_back(light);
    }

    for (int8_t button_value = Button::Value::UNDEFINED; button_value <= Button::Value::RESET; button_value++)
    {
      Button button;
      button.current_ = (Button::Value)button_value;
      action.button_vec_.push_back(button);
    }

    switch (action.state_)
    {
      case packml_msgs::State::ABORTING:
      case packml_msgs::State::ABORTED:
        action.light_vec_[Light::Value::RED].active_ = true;
        action.light_vec_[Light::Value::RED].flashing_ = true;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.button_vec_[Button::Value::RESET].light_.active_ = true;
        action.button_vec_[Button::Value::RESET].light_.current_ = Light::Value::BLUE;
        action.button_vec_[Button::Value::RESET].light_.flashing_ = true;
        break;

      case packml_msgs::State::CLEARING:
        action.light_vec_[Light::Value::RED].active_ = true;
        action.light_vec_[Light::Value::RED].flashing_ = true;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::STOPPING:
      case packml_msgs::State::STOPPED:
        action.light_vec_[Light::Value::RED].active_ = true;
        action.light_vec_[Light::Value::RED].flashing_ = false;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::RESETTING:
        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::IDLE:
        action.light_vec_[Light::Value::GREEN].active_ = true;
        action.light_vec_[Light::Value::GREEN].flashing_ = true;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.button_vec_[Button::Value::START].light_.active_ = true;
        action.button_vec_[Button::Value::START].light_.current_ = Light::Value::GREEN;
        action.button_vec_[Button::Value::START].light_.flashing_ = true;
        break;

      case packml_msgs::State::STARTING:
      case packml_msgs::State::UNHOLDING:
        action.light_vec_[Light::Value::GREEN].active_ = true;
        action.light_vec_[Light::Value::GREEN].flashing_ = false;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.button_vec_[Button::Value::START].light_.active_ = true;
        action.button_vec_[Button::Value::START].light_.current_ = Light::Value::GREEN;
        action.button_vec_[Button::Value::START].light_.flashing_ = false;

        action.buzzer_.active_ = true;
        action.buzzer_.flashing_ = true;
        break;

      case packml_msgs::State::EXECUTE:
        action.light_vec_[Light::Value::GREEN].active_ = true;
        action.light_vec_[Light::Value::GREEN].flashing_ = false;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.button_vec_[Button::Value::START].light_.active_ = true;
        action.button_vec_[Button::Value::START].light_.current_ = Light::Value::GREEN;
        action.button_vec_[Button::Value::START].light_.flashing_ = false;
        break;

      case packml_msgs::State::HOLDING:
      case packml_msgs::State::HELD:
        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = false;
        break;

      case packml_msgs::State::SUSPENDING:
      case packml_msgs::State::SUSPENDED:
        // todo starving vs blocked => flashing vs steady
        action.light_vec_[Light::Value::AMBER].active_ = true;
        action.light_vec_[Light::Value::AMBER].flashing_ = true;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.button_vec_[Button::Value::START].light_.active_ = true;
        action.button_vec_[Button::Value::START].light_.current_ = Light::Value::GREEN;
        action.button_vec_[Button::Value::START].light_.flashing_ = false;
        break;

      case packml_msgs::State::UNSUSPENDING:
        action.light_vec_[Light::Value::GREEN].active_ = true;
        action.light_vec_[Light::Value::GREEN].flashing_ = false;

        action.light_vec_[Light::Value::BLUE].active_ = true;
        action.light_vec_[Light::Value::BLUE].flashing_ = true;

        action.buzzer_.active_ = true;
        action.buzzer_.flashing_ = true;
        break;

      default:
        break;
    }

    temp_vec.push_back(action);
  }

  return temp_vec;
}

bool Utils::getSuspendStarving()
{
  return action_vec_[packml_msgs::State::SUSPENDING].light_vec_[Light::Value::AMBER].flashing_;
}

bool Utils::setSuspendStarving(bool starving)
{
  action_vec_[packml_msgs::State::SUSPENDING].light_vec_[Light::Value::AMBER].flashing_ = starving;
  action_vec_[packml_msgs::State::SUSPENDED].light_vec_[Light::Value::AMBER].flashing_ = starving;
  return getSuspendStarving();
}

Flash::Value Utils::getLightFlash(packml_msgs::State current_state)
{
  static ros::Time last_time(0);
  static int8_t last_state = -1;
  static Flash::Value last_flash = Flash::Value::ON;

  getFlash(current_state, last_state, last_flash, last_time, flash_sec_light_on_, flash_sec_light_off_);

  return last_flash;
}

Flash::Value Utils::getBuzzerFlash(packml_msgs::State current_state)
{
  static ros::Time last_time(0);
  static int8_t last_state = -1;
  static Flash::Value last_flash = Flash::Value::ON;

  getFlash(current_state, last_state, last_flash, last_time, flash_sec_buzzer_on_, flash_sec_buzzer_off_);

  return last_flash;
}

void Utils::getFlash(packml_msgs::State current_state, int8_t& last_state, Flash::Value& last_flash,
                     ros::Time& last_time, double on_secs, double off_secs)
{
  ros::Time new_time = ros::Time::now();
  ros::Duration dur = new_time - last_time;

  if (last_state != current_state.val)
  {
    last_time = new_time;
    last_state = current_state.val;
    last_flash = Flash::Value::ON;
    return;
  }

  if (last_flash == Flash::Value::ON && dur.toSec() >= on_secs)
  {
    last_time = new_time;
    last_flash = Flash::Value::OFF;
    return;
  }

  if (last_flash == Flash::Value::OFF && dur.toSec() >= off_secs)
  {
    last_time = new_time;
    last_flash = Flash::Value::ON;
    return;
  }
}

bool Utils::getShouldPublish(packml_msgs::State current_state)
{
  static ros::Time last_time(0);
  static int8_t last_state = -1;
  ros::Time new_time = ros::Time::now();
  ros::Duration dur = new_time - last_time;

  if (last_state != current_state.val)
  {
    last_time = new_time;
    last_state = current_state.val;
    return true;
  }

  if (dur.toSec() >= publish_frequency_)
  {
    last_time = new_time;
    return true;
  }

  return false;
}

Action Utils::getActionFromState(packml_msgs::State current_state)
{
  return action_vec_[current_state.val];
}

std::map<std::string, uint8_t> Utils::getPubMap(packml_msgs::State current_state)
{
  return getPubMap(getActionFromState(current_state));
}

std::map<std::string, uint8_t> Utils::getPubMap(Action action)
{
  std::map<std::string, uint8_t> out_map;
  packml_msgs::State current;
  current.val = action.state_;

  Flash::Value light_flash = getLightFlash(current);
  Flash::Value buzzer_flash = getBuzzerFlash(current);

  std::vector<Light>::iterator light_it;
  for (light_it = action.light_vec_.begin(); light_it != action.light_vec_.end(); ++light_it)
  {
    if (light_it->current_ == Light::Value::UNDEFINED)
    {
      continue;
    }

    uint8_t light_val = 0;
    if (light_it->active_)
    {
      light_val = 1;
    }

    if (light_it->flashing_ && light_flash == Flash::Value::OFF)
    {
      light_val = 0;
    }

    out_map.insert(std::pair<std::string, uint8_t>(light_it->map_.find(light_it->current_)->second, light_val));
  }

  std::vector<Button>::iterator button_it;
  for (button_it = action.button_vec_.begin(); button_it != action.button_vec_.end(); ++button_it)
  {
    if (button_it->current_ == Button::Value::UNDEFINED)
    {
      continue;
    }

    uint8_t button_val = 0;
    if (button_it->light_.active_)
    {
      button_val = 1;
    }

    if (button_it->light_.flashing_ && light_flash == Flash::Value::OFF)
    {
      button_val = 0;
    }

    out_map.insert(std::pair<std::string, uint8_t>(button_it->map_.find(button_it->current_)->second, button_val));
    // todo output color mapp as well
  }

  uint8_t buz_val = 0;
  if (action.buzzer_.active_)
  {
    buz_val = 1;
  }

  if (action.buzzer_.flashing_ && buzzer_flash == Flash::Value::OFF)
  {
    buz_val = 0;
  }
  out_map.insert(std::pair<std::string, uint8_t>(action.buzzer_.map_, buz_val));

  return out_map;
}

void Utils::maybeResetState(packml_msgs::State& current_state, ros::Time& last_time)
{
  ros::Time new_time = ros::Time::now();
  ros::Duration dur = new_time - last_time;

  if (status_timeout_ <= 0)
  {
    last_time = new_time;
    return;
  }

  if (last_time == ros::Time(0))
  {
    last_time = new_time;
    return;
  }

  if (current_state.val == packml_msgs::State::UNDEFINED)
  {
    last_time = new_time;
    return;
  }

  if (dur.toSec() >= status_timeout_)
  {
    ROS_WARN("%s status_timeout_ reached, setting current_state to %d", __FUNCTION__, packml_msgs::State::UNDEFINED);
    last_time = new_time;
    current_state.val = packml_msgs::State::UNDEFINED;
    return;
  }
}

}  // namespace packml_stacklight
