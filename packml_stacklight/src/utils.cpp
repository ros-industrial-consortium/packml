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
static std::vector<StatusAction> initDefaultStatusActions();
static void getFlash(packml_msgs::State current_state, int8_t& last_state, FlashState& last_flash, ros::Time& last_time,
                     double on_secs, double off_secs);

static std::vector<StatusAction> status_action_vec = initDefaultStatusActions();
static double light_on_secs = 2.0;
static double light_off_secs = 2.0;

static double buzzer_on_secs = 2.0;
static double buzzer_off_secs = 2.0;

double getFlashingLightOnDur ()
{
  return light_on_secs;
}

double setFlashingLightOnDur(double secs)
{
  light_on_secs = secs;
  return light_on_secs;
}

double getFlashingLightOffDur()
{
  return light_off_secs;
}

double setFlashingLightOffDur(double secs)
{
  light_off_secs = secs;
  return light_off_secs;
}

double getFlashingBuzzerOnDur()
{
  return buzzer_on_secs;
}

double setFlashingBuzzerOnDur(double secs)
{
  buzzer_on_secs = secs;
  return buzzer_on_secs;
}

double getFlashingBuzzerOffDur()
{
  return buzzer_off_secs;
}

double setFlashingBuzzerOffDur(double secs)
{
  buzzer_off_secs = secs;
  return buzzer_off_secs;
}

std::vector<StatusAction>* getStatusActionVec()
{
  return &status_action_vec;
}

std::vector<StatusAction> initDefaultStatusActions()
{
  std::vector<StatusAction> temp_vec;
  for (int8_t state = packml_msgs::State::UNDEFINED; state <= packml_msgs::State::COMPLETE; state++)
  {
    StatusAction action;
    action.state_ = state;

    for (int8_t light_value = LightValues::UNDEFINED; light_value <= LightValues::BLUE; light_value++)
    {
      LightAction light;
      light.light_value_ = (LightValue)light_value;
      action.light_vec_.push_back(light);
    }

    for (int8_t button_value = ButtonValues::UNDEFINED; button_value <= ButtonValues::RESET; button_value++)
    {
      ButtonAction button;
      button.button_value_ = (ButtonValue)button_value;
      action.button_vec_.push_back(button);
    }

    switch (action.state_)
    {
      case packml_msgs::State::ABORTING:
      case packml_msgs::State::ABORTED:
        action.light_vec_[LightValues::RED].active_ = true;
        action.light_vec_[LightValues::RED].flashing_ = true;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.button_vec_[ButtonValues::RESET].light_action_.active_ = true;
        action.button_vec_[ButtonValues::RESET].light_action_.light_value_ = LightValues::BLUE;
        action.button_vec_[ButtonValues::RESET].light_action_.flashing_ = true;
        break;

      case packml_msgs::State::CLEARING:
        action.light_vec_[LightValues::RED].active_ = true;
        action.light_vec_[LightValues::RED].flashing_ = true;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::STOPPING:
      case packml_msgs::State::STOPPED:
        action.light_vec_[LightValues::RED].active_ = true;
        action.light_vec_[LightValues::RED].flashing_ = false;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::RESETTING:
        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::IDLE:
        action.light_vec_[LightValues::GREEN].active_ = true;
        action.light_vec_[LightValues::GREEN].flashing_ = true;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.button_vec_[ButtonValues::START].light_action_.active_ = true;
        action.button_vec_[ButtonValues::START].light_action_.light_value_ = LightValues::GREEN;
        action.button_vec_[ButtonValues::START].light_action_.flashing_ = true;
        break;

      case packml_msgs::State::STARTING:
      case packml_msgs::State::UNHOLDING:
        action.light_vec_[LightValues::GREEN].active_ = true;
        action.light_vec_[LightValues::GREEN].flashing_ = false;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.button_vec_[ButtonValues::START].light_action_.active_ = true;
        action.button_vec_[ButtonValues::START].light_action_.light_value_ = LightValues::GREEN;
        action.button_vec_[ButtonValues::START].light_action_.flashing_ = false;

        action.buzzer_action_.active_ = true;
        action.buzzer_action_.flashing_ = true;
        break;

      case packml_msgs::State::EXECUTE:
        action.light_vec_[LightValues::GREEN].active_ = true;
        action.light_vec_[LightValues::GREEN].flashing_ = false;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.button_vec_[ButtonValues::START].light_action_.active_ = true;
        action.button_vec_[ButtonValues::START].light_action_.light_value_ = LightValues::GREEN;
        action.button_vec_[ButtonValues::START].light_action_.flashing_ = false;
        break;

      case packml_msgs::State::HOLDING:
      case packml_msgs::State::HELD:
        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = false;
        break;

      case packml_msgs::State::SUSPENDING:
      case packml_msgs::State::SUSPENDED:
        // todo starving vs blocked => flashing vs steady
        action.light_vec_[LightValues::AMBER].active_ = true;
        action.light_vec_[LightValues::AMBER].flashing_ = true;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.button_vec_[ButtonValues::START].light_action_.active_ = true;
        action.button_vec_[ButtonValues::START].light_action_.light_value_ = LightValues::GREEN;
        action.button_vec_[ButtonValues::START].light_action_.flashing_ = false;
        break;

      case packml_msgs::State::UNSUSPENDING:
        action.light_vec_[LightValues::GREEN].active_ = true;
        action.light_vec_[LightValues::GREEN].flashing_ = false;

        action.light_vec_[LightValues::BLUE].active_ = true;
        action.light_vec_[LightValues::BLUE].flashing_ = true;

        action.buzzer_action_.active_ = true;
        action.buzzer_action_.flashing_ = true;
        break;

      default:
        break;
    }

    temp_vec.push_back(action);
  }

  return temp_vec;
}

FlashState getLightFlash(packml_msgs::State current_state)
{
  static ros::Time last_time(0);
  static int8_t last_state = packml_msgs::State::UNDEFINED;
  static FlashState last_flash = FlashState::FLASH_ON;

  getFlash(current_state, last_state, last_flash, last_time, light_on_secs, light_off_secs);

  return last_flash;
}

FlashState getBuzzerFlash(packml_msgs::State current_state)
{
  static ros::Time last_time(0);
  static int8_t last_state = packml_msgs::State::UNDEFINED;
  static FlashState last_flash = FlashState::FLASH_ON;

  getFlash(current_state, last_state, last_flash, last_time, buzzer_on_secs, buzzer_off_secs);

  return last_flash;
}

static void getFlash(packml_msgs::State current_state, int8_t& last_state, FlashState& last_flash, ros::Time& last_time,
                     double on_secs, double off_secs)
{
  ros::Time new_time = ros::Time::now();
  ros::Duration dur = new_time - last_time;

  if (last_state == packml_msgs::State::UNDEFINED || last_state != current_state.val)
  {
    last_time = new_time;
    last_state = current_state.val;
    last_flash = FlashState::FLASH_ON;
    return;
  }

  if (last_flash == FlashState::FLASH_ON && dur.toSec() >= on_secs)
  {
    last_time = new_time;
    last_flash = FlashState::FLASH_OFF;
    return;
  }

  if (last_flash == FlashState::FLASH_OFF && dur.toSec() >= off_secs)
  {
    last_time = new_time;
    last_flash = FlashState::FLASH_ON;
    return;
  }
}

StatusAction* getActionFromState(packml_msgs::State current_state)
{
  return &status_action_vec[current_state.val];
}

std::map<std::string, uint8_t> getPubMap(StatusAction* status_action)
{
  std::map<std::string, uint8_t> out_map;
  packml_msgs::State current;
  current.val = status_action->state_;

  FlashState light_flash = getLightFlash(current);
  FlashState buzzer_flash = getBuzzerFlash(current);

  if (status_action == nullptr)
  {
    return out_map;
  }

  std::vector<LightAction>::iterator light_it;
  for (light_it = status_action->light_vec_.begin(); light_it != status_action->light_vec_.end(); ++light_it)
  {
    if (light_it->light_value_ == LightValue::UNDEFINED)
    {
      continue;
    }

    uint8_t light_val = 0;
    if (light_it->active_)
    {
      light_val = 1;
    }

    if (light_it->flashing_ && light_flash == FlashState::FLASH_OFF)
    {
      light_val = 0;
    }

    out_map.insert(std::pair<std::string, uint8_t>(LightValues::LightValueMap[light_it->light_value_], light_val));
  }

  std::vector<ButtonAction>::iterator button_it;
  for (button_it = status_action->button_vec_.begin(); button_it != status_action->button_vec_.end(); ++button_it)
  {
    if (button_it->button_value_ == ButtonValue::UNDEFINED)
    {
      continue;
    }

    uint8_t button_val = 0;
    if (button_it->light_action_.active_)
    {
      button_val = 1;
    }

    if (button_it->light_action_.flashing_ && light_flash == FlashState::FLASH_OFF)
    {
      button_val = 0;
    }

    out_map.insert(std::pair<std::string, uint8_t>(ButtonValues::ButtonValueMap[button_it->button_value_], button_val));
    // todo output color mapp as well
  }

  uint8_t buz_val = 0;
  if (status_action->buzzer_action_.active_)
  {
    buz_val = 1;
  }

  if (status_action->buzzer_action_.flashing_ && buzzer_flash == FlashState::FLASH_OFF)
  {
    buz_val = 0;
  }
  out_map.insert(std::pair<std::string, uint8_t>(status_action->buzzer_action_.nameMap_, buz_val));

  return out_map;
}

}  // namespace packml_stacklight
