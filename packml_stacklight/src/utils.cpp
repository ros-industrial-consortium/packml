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
static std::vector<StatusActions> status_action_vec = initDefaultStatusActions();

std::vector<StatusActions> initDefaultStatusActions()
{
  static std::vector<StatusActions> temp_vec;
  for (const auto state : { packml_msgs::State::UNDEFINED, packml_msgs::State::COMPLETE })
  {
    StatusActions action;
    action.state_ = state;

    for (const auto light_value : { LightAction::LightValues::UNDEFINED, LightAction::LightValues::BLUE })
    {
      LightAction light;
      light.light_value_ = light_value;
      action.light_vec.push_back(light);
    }

    switch (action.state_)
    {
      case packml_msgs::State::ABORTING:
      case packml_msgs::State::ABORTED:
        action.light_vec[LightAction::LightValues::RED].active = true;
        action.light_vec[LightAction::LightValues::RED].flashing_ = true;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.button_action_.button_type_ = ButtonAction::ButtonTypes::RESET;
        action.button_action_.light_action_.active = true;
        action.button_action_.light_action_.light_value_ = LightAction::LightValues::BLUE;
        action.button_action_.light_action_.flashing_ = true;
        break;

      case packml_msgs::State::CLEARING:
        action.light_vec[LightAction::LightValues::RED].active = true;
        action.light_vec[LightAction::LightValues::RED].flashing_ = true;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::STOPPING:
      case packml_msgs::State::STOPPED:
        action.light_vec[LightAction::LightValues::RED].active = true;
        action.light_vec[LightAction::LightValues::RED].flashing_ = false;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::RESETTING:
        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;
        break;

      case packml_msgs::State::IDLE:
        action.light_vec[LightAction::LightValues::GREEN].active = true;
        action.light_vec[LightAction::LightValues::GREEN].flashing_ = true;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.button_action_.button_type_ = ButtonAction::ButtonTypes::START;
        action.button_action_.light_action_.active = true;
        action.button_action_.light_action_.light_value_ = LightAction::LightValues::GREEN;
        action.button_action_.light_action_.flashing_ = true;
        break;

      case packml_msgs::State::STARTING:
      case packml_msgs::State::UNHOLDING:
        action.light_vec[LightAction::LightValues::GREEN].active = true;
        action.light_vec[LightAction::LightValues::GREEN].flashing_ = false;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.button_action_.button_type_ = ButtonAction::ButtonTypes::START;
        action.button_action_.light_action_.active = true;
        action.button_action_.light_action_.light_value_ = LightAction::LightValues::GREEN;
        action.button_action_.light_action_.flashing_ = false;

        action.buzzer_action_.active = true;
        action.buzzer_action_.pulse_off_seconds = 2;
        action.buzzer_action_.pulse_on_seconds_ = 2;
        break;

      case packml_msgs::State::EXECUTE:
        action.light_vec[LightAction::LightValues::GREEN].active = true;
        action.light_vec[LightAction::LightValues::GREEN].flashing_ = false;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.button_action_.button_type_ = ButtonAction::ButtonTypes::START;
        action.button_action_.light_action_.active = true;
        action.button_action_.light_action_.light_value_ = LightAction::LightValues::GREEN;
        action.button_action_.light_action_.flashing_ = false;
        break;

      case packml_msgs::State::HOLDING:
      case packml_msgs::State::HELD:
        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = false;
        break;

      case packml_msgs::State::SUSPENDING:
      case packml_msgs::State::SUSPENDED:
        // todo starving vs blocked => flashing vs steady
        action.light_vec[LightAction::LightValues::AMBER].active = true;
        action.light_vec[LightAction::LightValues::AMBER].flashing_ = true;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.button_action_.button_type_ = ButtonAction::ButtonTypes::START;
        action.button_action_.light_action_.active = true;
        action.button_action_.light_action_.light_value_ = LightAction::LightValues::GREEN;
        action.button_action_.light_action_.flashing_ = false;
        break;

      case packml_msgs::State::UNSUSPENDING:
        action.light_vec[LightAction::LightValues::GREEN].active = true;
        action.light_vec[LightAction::LightValues::GREEN].flashing_ = false;

        action.light_vec[LightAction::LightValues::BLUE].active = true;
        action.light_vec[LightAction::LightValues::BLUE].flashing_ = true;

        action.buzzer_action_.active = true;
        action.buzzer_action_.pulse_off_seconds = 2;
        action.buzzer_action_.pulse_on_seconds_ = 2;
        break;

      default:
        break;
    }
  }

  return temp_vec;
}

}  // namespace packml_stacklight
