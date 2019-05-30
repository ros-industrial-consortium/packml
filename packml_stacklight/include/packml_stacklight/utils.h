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

#ifndef PACKML_STACKLIGHT_UTILS_H
#define PACKML_STACKLIGHT_UTILS_H

#include <map>
#include <ros/ros.h>
#include <packml_stacklight/action.h>
#include <packml_stacklight/flash.h>
#include <packml_stacklight/light.h>
#include <packml_stacklight/button.h>
#include <packml_stacklight/buzzer.h>

namespace packml_stacklight
{
class Utils
{
protected:
  std::vector<Action> action_vec_ = initDefaultStatusActions();

public:
  double flash_sec_light_on_ = 2.0;
  double flash_sec_light_off_ = 2.0;
  double flash_sec_buzzer_on_ = 2.0;
  double flash_sec_buzzer_off_ = 2.0;
  double publish_frequency_ = 0.5;

private:
  std::vector<Action> initDefaultStatusActions();
  void getFlash(packml_msgs::State current_state, int8_t& last_state, packml_stacklight::Flash::Value& last_flash,
                ros::Time& last_time, double on_secs, double off_secs);

protected:
  Flash::Value getLightFlash(packml_msgs::State current_state);
  Flash::Value getBuzzerFlash(packml_msgs::State current_state);
  Action getActionFromState(packml_msgs::State current_state);
  std::map<std::string, uint8_t> getPubMap(Action action);

public:
  Utils();
  ~Utils();
  bool getSuspendStarving();
  bool setSuspendStarving(bool starving = true);
  bool getShouldPublish(packml_msgs::State current_state);
  std::map<std::string, uint8_t> getPubMap(packml_msgs::State current_state);
};

}  // namespace packml_stacklight

#endif  // PACKML_STACKLIGHT_UTILS_H
