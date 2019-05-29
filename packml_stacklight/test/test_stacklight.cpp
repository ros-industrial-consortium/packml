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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "packml_stacklight/utils.h"

namespace utils_test
{
TEST(StacklightTest, LightActionDefault)
{
  packml_stacklight::LightAction light;
  EXPECT_EQ(false, light.active_);
  EXPECT_EQ(false, light.flashing_);
  EXPECT_EQ(packml_stacklight::LightValues::UNDEFINED, light.light_value_);
}

TEST(StacklightTest, BuzzerActionDefault)
{
  packml_stacklight::BuzzerAction buzzer;
  EXPECT_EQ(false, buzzer.active_);
  EXPECT_EQ(false, buzzer.flashing_);
}

TEST(StacklightTest, ButtonActionDefault)
{
  packml_stacklight::ButtonAction button;
  EXPECT_EQ(false, button.light_action_.active_);
  EXPECT_EQ(false, button.light_action_.flashing_);
  EXPECT_EQ(packml_stacklight::ButtonValues::UNDEFINED, button.button_value_);
}

TEST(StacklightTest, getStatusActionVec)
{
  std::vector<packml_stacklight::StatusAction>* status_action_vec = packml_stacklight::getStatusActionVec();
  int8_t max_state_value = packml_msgs::State::COMPLETE + 1;

  EXPECT_NE(nullptr, status_action_vec);
  EXPECT_EQ(max_state_value, status_action_vec->size());
}

TEST(StacklightTest, LightVectorInit)
{
  std::vector<packml_stacklight::StatusAction>* status_action_vec = packml_stacklight::getStatusActionVec();
  int8_t max_light_value = packml_stacklight::LightValues::BLUE + 1;

  std::vector<packml_stacklight::StatusAction>::iterator status_action_it;
  for (status_action_it = status_action_vec->begin(); status_action_it != status_action_vec->end(); ++status_action_it)
  {
    EXPECT_EQ(max_light_value, status_action_it->light_vec_.size());
  }
}

TEST(StacklightTest, ButtonVectorInit)
{
  std::vector<packml_stacklight::StatusAction>* status_action_vec = packml_stacklight::getStatusActionVec();
  int8_t max_button_value = packml_stacklight::ButtonValues::RESET + 1;

  std::vector<packml_stacklight::StatusAction>::iterator status_action_it;
  for (status_action_it = status_action_vec->begin(); status_action_it != status_action_vec->end(); ++status_action_it)
  {
    EXPECT_EQ(max_button_value, status_action_it->button_vec_.size());
  }
}

TEST(StacklightTest, DefaultStateMatch)
{
  for (int8_t state = packml_msgs::State::UNDEFINED; state <= packml_msgs::State::COMPLETE; state++)
  {
    std::vector<packml_stacklight::LightAction>::iterator light_it;
    std::vector<packml_stacklight::ButtonAction>::iterator button_it;
    packml_msgs::State temp;
    temp.val = state;

    packml_stacklight::StatusAction* status_ptr = packml_stacklight::getActionFromState(temp);
    EXPECT_NE(nullptr, status_ptr);

    switch (state)
    {
      case packml_msgs::State::ABORTING:
      case packml_msgs::State::ABORTED:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::RED ||
              light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::RESET)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(true, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValues::BLUE, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::CLEARING:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::RED ||
              light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::STOPPING:
      case packml_msgs::State::STOPPED:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else if (light_it->light_value_ == packml_stacklight::LightValue::RED)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(false, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::RESETTING:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::IDLE:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::GREEN ||
              light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(true, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValues::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::STARTING:
      case packml_msgs::State::UNHOLDING:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::GREEN)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(false, light_it->flashing_);
          }
          else if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValues::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(true, status_ptr->buzzer_action_.active_);
        EXPECT_EQ(true, status_ptr->buzzer_action_.flashing_);
        break;

      case packml_msgs::State::EXECUTE:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::GREEN)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(false, light_it->flashing_);
          }
          else if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValues::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::HOLDING:
      case packml_msgs::State::HELD:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(false, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::SUSPENDING:
      case packml_msgs::State::SUSPENDED:
        // todo starving vs blocked => flashing vs steady
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::AMBER)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);  // todo check if flashing or steady from config
          }
          else if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValues::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;

      case packml_msgs::State::UNSUSPENDING:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          if (light_it->light_value_ == packml_stacklight::LightValue::GREEN)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(false, light_it->flashing_);
          }
          else if (light_it->light_value_ == packml_stacklight::LightValue::BLUE)
          {
            EXPECT_EQ(true, light_it->active_);
            EXPECT_EQ(true, light_it->flashing_);
          }
          else
          {
            EXPECT_EQ(false, light_it->active_);
          }
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(true, status_ptr->buzzer_action_.active_);
        EXPECT_EQ(true, status_ptr->buzzer_action_.flashing_);
        break;

      default:
        for (light_it = status_ptr->light_vec_.begin(); light_it != status_ptr->light_vec_.end(); ++light_it)
        {
          EXPECT_EQ(false, light_it->active_);
        }

        for (button_it = status_ptr->button_vec_.begin(); button_it != status_ptr->button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status_ptr->buzzer_action_.active_);
        break;
    }
  }
}

TEST(StacklightTest, LightFlashOnSecs)
{
  double default_val = packml_stacklight::getFlashingLightOnDur();
  double set_val = packml_stacklight::setFlashingLightOnDur(default_val * 2);
  EXPECT_EQ(packml_stacklight::getFlashingLightOnDur(), set_val);
  EXPECT_NE(packml_stacklight::getFlashingLightOnDur(), default_val);
}

TEST(StacklightTest, LightFlashOffSecs)
{
  double default_val = packml_stacklight::getFlashingLightOffDur();
  double set_val = packml_stacklight::setFlashingLightOffDur(default_val * 2);
  EXPECT_EQ(packml_stacklight::getFlashingLightOffDur(), set_val);
  EXPECT_NE(packml_stacklight::getFlashingLightOffDur(), default_val);
}

TEST(StacklightTest, BuzzerFlashOnSecs)
{
  double default_val = packml_stacklight::getFlashingBuzzerOnDur();
  double set_val = packml_stacklight::setFlashingBuzzerOnDur(default_val * 2);
  EXPECT_EQ(packml_stacklight::getFlashingBuzzerOnDur(), set_val);
  EXPECT_NE(packml_stacklight::getFlashingBuzzerOnDur(), default_val);
}

TEST(StacklightTest, BuzzerFlashOffSecs)
{
  double default_val = packml_stacklight::getFlashingBuzzerOffDur();
  double set_val = packml_stacklight::setFlashingBuzzerOffDur(default_val * 2);
  EXPECT_EQ(packml_stacklight::getFlashingBuzzerOffDur(), set_val);
  EXPECT_NE(packml_stacklight::getFlashingBuzzerOffDur(), default_val);
}

TEST(StacklightTest, PublishFrequency)
{
  double default_val = packml_stacklight::getPublishFrequency();
  double set_val = packml_stacklight::setPublishFrequency(default_val * 2);
  EXPECT_EQ(packml_stacklight::getPublishFrequency(), set_val);
  EXPECT_NE(packml_stacklight::getPublishFrequency(), default_val);
}

TEST(StacklightTest, LightFlash)
{
  packml_stacklight::FlashState flash = packml_stacklight::FlashState::FLASH_ON;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::ABORTING;

  double on_secs = packml_stacklight::setFlashingLightOnDur(0.5);
  double off_secs = packml_stacklight::setFlashingLightOffDur(0.5);

  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(0.1).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(0.1).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(off_secs).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs * 2).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  temp.val = packml_msgs::State::ABORTED;
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);
}

TEST(StacklightTest, BuzzerFlash)
{
  packml_stacklight::FlashState flash = packml_stacklight::FlashState::FLASH_ON;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STOPPING;

  double on_secs = packml_stacklight::setFlashingBuzzerOnDur(0.5);
  double off_secs = packml_stacklight::setFlashingBuzzerOffDur(0.5);

  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(0.1).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(0.1).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(off_secs).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs * 2).sleep();
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  temp.val = packml_msgs::State::STOPPED;
  flash = packml_stacklight::getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);
}

TEST(StacklightTest, PublishAll)
{
  bool pub_all = false;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STOPPING;

  double secs = packml_stacklight::getPublishFrequency();

  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(secs).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(secs).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  temp.val = packml_msgs::State::STOPPED;
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = packml_stacklight::doPublishAll(temp);
  EXPECT_EQ(false, pub_all);
}

TEST(StacklightTest, TestPubMap)
{
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STARTING;
  int8_t max_light_value = packml_stacklight::LightValues::BLUE;
  int8_t max_button_value = packml_stacklight::ButtonValues::RESET;
  int32_t max_map_count = max_light_value + max_button_value + 1;  // plus one is for buzzer

  packml_stacklight::StatusAction* status_ptr = packml_stacklight::getActionFromState(temp);
  EXPECT_NE(nullptr, status_ptr);

  std::map<std::string, uint8_t> temp_map = packml_stacklight::getPubMap(status_ptr);
  EXPECT_EQ(max_map_count, temp_map.size());
}

TEST(StacklightTest, TestPublishTopics)
{
  packml_msgs::State temp;
  temp.val = packml_msgs::State::UNDEFINED;

  packml_stacklight::StatusAction* status_ptr = packml_stacklight::getActionFromState(temp);
  EXPECT_NE(nullptr, status_ptr);

  std::map<std::string, uint8_t> temp_map = packml_stacklight::getPubMap(status_ptr);
  std::map<std::string, uint8_t>::iterator map_itr;

  map_itr = temp_map.find("UNDEFINED");
  EXPECT_EQ(temp_map.end(), map_itr);

  map_itr = temp_map.find("UNDEFINED-LIGHT");
  EXPECT_EQ(temp_map.end(), map_itr);

  map_itr = temp_map.find("UNDEFINED-BUTTON");
  EXPECT_EQ(temp_map.end(), map_itr);

  map_itr = temp_map.find("red");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("amber");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("green");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("blue");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("start");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("reset");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  map_itr = temp_map.find("buzzer");
  EXPECT_NE(temp_map.end(), map_itr);
  temp_map.erase(map_itr);

  EXPECT_EQ(0, temp_map.size());
}
}  // namespace utils_test

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_example_test_node");
  ros::Time::init();

  int res = RUN_ALL_TESTS();

  return res;
}