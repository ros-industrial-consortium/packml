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
class StacklightTest : public testing::Test, packml_stacklight::Utils
{
private:
  packml_stacklight::Utils utils_;

protected:
  StacklightTest()
  {
    int8_t max_state_value = packml_msgs::State::COMPLETE + 1;

    SCOPED_TRACE("StatusActionEmptyTest");
    EXPECT_NE(0, status_action_vec.size());

    SCOPED_TRACE("StatusActionCorrectSize");
    EXPECT_EQ(max_state_value, status_action_vec.size());
  }

  FRIEND_TEST(StacklightTest, LightVectorInit);
  FRIEND_TEST(StacklightTest, ButtonVectorInit);
  FRIEND_TEST(StacklightTest, DefaultStateMatch);
  FRIEND_TEST(StacklightTest, LightFlashOnSecs);
  FRIEND_TEST(StacklightTest, LightFlashOffSecs);
  FRIEND_TEST(StacklightTest, BuzzerFlashOnSecs);
  FRIEND_TEST(StacklightTest, BuzzerFlashOffSecs);
  FRIEND_TEST(StacklightTest, PublishFrequency);
  FRIEND_TEST(StacklightTest, LightFlash);
  FRIEND_TEST(StacklightTest, BuzzerFlash);
  FRIEND_TEST(StacklightTest, PublishAll);
  FRIEND_TEST(StacklightTest, TestPubMapFromAction);
  FRIEND_TEST(StacklightTest, TestPubMapFromState);
  FRIEND_TEST(StacklightTest, TestPublishTopics);
};

TEST_F(StacklightTest, LightActionDefault)
{
  packml_stacklight::LightAction light;
  EXPECT_EQ(false, light.active_);
  EXPECT_EQ(false, light.flashing_);
  EXPECT_EQ(packml_stacklight::LightValue::UNDEFINED, light.light_value_);
}

TEST_F(StacklightTest, BuzzerActionDefault)
{
  packml_stacklight::BuzzerAction buzzer;
  EXPECT_EQ(false, buzzer.active_);
  EXPECT_EQ(false, buzzer.flashing_);
}

TEST_F(StacklightTest, ButtonActionDefault)
{
  packml_stacklight::ButtonAction button;
  EXPECT_EQ(false, button.light_action_.active_);
  EXPECT_EQ(false, button.light_action_.flashing_);
  EXPECT_EQ(packml_stacklight::ButtonValue::UNDEFINED, button.button_value_);
}

TEST_F(StacklightTest, LightVectorInit)
{
  int8_t max_light_value = packml_stacklight::LightValue::BLUE + 1;

  std::vector<packml_stacklight::StatusAction>::iterator status_action_it;
  for (status_action_it = status_action_vec.begin(); status_action_it != status_action_vec.end(); ++status_action_it)
  {
    EXPECT_EQ(max_light_value, status_action_it->light_vec_.size());
  }
}

TEST_F(StacklightTest, ButtonVectorInit)
{
  int8_t max_button_value = packml_stacklight::ButtonValue::RESET + 1;

  std::vector<packml_stacklight::StatusAction>::iterator status_action_it;
  for (status_action_it = status_action_vec.begin(); status_action_it != status_action_vec.end(); ++status_action_it)
  {
    EXPECT_EQ(max_button_value, status_action_it->button_vec_.size());
  }
}

TEST_F(StacklightTest, DefaultStateMatch)
{
  for (int8_t state = packml_msgs::State::UNDEFINED; state <= packml_msgs::State::COMPLETE; state++)
  {
    std::vector<packml_stacklight::LightAction>::iterator light_it;
    std::vector<packml_stacklight::ButtonAction>::iterator button_it;
    packml_msgs::State temp;
    temp.val = state;

    packml_stacklight::StatusAction status = getActionFromState(temp);

    switch (state)
    {
      case packml_msgs::State::ABORTING:
      case packml_msgs::State::ABORTED:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::RESET)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(true, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValue::BLUE, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::CLEARING:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::STOPPING:
      case packml_msgs::State::STOPPED:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::RESETTING:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::IDLE:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(true, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValue::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::STARTING:
      case packml_msgs::State::UNHOLDING:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValue::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(true, status.buzzer_action_.active_);
        EXPECT_EQ(true, status.buzzer_action_.flashing_);
        break;

      case packml_msgs::State::EXECUTE:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValue::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::HOLDING:
      case packml_msgs::State::HELD:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::SUSPENDING:
      case packml_msgs::State::SUSPENDED:
        // todo starving vs blocked => flashing vs steady
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          if (button_it->button_value_ == packml_stacklight::ButtonValue::START)
          {
            EXPECT_EQ(true, button_it->light_action_.active_);
            EXPECT_EQ(false, button_it->light_action_.flashing_);
            EXPECT_EQ(packml_stacklight::LightValue::GREEN, button_it->light_action_.light_value_);
          }
          else
          {
            EXPECT_EQ(false, button_it->light_action_.active_);
          }
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;

      case packml_msgs::State::UNSUSPENDING:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
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

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(true, status.buzzer_action_.active_);
        EXPECT_EQ(true, status.buzzer_action_.flashing_);
        break;

      default:
        for (light_it = status.light_vec_.begin(); light_it != status.light_vec_.end(); ++light_it)
        {
          EXPECT_EQ(false, light_it->active_);
        }

        for (button_it = status.button_vec_.begin(); button_it != status.button_vec_.end(); ++button_it)
        {
          EXPECT_EQ(false, button_it->light_action_.active_);
        }

        EXPECT_EQ(false, status.buzzer_action_.active_);
        break;
    }
  }
}

TEST_F(StacklightTest, LightFlashOnSecs)
{
  double default_val = flash_sec_light_on_;
  double set_val = flash_sec_light_on_ = default_val * 2;
  EXPECT_EQ(flash_sec_light_on_, set_val);
  EXPECT_NE(flash_sec_light_on_, default_val);
}

TEST_F(StacklightTest, LightFlashOffSecs)
{
  double default_val = flash_sec_light_off_;
  double set_val = flash_sec_light_off_ = default_val * 2;
  EXPECT_EQ(flash_sec_light_off_, set_val);
  EXPECT_NE(flash_sec_light_off_, default_val);
}

TEST_F(StacklightTest, BuzzerFlashOnSecs)
{
  double default_val = flash_sec_buzzer_on_;
  double set_val = flash_sec_buzzer_on_ = default_val * 2;
  EXPECT_EQ(flash_sec_buzzer_on_, set_val);
  EXPECT_NE(flash_sec_buzzer_on_, default_val);
}

TEST_F(StacklightTest, BuzzerFlashOffSecs)
{
  double default_val = flash_sec_buzzer_off_;
  double set_val = flash_sec_buzzer_off_ = default_val * 2;
  EXPECT_EQ(flash_sec_buzzer_off_, set_val);
  EXPECT_NE(flash_sec_buzzer_off_, default_val);
}

TEST_F(StacklightTest, PublishFrequency)
{
  double default_val = publish_frequency_;
  double set_val = publish_frequency_ = default_val * 2;
  EXPECT_EQ(publish_frequency_, set_val);
  EXPECT_NE(publish_frequency_, default_val);
}

TEST_F(StacklightTest, LightFlash)
{
  packml_stacklight::FlashState flash = packml_stacklight::FlashState::FLASH_ON;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::ABORTING;

  flash_sec_light_on_ = 0.5;
  flash_sec_light_off_ = 0.5;

  double on_secs = flash_sec_light_on_;
  double off_secs = flash_sec_light_off_;

  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(0.1).sleep();
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs).sleep();
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(0.1).sleep();
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(off_secs).sleep();
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs * 2).sleep();
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  temp.val = packml_msgs::State::ABORTED;
  flash = getLightFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);
}

TEST_F(StacklightTest, BuzzerFlash)
{
  packml_stacklight::FlashState flash = packml_stacklight::FlashState::FLASH_ON;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STOPPING;

  flash_sec_buzzer_on_ = 0.5;
  flash_sec_buzzer_off_ = 0.5;

  double on_secs = flash_sec_buzzer_on_;
  double off_secs = flash_sec_buzzer_off_;

  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(0.1).sleep();
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs).sleep();
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(0.1).sleep();
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  ros::Duration(off_secs).sleep();
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);

  ros::Duration(on_secs * 2).sleep();
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_OFF, flash);

  temp.val = packml_msgs::State::STOPPED;
  flash = getBuzzerFlash(temp);
  EXPECT_EQ(packml_stacklight::FlashState::FLASH_ON, flash);
}

TEST_F(StacklightTest, PublishAll)
{
  bool pub_all = false;
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STOPPING;

  double secs = publish_frequency_;

  pub_all = getShouldPublish(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(secs).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(false, pub_all);

  ros::Duration(secs).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  temp.val = packml_msgs::State::STOPPED;
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(true, pub_all);

  ros::Duration(0.1).sleep();
  pub_all = getShouldPublish(temp);
  EXPECT_EQ(false, pub_all);
}

TEST_F(StacklightTest, TestPubMapFromAction)
{
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STARTING;
  int8_t max_light_value = packml_stacklight::LightValue::BLUE;
  int8_t max_button_value = packml_stacklight::ButtonValue::RESET;
  int32_t max_map_count = max_light_value + max_button_value + 1;  // plus one is for buzzer

  packml_stacklight::StatusAction status = getActionFromState(temp);
  std::map<std::string, uint8_t> temp_map = getPubMap(status);
  EXPECT_EQ(max_map_count, temp_map.size());
}

TEST_F(StacklightTest, TestPubMapFromState)
{
  packml_msgs::State temp;
  temp.val = packml_msgs::State::STARTING;
  int8_t max_light_value = packml_stacklight::LightValue::BLUE;
  int8_t max_button_value = packml_stacklight::ButtonValue::RESET;
  int32_t max_map_count = max_light_value + max_button_value + 1;  // plus one is for buzzer

  std::map<std::string, uint8_t> temp_map = getPubMap(temp);
  EXPECT_EQ(max_map_count, temp_map.size());
}

TEST_F(StacklightTest, TestPublishTopics)
{
  packml_msgs::State temp;
  temp.val = packml_msgs::State::UNDEFINED;

  std::map<std::string, uint8_t> temp_map = getPubMap(temp);
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