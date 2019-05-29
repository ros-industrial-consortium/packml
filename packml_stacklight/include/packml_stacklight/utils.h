#ifndef PACKML_STACKLIGHT_UTILS_H
#define PACKML_STACKLIGHT_UTILS_H

#include <map>
#include <ros/ros.h>
#include <packml_msgs/State.h>

namespace packml_stacklight
{
namespace LightValues
{
enum LightValue
{
  UNDEFINED = 0,
  RED = 1,
  AMBER = 2,
  GREEN = 3,
  BLUE = 4,
};

static std::map<LightValues::LightValue, std::string> light_value_map = { { UNDEFINED, "UNDEFINED-LIGHT" },
                                                                          { RED, "red" },
                                                                          { AMBER, "amber" },
                                                                          { GREEN, "green" },
                                                                          { BLUE, "blue" } };
}  // namespace LightValues
typedef LightValues::LightValue LightValue;

namespace ButtonValues
{
enum ButtonValue
{
  UNDEFINED = 0,
  START = 1,
  RESET = 2,
};

static std::map<ButtonValues::ButtonValue, std::string> button_value_map = { { UNDEFINED, "UNDEFINED-BUTTON" },
                                                                             { START, "start" },
                                                                             { RESET, "reset" } };
}  // namespace ButtonValues
typedef ButtonValues::ButtonValue ButtonValue;

namespace FlashStates
{
enum FlashState
{
  FLASH_ON = 0,
  FLASH_OFF = 1,
};
}
typedef FlashStates::FlashState FlashState;

typedef struct
{
  LightValue light_value_ = LightValues::UNDEFINED;
  bool active_ = false;
  bool flashing_ = false;
} LightAction;

typedef struct
{
  bool active_ = false;
  bool flashing_ = false;
  std::string nameMap_ = "buzzer";
} BuzzerAction;

typedef struct
{
  ButtonValue button_value_ = ButtonValues::UNDEFINED;
  LightAction light_action_;
} ButtonAction;

typedef struct
{
  int8_t state_ = packml_msgs::State::UNDEFINED;
  std::vector<LightAction> light_vec_;
  std::vector<ButtonAction> button_vec_;
  BuzzerAction buzzer_action_;
} StatusAction;

class Utils
{
private:
  std::vector<StatusAction> initDefaultStatusActions();
  void getFlash(packml_msgs::State current_state, int8_t& last_state, FlashState& last_flash, ros::Time& last_time,
                double on_secs, double off_secs);

protected:
  FlashState getLightFlash(packml_msgs::State current_state);
  FlashState getBuzzerFlash(packml_msgs::State current_state);
  StatusAction getActionFromState(packml_msgs::State current_state);
  std::map<std::string, uint8_t> getPubMap(StatusAction status_action);

public:
  Utils();
  ~Utils();
  bool getSuspendStarving();
  bool setSuspendStarving(bool starving = true);
  bool getShouldPublish(packml_msgs::State current_state);
  std::map<std::string, uint8_t> getPubMap(packml_msgs::State current_state);

protected:
  std::vector<StatusAction> status_action_vec = initDefaultStatusActions();

public:
  double flash_sec_light_on_ = 2.0;
  double flash_sec_light_off_ = 2.0;
  double flash_sec_buzzer_on_ = 2.0;
  double flash_sec_buzzer_off_ = 2.0;
  double publish_frequency_ = 0.5;
};

}  // namespace packml_stacklight

#endif  // PACKML_STACKLIGHT_UTILS_H
