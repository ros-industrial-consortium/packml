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

static std::map<LightValues::LightValue, std::string> LightValueMap = { { UNDEFINED, "UNDEFINED" },
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

static std::map<ButtonValues::ButtonValue, std::string> ButtonValueMap = { { UNDEFINED, "UNDEFINED" },
                                                                           { START, "start" },
                                                                           { RESET, "reset" } };
}  // namespace ButtonValues
typedef ButtonValues::ButtonValue ButtonValue;

namespace ProcessStates
{
enum ProcessState
{
  NO_WORK = 0,
  NEW_STATE = 1,
  EXISTING_STATE = 2,
};
}
typedef ProcessStates::ProcessState ProcessState;

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

double getFlashingLightOnDur();
double setFlashingLightOnDur(double secs);

double getFlashingLightOffDur();
double setFlashingLightOffDur(double secs);

double getFlashingBuzzerOnDur();
double setFlashingBuzzerOnDur(double secs);

double getFlashingBuzzerOffDur();
double setFlashingBuzzerOffDur(double secs);

// std::vector<StatusAction> initDefaultStatusActions();
std::vector<StatusAction>* getStatusActionVec();

// todo this shoulud no longer be needed
ProcessState getNextProcess(packml_msgs::State current_state);

FlashState getLightFlash(packml_msgs::State current_state);
FlashState getBuzzerFlash(packml_msgs::State current_state);
StatusAction* getActionFromState(packml_msgs::State current_state);
std::map<std::string, uint8_t> getPubMap(StatusAction* status_action);

}  // namespace packml_stacklight

#endif  // PACKML_STACKLIGHT_UTILS_H
