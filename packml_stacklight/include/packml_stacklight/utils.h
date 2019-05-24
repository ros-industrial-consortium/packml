#ifndef PACKML_STACKLIGHT_UTILS_H
#define PACKML_STACKLIGHT_UTILS_H

#include <packml_msgs/State.h>

namespace packml_stacklight
{

typedef struct
{
  typedef enum
  {
    UNDEFINED = 0,
    RED = 1,
    AMBER = 2,
    GREEN = 3,
    BLUE = 4,
  } LightValues;

  LightValues light_value_ = LightValues::UNDEFINED;
  bool active = false;
  bool flashing_ = false;
} LightAction;

typedef struct
{
  bool active = false;
  double pulse_on_seconds_ = 0;
  double pulse_off_seconds = 0;
} BuzzerAction;

typedef struct
{
  typedef enum
  {
    UNDEFINED = 0,
    START = 1,
    RESET = 1,
  } ButtonTypes;

  ButtonTypes button_type_ = ButtonTypes::UNDEFINED;
  LightAction light_action_;
} ButtonAction;

typedef struct
{
  int8_t state_ = packml_msgs::State::UNDEFINED;
  std::vector<LightAction> light_vec;
  BuzzerAction buzzer_action_;
  ButtonAction button_action_;
} StatusActions;

std::vector<StatusActions> initDefaultStatusActions();

}  // namespace packml_stacklight

#endif  // PACKML_STACKLIGHT_UTILS_H
