#ifndef PACKML_NAVIGATION_WIDGET_H
#define PACKML_NAVIGATION_WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include "packml_msgs/Transition.h"
#include "packml_msgs/State.h"
#include "packml_msgs/Status.h"
#include "packml_msgs/Mode.h"

namespace Ui
{
class NavigationWidget;
}

namespace packml_gui
{

class NavigationWidget : public QWidget
{
  Q_OBJECT
public:
  NavigationWidget(QWidget* parent = 0);

  virtual ~NavigationWidget();

protected Q_SLOTS:
  void onStartButton();
  void onAbortButton();
  void onClearButton();
  void onHoldButton();
  void onResetButton();
  void onUnsuspendButton();
  void onUnholdButton();
  void onSuspendButton();
  void onStopButton();

protected:
  // UI
  Ui::NavigationWidget* ui_;
  void setMessage(const std::string &text);
  void updateButtonState(const packml_msgs::State &state);
  void updateStatusField(const packml_msgs::Status &msg);
  void updateStateField(const packml_msgs::State &state);
  void statusCallBack(const packml_msgs::Status &msg);
  void disableAllButtons();
  void updateModeField(const packml_msgs::Mode &mode);
  ros::ServiceClient transition_client_;
  ros::Subscriber status_subscriber_;

};
}

#endif
