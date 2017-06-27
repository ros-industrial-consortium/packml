#ifndef PACKML_NAVIGATION_WIDGET_H
#define PACKML_NAVIGATION_WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include "packml_msgs/Transition.h"
#include "packml_msgs/State.h"

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
  void setMessage(const std::string text);
  void updateButtons(packml_msgs::State state);
  ros::ServiceClient sm_client_;

};
}

#endif
