#include <QString>
#include <ros/console.h>
#include "packml_gui/navigation_widget.h"
#include "ui_packml_navigation.h"

packml_gui::NavigationWidget::NavigationWidget(QWidget* parent)
    : QWidget(parent)
{
  ros::NodeHandle nh;
  sm_client_ = nh.serviceClient<packml_msgs::Transition>("/packml_node/packml/transition");

  // UI setup
  ui_ = new Ui::NavigationWidget;
  ui_->setupUi(this);

  connect(ui_->start_button, SIGNAL(clicked()), this, SLOT(onStartButton()));
  connect(ui_->abort_button, SIGNAL(clicked()), this, SLOT(onAbortButton()));
  connect(ui_->clear_button, SIGNAL(clicked()), this, SLOT(onClearButton()));
  connect(ui_->hold_button, SIGNAL(clicked()), this, SLOT(onHoldButton()));
  connect(ui_->reset_button, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->unsuspend_button, SIGNAL(clicked()), this, SLOT(onUnsuspendButton()));
  connect(ui_->unhold_button, SIGNAL(clicked()), this, SLOT(onUnholdButton()));
  connect(ui_->suspend_button, SIGNAL(clicked()), this, SLOT(onSuspendButton()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(onStopButton()));
}

packml_gui::NavigationWidget::~NavigationWidget(){}

void packml_gui::NavigationWidget::onStartButton(){
  ROS_INFO_STREAM("GUI Start Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::START;
  ROS_INFO_STREAM(sm_client_.call(trans));
}

void packml_gui::NavigationWidget::onAbortButton(){
  ROS_INFO_STREAM("GUI Abort Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::ABORT;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onClearButton(){
  ROS_INFO_STREAM("GUI Clear Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::CLEAR;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onHoldButton(){
  ROS_INFO_STREAM("GUI Hold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::HOLD;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onResetButton(){
  ROS_INFO_STREAM("GUI Reset Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::RESET;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onUnsuspendButton(){
  ROS_INFO_STREAM("GUI Unsuspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNSUSPEND;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onUnholdButton(){
  ROS_INFO_STREAM("GUI Unhold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNHOLD;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onSuspendButton(){
  ROS_INFO_STREAM("GUI Suspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::SUSPEND;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::onStopButton(){
  ROS_INFO_STREAM("GUI Stop Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::STOP;
  sm_client_.call(trans);
}

void packml_gui::NavigationWidget::setMessage(const std::string text){
  ui_->message_box->setText(QString::fromStdString(text));
}
