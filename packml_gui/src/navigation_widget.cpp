#include <QString>
#include <ros/console.h>
#include "packml_gui/navigation_widget.h"
#include "ui_packml_navigation.h"

packml_gui::NavigationWidget::NavigationWidget(QWidget* parent)
    : QWidget(parent)
{
  ros::NodeHandle nh;
  transition_client_ = nh.serviceClient<packml_msgs::Transition>("/packml_node/packml/transition");
  status_subscriber_ = nh.subscribe("/packml_node/packml/status", 1, &packml_gui::NavigationWidget::statusCallBack, this);

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

void packml_gui::NavigationWidget::statusCallBack(const packml_msgs::Status &msg){
  updateButtonState(msg.state);
  updateStatusField(msg);
}

void packml_gui::NavigationWidget::updateStatusField(const packml_msgs::Status &msg){
  updateStateField(msg.state);
  //ui_->substate->setText(msg.sub_state));
  updateModeField(msg.mode);
  //ui_->error_code->setText(QString::fromStdString(msg.error));
  //ui_->suberror_code->setText(QString::fromStdString(msg.sub_error));
}

void packml_gui::NavigationWidget::updateStateField(const packml_msgs::State &state){
  if (state.val == packml_msgs::State::UNDEFINED){
    ui_->state_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else if (state.val == packml_msgs::State::OFF){
    ui_->state_name->setText(QString::fromStdString("OFF"));
  }
  else if (state.val == packml_msgs::State::STOPPED){
    ui_->state_name->setText(QString::fromStdString("STOPPED"));
  }
  else if (state.val == packml_msgs::State::STARTING){
    ui_->state_name->setText(QString::fromStdString("STARTING"));
  }
  else if (state.val == packml_msgs::State::IDLE){
    ui_->state_name->setText(QString::fromStdString("IDLE"));
  }
  else if (state.val == packml_msgs::State::SUSPENDED){
    ui_->state_name->setText(QString::fromStdString("SUSPENDED"));
  }
  else if (state.val == packml_msgs::State::EXECUTE){
    ui_->state_name->setText(QString::fromStdString("EXECUTE"));
  }
  else if (state.val == packml_msgs::State::STOPPING){
    ui_->state_name->setText(QString::fromStdString("STOPPING"));
  }
  else if (state.val == packml_msgs::State::ABORTING){
    ui_->state_name->setText(QString::fromStdString("ABORTING"));
  }
  else if (state.val == packml_msgs::State::ABORTED){
    ui_->state_name->setText(QString::fromStdString("ABORTED"));
  }
  else if (state.val == packml_msgs::State::HOLDING){
    ui_->state_name->setText(QString::fromStdString("HOLDING"));
  }
  else if (state.val == packml_msgs::State::HELD){
    ui_->state_name->setText(QString::fromStdString("HELD"));
  }
  else if (state.val == packml_msgs::State::RESETTING){
    ui_->state_name->setText(QString::fromStdString("RESETTING"));
  }
  else if (state.val == packml_msgs::State::SUSPENDING){
    ui_->state_name->setText(QString::fromStdString("SUSPENDING"));
  }
  else if (state.val == packml_msgs::State::UNSUSPENDING){
    ui_->state_name->setText(QString::fromStdString("UNSUSPENDING"));
  }
  else if (state.val == packml_msgs::State::CLEARING){
    ui_->state_name->setText(QString::fromStdString("CLEARING"));
  }
  else if (state.val == packml_msgs::State::UNHOLDING){
    ui_->state_name->setText(QString::fromStdString("UNHOLDING"));
  }
  else if (state.val == packml_msgs::State::COMPLETING){
    ui_->state_name->setText(QString::fromStdString("COMPLETING"));
  }
  else if (state.val == packml_msgs::State::COMPLETE){
    ui_->state_name->setText(QString::fromStdString("COMPLETE"));
  }
  else if (state.val == packml_msgs::State::UNDEFINED){
    ui_->state_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else{
    ui_->state_name->setText(QString::fromStdString("UNKNOWN"));
  }

}

void packml_gui::NavigationWidget::updateModeField(const packml_msgs::Mode &mode){
  if (mode.val == packml_msgs::Mode::UNDEFINED){
    ui_->mode_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else if (mode.val == packml_msgs::Mode::AUTOMATIC){
    ui_->mode_name->setText(QString::fromStdString("AUTOMATIC"));
  }
  else if (mode.val == packml_msgs::Mode::SEMI_AUTOMATIC){
    ui_->mode_name->setText(QString::fromStdString("SEMI-AUTOMATIC"));
  }
  else if (mode.val == packml_msgs::Mode::MANUAL){
    ui_->mode_name->setText(QString::fromStdString("MANUAL"));
  }
  else if (mode.val == packml_msgs::Mode::IDLE){
    ui_->mode_name->setText(QString::fromStdString("IDLE"));
  }
  else if (mode.val == packml_msgs::Mode::SETUP){
    ui_->mode_name->setText(QString::fromStdString("SETUP"));
  }
  else{
    ui_->mode_name->setText(QString::fromStdString("UNKNOWN"));
  }
}

void packml_gui::NavigationWidget::updateButtonState(const packml_msgs::State &state){

  disableAllButtons();

  if (state.val == packml_msgs::State::ABORTED){
    ui_->clear_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::STOPPED) {
    ui_->reset_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::IDLE) {
    ui_->start_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::EXECUTE) {
    ui_->hold_button->setEnabled(true);
    ui_->suspend_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::HELD) {
    ui_->unhold_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::SUSPENDED) {
    ui_->unsuspend_button->setEnabled(true);
  }
  else if (state.val == packml_msgs::State::COMPLETE) {
    ui_->reset_button->setEnabled(true);
  }

  if (state.val != packml_msgs::State::STOPPED &&
      state.val != packml_msgs::State::STOPPING &&
      state.val != packml_msgs::State::ABORTED &&
      state.val != packml_msgs::State::ABORTING &&
      state.val != packml_msgs::State::CLEARING){
    ui_->stop_button->setEnabled(true);
  }

  if (state.val != packml_msgs::State::ABORTED &&
      state.val != packml_msgs::State::ABORTING){
    ui_->abort_button->setEnabled(true);
  }

}

packml_gui::NavigationWidget::~NavigationWidget(){}

void packml_gui::NavigationWidget::onStartButton(){
  ROS_INFO_STREAM("GUI Start Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::START;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onAbortButton(){
  ROS_INFO_STREAM("GUI Abort Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::ABORT;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onClearButton(){
  ROS_INFO_STREAM("GUI Clear Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::CLEAR;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onHoldButton(){
  ROS_INFO_STREAM("GUI Hold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::HOLD;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onResetButton(){
  ROS_INFO_STREAM("GUI Reset Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::RESET;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onUnsuspendButton(){
  ROS_INFO_STREAM("GUI Unsuspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNSUSPEND;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onUnholdButton(){
  ROS_INFO_STREAM("GUI Unhold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNHOLD;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onSuspendButton(){
  ROS_INFO_STREAM("GUI Suspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::SUSPEND;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::onStopButton(){
  ROS_INFO_STREAM("GUI Stop Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::STOP;
  transition_client_.call(trans);
}

void packml_gui::NavigationWidget::setMessage(const std::string &text){
  ui_->message_box->setText(QString::fromStdString(text));
}

void packml_gui::NavigationWidget::disableAllButtons(){
  ui_->abort_button->setEnabled(false);
  ui_->clear_button->setEnabled(false);
  ui_->hold_button->setEnabled(false);
  ui_->reset_button->setEnabled(false);
  ui_->start_button->setEnabled(false);
  ui_->stop_button->setEnabled(false);
  ui_->suspend_button->setEnabled(false);
  ui_->unhold_button->setEnabled(false);
  ui_->unsuspend_button->setEnabled(false);
}




