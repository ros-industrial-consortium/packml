/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
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

#include "packml_ros/packml_ros.h"
#include "packml_msgs/utils.h"

namespace packml_ros
{


PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn,
          std::shared_ptr<packml_sm::StateMachine> sm) :
 nh_(nh), pn_(pn), sm_(sm)
{
  ros::NodeHandle packml_node("~/packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10, true);
  trans_server_ = packml_node.advertiseService("transition", &PackmlRos::transRequest, this);
  status_msg_ = packml_msgs::initStatus(pn.getNamespace());

  reset_stats_ = packml_node.advertiseService("reset_stats", &PackmlRos::resetStatsRequest, this);

  connect(sm.get(), SIGNAL(stateChanged(int, QString)), this, SLOT(pubState(int, QString)));
}

void PackmlRos::spin()
{
  while(ros::ok())
  {
    spinOnce();
    ros::Duration(0.001).sleep();
  }
  return;
}


void PackmlRos::spinOnce()
{
  ros::spinOnce();
  QCoreApplication::instance()->processEvents();
}


void PackmlRos::pubState(int value, QString name)
{
  ROS_DEBUG_STREAM("Publishing state change: " << name.toStdString() << "(" << value << ")");

  status_msg_.header.stamp = ros::Time().now();
  if( packml_msgs::isStandardState(value) )
  {
    status_msg_.state.val = value;
    status_msg_.sub_state = packml_msgs::State::UNDEFINED;
  }
  else
  {
    status_msg_.sub_state = value;
  }
  status_pub_.publish(status_msg_);

}

bool PackmlRos::transRequest(packml_msgs::Transition::Request &req,
                  packml_msgs::Transition::Response &res)
{
  bool command_rtn = false;
  bool command_valid = true;
  int command_int = static_cast<int>(req.command);
  std::stringstream ss;

  ROS_DEBUG_STREAM("Evaluating transition request command: " << command_int);

  switch(command_int) {
  case req.ABORT:
  case req.ESTOP:
    command_rtn = sm_->abort();
    break;
  case req.CLEAR:
    command_rtn = sm_->clear();
    break;
  case req.HOLD:
    command_rtn = sm_->hold();
    break;
  case req.RESET:
    command_rtn = sm_->reset();
    break;
  case req.START:
    command_rtn = sm_->start();
    break;
  case req.STOP:
    command_rtn = sm_->stop();
    break;
  case req.SUSPEND:
    command_rtn = sm_->suspend();
    break;
  case req.UNHOLD:
    command_rtn = sm_->unhold();
    break;
  case req.UNSUSPEND:
    command_rtn = sm_->unsuspend();
    break;

  default:
    command_valid = false;
    break;

  }
  if(command_valid)
  {
    if(command_rtn)
    {
      ss << "Successful transition request command: " << command_int;
      ROS_INFO_STREAM(ss.str());
      res.success = true;
      res.error_code = res.SUCCESS;
      res.message = ss.str();
    }
    else
    {
      ss << "Invalid transition request command: " << command_int;
      ROS_ERROR_STREAM(ss.str());
      res.success = false;
      res.error_code = res.INVALID_TRANSITION_REQUEST;
      res.message = ss.str();
    }
  }
  else
  {
    ss << "Unrecognized transition request command: " << command_int;
    ROS_ERROR_STREAM(ss.str());
    res.success = false;
    res.error_code = res.UNRECGONIZED_REQUEST;
    res.message = ss.str();
  }

}
bool PackmlRos::resetStatsRequest(packml_msgs::ResetStats::Request &req,
                                           packml_msgs::ResetStats::Response &res)
{
  ros::Duration tmp_duration;
  ros::Duration full_duration;

  // Idle time
  tmp_duration = sm_->getIdleTime();
  tmp_duration += sm_->getStartingTime();
  tmp_duration += sm_->getResettingTime();

  res.last_stat.idle_duration.data = tmp_duration;
  full_duration = tmp_duration;

  // Execute time
  tmp_duration = sm_->getExecuteTime();
  res.last_stat.exe_duration.data = tmp_duration;
  full_duration += tmp_duration;

  // Held time
  tmp_duration = sm_->getHeldTime();
  tmp_duration += sm_->getHoldingTime();
  tmp_duration += sm_->getUnholdingTime();

  res.last_stat.held_duration.data = tmp_duration;
  full_duration += tmp_duration;
  // Suspended time
  tmp_duration = sm_->getSuspendedTime();
  tmp_duration += sm_->getSuspendingTime();
  tmp_duration += sm_->getUnsuspendingTime();

  res.last_stat.susp_duration.data = tmp_duration;
  full_duration += tmp_duration;

  // Complete time
  tmp_duration = sm_->getCompleteTime();
  tmp_duration += sm_->getCompletingTime();

  res.last_stat.cmplt_duration.data = tmp_duration;
  full_duration += tmp_duration;

  // Stopped time
  tmp_duration = sm_->getStoppedTime();
  tmp_duration += sm_->getClearingTime();
  tmp_duration += sm_->getStoppingTime();

  res.last_stat.stop_duration.data = tmp_duration;
  full_duration += tmp_duration;

  // Aborted time
  tmp_duration = sm_->getAbortedTime();
  tmp_duration += sm_->getAbortingTime();

  res.last_stat.abort_duration.data = tmp_duration;
  full_duration += tmp_duration;

  res.last_stat.duration.data = full_duration;
  res.success = true;

}


} // namespace kitsune_robot
