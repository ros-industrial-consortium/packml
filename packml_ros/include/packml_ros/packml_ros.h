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


#ifndef PACKML_ROS_H
#define PACKML_ROS_H

#include <ros/ros.h>
#include <packml_msgs/Status.h>
#include <packml_msgs/Transition.h>
#include <packml_msgs/ResetStats.h>
#include <packml_sm/state_machine.h>


namespace packml_ros {

class PackmlRos: public QObject {
  Q_OBJECT

public:
    PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn,
              std::shared_ptr<packml_sm::StateMachine> sm);
    void spin();
    void spinOnce();

protected slots:

    void pubState(int value, QString name);

protected:

    bool transRequest(packml_msgs::Transition::Request &req,
                      packml_msgs::Transition::Response &res);
    bool resetStatsRequest(packml_msgs::ResetStats::Request &req,
                            packml_msgs::ResetStats::Response &res);

    ros::NodeHandle nh_;
    ros::NodeHandle pn_;
    std::shared_ptr<packml_sm::StateMachine> sm_;

    ros::Publisher status_pub_;
    ros::ServiceServer trans_server_;
    ros::ServiceServer reset_stats_;

    packml_msgs::Status status_msg_;

};
} // namespace packml_ros


#endif // PACKML_ROS_H
