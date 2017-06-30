#include <ros/console.h>
#include <QVBoxLayout>
#include "packml_gui/dashboard_panel.h"

packml_gui::DashboardPanel::DashboardPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new DashboardWidget();
  layout->addWidget(widget_);
  setLayout(layout);
  ROS_INFO("Loaded simple Packml Dashboard panel");

}

packml_gui::DashboardPanel::~DashboardPanel() {}

void packml_gui::DashboardPanel::onInitialize()
{
  ROS_INFO("Initializng Packml Dashboard panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(packml_gui::DashboardPanel, rviz::Panel)
