#include <ros/console.h>
#include <QVBoxLayout>
#include "packml_gui/navigation_panel.h"
#include "packml_gui/navigation_widget.h"


packml_gui::NavigationPanel::NavigationPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new NavigationWidget();
  layout->addWidget(widget_);
  setLayout(layout);
  ROS_INFO("Loaded Packml Navigation panel");
}

packml_gui::NavigationPanel::~NavigationPanel() {}

void packml_gui::NavigationPanel::onInitialize()
{
  ROS_INFO("Initializng Packml Navigation panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(packml_gui::NavigationPanel, rviz::Panel)
