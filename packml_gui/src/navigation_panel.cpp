#include <ros/console.h>
#include <QVBoxLayout>
#include "packml_gui/navigation_panel.h"
#include "packml_gui/navigation_widget.h"


packml_gui::NavigationPanel::NavigationPanel(QWidget* parent) : rviz::Panel(parent)
{
  ROS_INFO("Loaded simple blending panel");
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new NavigationWidget();
  layout->addWidget(widget_);
  setLayout(layout);
}

packml_gui::NavigationPanel::~NavigationPanel() {}

void packml_gui::NavigationPanel::onInitialize()
{
  ROS_INFO("Initializng simple blending panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(packml_gui::NavigationPanel, rviz::Panel)
