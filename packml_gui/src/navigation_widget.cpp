#include <ros/console.h>
#include "packml_gui/navigation_widget.h"
#include "ui_packml_navigation.h"

packml_gui::NavigationWidget::NavigationWidget(QWidget* parent)
    : QWidget(parent)
{
  // UI setup
  ui_ = new Ui::NavigationWidget;
  ui_->setupUi(this);

}

packml_gui::NavigationWidget::~NavigationWidget()
{

}
