#include <ros/console.h>
#include "packml_gui/dashboard_widget.h"

packml_gui::DashboardWidget::DashboardWidget(QWidget* parent)
    : QWidget(parent)
{
  // UI setup
  ui_ = new Ui::DashboardWidget;
  ui_->setupUi(this);

}

packml_gui::DashboardWidget::~DashboardWidget()
{

}
