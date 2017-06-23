#include <QWidget>
#include <ros/ros.h>
#include "ui_packml_dashboard.h"

namespace Ui
{
class DashboardWidget;
}

namespace packml_gui
{

class DashboardWidget : public QWidget
{
  Q_OBJECT
public:
  DashboardWidget(QWidget* parent = 0);

  virtual ~DashboardWidget();

protected:
  // UI
  Ui::DashboardWidget* ui_;

};
}
