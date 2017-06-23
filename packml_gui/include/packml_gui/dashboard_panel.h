#include "rviz/panel.h"
#include "packml_gui/dashboard_widget.h"

namespace packml_gui
{

// Forward declare blend widget
class DashboardWidget;

class DashboardPanel : public rviz::Panel
{
  Q_OBJECT
public:
  DashboardPanel(QWidget* parent = 0);

  virtual ~DashboardPanel();

  virtual void onInitialize();

protected:
  DashboardWidget* widget_;
};
}
