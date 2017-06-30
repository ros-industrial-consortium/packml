#include "rviz/panel.h"

namespace packml_gui
{

// Forward declare blend widget
class NavigationWidget;

class NavigationPanel : public rviz::Panel
{
  Q_OBJECT
public:
  NavigationPanel(QWidget* parent = 0);

  virtual ~NavigationPanel();

  virtual void onInitialize();

protected:
  NavigationWidget* widget_;
};
}
