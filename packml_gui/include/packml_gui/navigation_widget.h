#ifndef PACKML_NAVIGATION_WIDGET_H
#define PACKML_NAVIGATION_WIDGET_H

#include <QWidget>
#include <ros/ros.h>

namespace Ui
{
class NavigationWidget;
}

namespace packml_gui
{

class NavigationWidget : public QWidget
{
  Q_OBJECT
public:
  NavigationWidget(QWidget* parent = 0);

  virtual ~NavigationWidget();

protected:
  // UI
  Ui::NavigationWidget* ui_;

};
}

#endif
