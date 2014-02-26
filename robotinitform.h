#ifndef ROBOTINITFORM_H
#define ROBOTINITFORM_H

#include <QWidget>

namespace Ui {
class RobotInitForm;
}

class RobotInitForm : public QWidget
{
    Q_OBJECT

public:
    explicit RobotInitForm(QWidget *parent = 0);
    ~RobotInitForm();

private:
    Ui::RobotInitForm *ui;
};

#endif // ROBOTINITFORM_H
