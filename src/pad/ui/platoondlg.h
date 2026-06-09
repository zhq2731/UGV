#ifndef PLATOONDLG_H
#define PLATOONDLG_H

#include <QDialog>

namespace Ui {
class platoondlg;
}

class platoondlg : public QDialog
{
    Q_OBJECT

public:
    explicit platoondlg(QWidget *parent = nullptr);
    ~platoondlg();

private:
    Ui::platoondlg *ui;
};

#endif // PLATOONDLG_H
