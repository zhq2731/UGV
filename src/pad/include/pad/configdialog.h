#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>

#include "pad/alldeclare.h"
namespace Ui {
class ConfigDialog;
}

class ConfigDialog : public QDialog
{
Q_OBJECT

public:
explicit ConfigDialog(QWidget *parent = nullptr);
~ConfigDialog();
void init();
private:

void setUIState(bool bFlag);/*true表示界面不可编辑，false表示可编辑*/
void getUIValue();
void setUIValue();
private slots:
//    void on_btRead_clicked();

    void on_btConfirm_clicked();

    void on_btUpDate_clicked();

    void on_btModify_clicked();

    void on_btSave_clicked();

private:
Ui::ConfigDialog *ui;
};

#endif // CONFIGDIALOG_H
