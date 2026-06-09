#include "platoondlg.h"
#include "ui_platoondlg.h"

platoondlg::platoondlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::platoondlg)
{
    ui->setupUi(this);
}

platoondlg::~platoondlg()
{
    delete ui;
}
