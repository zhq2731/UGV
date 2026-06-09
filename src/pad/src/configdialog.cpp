#include "pad/configdialog.h"
#include "ui_configdialog.h"

#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>

ConfigDialog::ConfigDialog(QWidget *parent) :
QDialog(parent),
ui(new Ui::ConfigDialog)
{
    ui->setupUi(this);

    QPalette p = this->palette();
    p.setColor(QPalette::Window, QColor(50,50,50));
    this->setPalette(p);/*设置窗口背景颜色*/

}

ConfigDialog::~ConfigDialog()
{
    delete ui;
    qDebug() << "~ConfigDialog()";
}

void ConfigDialog::init()
{
    setUIValue();
    setUIState(true);
}
void ConfigDialog::getUIValue()/*修改配置文件的值*/
{
    USE_VEH_SIMU = ui->leSimulation->text().toUInt();
    VEHICLE_MASS =  ui->leVehMass->text().toDouble();
    VEHICLE_WHEEL_BASE = ui->leVehWheelBase->text().toDouble();
    VEHICLE_LENGTH = ui->leVehLength->text().toDouble();
    VEHICLE_WIDTH = ui->leVehWidth->text().toDouble();
    VEHICLE_HEIGHT = ui->leVehHeight->text().toDouble();
    CHAS_MOVE_CTRL_PERIOD = ui->leControlPeriod->text().toDouble();
    MAX_LINE_SPEED = ui->leMaxLineSpeed->text().toDouble();
    MAX_TURN_ANGLE =  ui->leMaxTurnAngle->text().toDouble();
    MIN_TURN_ANGLE =  ui->leMinTurnAngle->text().toDouble();
    TURN_ANGLE_SPEED =  ui->leTurnAngleSpeed->text().toDouble();
    GRAVITY_ACC = ui->leAccG->text().toDouble();
    ELECTRIC_FENCE = ui->leElecWidth->text().toDouble();
}
void ConfigDialog::setUIValue()
{
    ui->leSimulation->setText(QString::number(USE_VEH_SIMU));
    ui->leVehMass->setText(QString::number(VEHICLE_MASS));
    ui->leVehWheelBase->setText(QString::number(VEHICLE_WHEEL_BASE));
    ui->leVehLength->setText(QString::number(VEHICLE_LENGTH));
    ui->leVehWidth->setText(QString::number(VEHICLE_WIDTH));
    ui->leVehHeight->setText(QString::number(VEHICLE_HEIGHT));

    ui->leControlPeriod->setText(QString::number(CHAS_MOVE_CTRL_PERIOD));
    ui->leMaxLineSpeed->setText(QString::number(MAX_LINE_SPEED));
    ui->leMaxTurnAngle->setText(QString::number(MAX_TURN_ANGLE));
    ui->leMinTurnAngle->setText(QString::number(MIN_TURN_ANGLE));
    ui->leTurnAngleSpeed->setText(QString::number(TURN_ANGLE_SPEED));
    ui->leAccG->setText(QString::number(GRAVITY_ACC));
    ui->leElecWidth->setText(QString::number(ELECTRIC_FENCE));
}

void ConfigDialog::setUIState(bool bFlag)
{
    ui->leSimulation->setReadOnly(bFlag);
    ui->leVehMass->setReadOnly(bFlag);
    ui->leVehWheelBase->setReadOnly(bFlag);
    ui->leVehLength->setReadOnly(bFlag);
    ui->leVehWidth->setReadOnly(bFlag);
    ui->leVehHeight->setReadOnly(bFlag);
    ui->leControlPeriod->setReadOnly(bFlag);
    ui->leMaxLineSpeed->setReadOnly(bFlag);
    ui->leMaxTurnAngle->setReadOnly(bFlag);
    ui->leMinTurnAngle->setReadOnly(bFlag);
    ui->leTurnAngleSpeed->setReadOnly(bFlag);
    ui->leAccG->setReadOnly(bFlag);
    ui->leElecWidth->setReadOnly(bFlag);
}

void ConfigDialog::on_btConfirm_clicked()/*配置文件确认*/
{
    getUIValue();
    setUIState(true);
}

void ConfigDialog::on_btUpDate_clicked()/*配置文件更新*/
{
    setUIValue();
}

void ConfigDialog::on_btModify_clicked()/*配置文件修改*/
{
   setUIState(false);
}

void ConfigDialog::on_btSave_clicked()/*配置文件保存*/
{
    QString fileName = QFileDialog::getSaveFileName(this,tr("保存文件"),"",tr("地图文件(*.txt)"));
    QFile file(fileName);

    if(!file.open(QIODevice::WriteOnly|QIODevice::Text))
    {
        QMessageBox::critical(this,"critical",tr("路径保存失败"),QMessageBox::Yes,QMessageBox::Yes);
    }
    else
    {
        //在这个界面里头修改参数，在这里写入保存后的文件参数
        QTextStream stream(&file);
        stream.setCodec("utf-8"); //加入这行，并有tr中文，则保存为utf-8的txt
        stream << tr("/*车辆无人控制算法仿真*/\n");
        stream << tr("#define USE_VEH_SIMU ")<<QString::number(USE_VEH_SIMU) << "\n\n";
        stream << tr("/*车辆参数配置部分start*/\n");
        stream << tr("/*kg*/\n");
        stream << tr("#define VEHICLE_MASS ")<<QString::number(VEHICLE_MASS) << "\n";

        stream << tr("/*轴距 m*/\n");
        stream << tr("#define VEHICLE_WHEEL_BASE ")<<QString::number(VEHICLE_WHEEL_BASE) << "\n\n";

        stream << tr("/*车长，单位：m*/\n");
        stream << tr("#define VEHICLE_LENGTH ")<<QString::number(VEHICLE_LENGTH) << "\n";


        stream << tr("/*车宽，单位：m*/\n");
        stream << tr("#define VEHICLE_WIDTH ")<<QString::number(VEHICLE_WIDTH) << "\n";

        stream << tr("/*车高，单位：m*/\n");
        stream << tr("#define VEHICLE_HEIGHT ")<<QString::number(VEHICLE_HEIGHT) << "\n\n";

        stream << tr("/*一个底盘运动控制周期，单位s*/\n");
        stream << tr("#define CHAS_MOVE_CTRL_PERIOD ")<<QString::number(CHAS_MOVE_CTRL_PERIOD) << "\n\n";

        stream << tr("/*车辆参数配置部分end*/\n\n");
        stream << tr("/*最大线速度，单位m/s*/\n");
        stream << tr("#define MAX_LINE_SPEED ")<<QString::number(MAX_LINE_SPEED) << "\n";

        stream << tr("/*最高右转向角度，单位°*/\n");
        stream << tr("#define MAX_TURN_ANGLE ")<<QString::number(MAX_TURN_ANGLE) << "\n";

        stream << tr("/*最高左转向角度，单位°*/\n");
        stream << tr("#define MIN_TURN_ANGLE ")<<QString::number(MIN_TURN_ANGLE) << "\n\n";


        stream << tr("/*最大角速度*/\n");
        stream << tr("#define TURN_ANGLE_SPEED ")<<QString::number(TURN_ANGLE_SPEED) << "\n\n";

        stream << tr("/*重力加速度 m/s²*/\n");
        stream << tr("#define GRAVITY_ACC ")<<QString::number(GRAVITY_ACC) << "\n\n";

        stream << tr("/*电子围栏路宽，单位：m*/\n");
        stream << tr("#define ELECTRIC_FENCE ")<<QString::number(ELECTRIC_FENCE) << "\n"; //添加电子围栏 20220207 qixianyu

        stream.flush();
        file.close();

    }

}
