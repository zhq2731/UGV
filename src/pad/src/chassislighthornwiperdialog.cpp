#include "pad/chassislighthornwiperdialog.h"
#include "ui_chassislighthornwiperdlg.h"


#include <iostream>
ChassisLightHornWiperDialog::ChassisLightHornWiperDialog(QWidget *parent) : QDialog(parent),
                                                        ui(new Ui::ChassisLightHornWiperDialog)
{
    ui->setupUi(this);
}

ChassisLightHornWiperDialog::~ChassisLightHornWiperDialog()
{
    delete ui;
}

void ChassisLightHornWiperDialog::onLightHornWiperTopicCallback(ChassisLightHornWiper state)
{
    if(state.left_light == 1)
    {
        ui->leLeftLight->setText("打开");

    }
    else {
        ui->leLeftLight->setText("关闭");
    }

    if(state.right_light == 1)
    {
        ui->leRightLight->setText("打开");

    }
    else {
        ui->leRightLight->setText("关闭");
    }


    if(state.high_light == 1)
    {
        ui->leHighLight->setText("打开");

    }
    else {
        ui->leHighLight->setText("关闭");
    }

    if(state.low_light == 1)
    {
        ui->leLowLight->setText("打开");

    }
    else {
        ui->leLowLight->setText("关闭");
    }


    if(state.brake_light == 1)
    {
        ui->leBrakeLight->setText("打开");

    }
    else {
        ui->leBrakeLight->setText("关闭");
    }

    if(state.head_light == 1)
    {
        ui->leHeadLight->setText("打开");

    }
    else {
        ui->leHeadLight->setText("关闭");
    }

    if(state.emergency_light == 1)
    {
        ui->leEmergencyLight->setText("打开");

    }
    else {
        ui->leEmergencyLight->setText("关闭");
    }

    if(state.front_foggy_light == 1)
    {
        ui->leFrontFoggyLight->setText("打开");

    }
    else {
        ui->leFrontFoggyLight->setText("关闭");
    }


    if(state.rear_foggy_light == 1)
    {
        ui->leRearFoggyLight->setText("打开");

    }
    else {
        ui->leRearFoggyLight->setText("关闭");
    }


    if(state.position_light == 1)
    {
        ui->lePositionLight->setText("打开");

    }
    else {
        ui->lePositionLight->setText("关闭");
    }

    if(state.reverse_light == 1)
    {
        ui->leReverseLight->setText("打开");

    }
    else {
        ui->leReverseLight->setText("关闭");
    }


      if(state.horn == 1)
      {
          ui->leHorn->setText("打开");

      }
      else {
          ui->leHorn->setText("关闭");
      }

      if(state.wiper == 1)
      {
          ui->leWiper->setText("打开");

      }
      else {
          ui->leWiper->setText("关闭");
      }

}

void ChassisLightHornWiperDialog::on_btnHeadLight_clicked()
{
    if(!head_light_open_)
    {
        emit sendHeadLightCmd(true);
        head_light_open_ = true;
        ui->btnHeadLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendHeadLightCmd(false);
        head_light_open_ = false;
        ui->btnHeadLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}


void ChassisLightHornWiperDialog::on_btnLeftLight_clicked()
{
    if(!left_light_open_)
    {
        emit sendLeftLightCmd(true);
        left_light_open_ = true;
        ui->btnLeftLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendLeftLightCmd(false);
        left_light_open_ = false;
        ui->btnLeftLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}

void ChassisLightHornWiperDialog::on_btnRightLight_clicked()
{
    if(!right_light_open_)
    {
        emit sendRightLightCmd(true);
        right_light_open_ = true;
        ui->btnRightLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendRightLightCmd(false);
        right_light_open_ = false;
        ui->btnRightLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}


void ChassisLightHornWiperDialog::on_btnHighBeam_clicked()
{
    if(!high_light_open_)
    {
        emit sendHighBeamCmd(true);
        high_light_open_ = true;
        ui->btnHighBeam->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendHighBeamCmd(false);
        high_light_open_ = false;
        ui->btnHighBeam->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}


void ChassisLightHornWiperDialog::on_btnLowBeam_clicked()
{
    if(!low_light_open_)
    {
        emit sendLowBeamCmd(true);
        low_light_open_ = true;
        ui->btnLowBeam->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendLowBeamCmd(false);
        low_light_open_ = false;
        ui->btnLowBeam->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}

void ChassisLightHornWiperDialog::on_btnBrakeLight_clicked()
{
    if(!brake_light_open_)
    {
        emit sendBrakeLightCmd(true);
        brake_light_open_ = true;
        ui->btnBrakeLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendBrakeLightCmd(false);
        brake_light_open_ = false;
        ui->btnBrakeLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}


void ChassisLightHornWiperDialog::on_btnEmergencyLight_clicked()
{
    if(!emergency_light_open_)
    {
        emit sendEmcyFlasherCmd(true);
        emergency_light_open_ = true;
        ui->btnEmergencyLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendEmcyFlasherCmd(false);
        emergency_light_open_ = false;
        ui->btnEmergencyLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}

void ChassisLightHornWiperDialog::on_btnFrontFoggyLight_clicked()
{
    if(!front_foggy_light_open_)
    {
        emit sendFrontFoggyLightCmd(true);
        front_foggy_light_open_ = true;
        ui->btnFrontFoggyLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendFrontFoggyLightCmd(false);
        front_foggy_light_open_ = false;
        ui->btnFrontFoggyLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}



void ChassisLightHornWiperDialog::on_btnRearFoggyLight_clicked()
{
    if(!rear_foggy_light_open_)
    {
        emit sendRearFoggyLightCmd(true);
        rear_foggy_light_open_ = true;
        ui->btnRearFoggyLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendRearFoggyLightCmd(false);
        rear_foggy_light_open_ = false;
        ui->btnRearFoggyLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }
}


void ChassisLightHornWiperDialog::on_btnReverseLight_clicked()
{
    if(!reverse_light_open_)
    {
        emit sendReverseLightCmd(true);
        reverse_light_open_ = true;
        ui->btnReverseLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendReverseLightCmd(false);
        reverse_light_open_ = false;
        ui->btnReverseLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}

void ChassisLightHornWiperDialog::on_btnWiper_clicked()
{
    if(!wiper_open_)
    {
        emit sendWiperCmd(true);
        wiper_open_ = true;
        ui->btnWiper->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendWiperCmd(false);
        wiper_open_ = false;
        ui->btnWiper->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}

void ChassisLightHornWiperDialog::on_btnPositionLight_clicked()
{
    if(!position_light_open_)
    {
        emit sendPositionLightCmd(true);
        position_light_open_ = true;
        ui->btnPositionLight->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendPositionLightCmd(false);
        position_light_open_ = false;
        ui->btnPositionLight->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}

void ChassisLightHornWiperDialog::on_btnHorn_clicked()
{
    if(!horn_open_)
    {
        emit sendHornCmd(true);
        horn_open_ = true;
        ui->btnHorn->setStyleSheet("QPushButton{background-color:green; color:white}");
    }
    else
    {
        emit sendHornCmd(false);
        horn_open_ = false;
        ui->btnHorn->setStyleSheet("QPushButton{font:11pt;width:120px;color:rgb(50,223,249); "
                                        "background-color:rgb(28,50,69); border-radius:0px; border:1px groove gray;border-style:outset;}"
                                        "QPushButton:hover{background-color:rgb(50,223,249);color:rgb(28,50,69);}"
                                        "QPushButton:pressed{color:yellow;border-style:inset;}");
    }

}
