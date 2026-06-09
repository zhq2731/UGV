#include "pad/vehiclewirecontrol.h"
#include "ui_vehiclewirecontrol.h"


VehicleWireControl::VehicleWireControl(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VehicleWireControl)
{
    ui->setupUi(this);
}

VehicleWireControl::~VehicleWireControl()
{
    delete ui;
}

void VehicleWireControl::on_btnDrivingModeSet_clicked()
{
    int index = ui->comBoxDrivingMode->currentIndex();

    if (0 == index) //
    {
//      MsgTableDisp("操作输入", "人控模式设置");
      emit sendDriveModeCmd(0);
      //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = CHAS_WORK_MODE_MAN;
    }
    else if (1 == index)
    {
//      MsgTableDisp("操作输入", "无人驾驶模式模式设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = CHAS_WORK_MODE_AUTO_DRIVE;
      emit sendDriveModeCmd(1);
    }
    else if (2 == index)
    {
//      MsgTableDisp("操作输入", "遥控模式模式设置");
      emit sendDriveModeCmd(2);
    }
    else if (3 == index)
    {
//      MsgTableDisp("操作输入", "人工反向驾驶模式设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = 0;
      emit sendDriveModeCmd(3);
    }
    else if (4 == index)
    {
//      MsgTableDisp("操作输入", "横向控制模式设置");

      emit sendDriveModeCmd(4);
    }
    else if (5 == index)
    {
//      MsgTableDisp("操作输入", "纵向控制模式设置");

      emit sendDriveModeCmd(5);
    }
    else
    {
//      MsgTableDisp("操作输入", "无效模式设置");
    }

}

void VehicleWireControl::on_btnEngineOn_clicked()
{
      emit sendEngineCmd(true);

}

void VehicleWireControl::on_btnEngineOff_clicked()
{
      emit sendEngineCmd(false);

}

void VehicleWireControl::on_btnLowVoltageOn_clicked()
{
    emit sendLowVoltageCmd(true);

}

void VehicleWireControl::on_btnLowVoltageOff_clicked()
{
    emit sendLowVoltageCmd(false);

}

void VehicleWireControl::on_btnHighVoltageOn_clicked()
{
    emit sendHighVoltageCmd(true);

}

void VehicleWireControl::on_btnHighVoltageOff_clicked()
{
    emit sendHighVoltageCmd(false);

}

void VehicleWireControl::on_btnGearSet_clicked()
{
    int index = ui->comBoxGear->currentIndex();

    if (1 == index)
    {
//      MsgTableDisp("操作输入", "D档位设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_D;
      emit sendTargetGearCmd(1);
    }
    else if (2 == index)
    {
//      MsgTableDisp("操作输入", "R档位设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_R;
      emit sendTargetGearCmd(7);
    }
    else if (0 == index)
    {
//      MsgTableDisp("操作输入", "N档位设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_N;
      emit sendTargetGearCmd(0);
    }
    else if (3 == index)
    {
//      MsgTableDisp("操作输入", "P档位设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_N;
      emit sendTargetGearCmd(2);
    }
    else
    {
//      MsgTableDisp("操作输入", "无效档位设置");
      //    stFP2VCMsg.stVC2ChasCmd.ucGear = 0;
    }


}

void VehicleWireControl::on_btnParkingBrakeOn_clicked()
{
//    MsgTableDisp("操作输入", "驻车制动");
    emit sendParkBrakeCmd(true);
}

void VehicleWireControl::on_btnParkingBrakeOff_clicked()
{
    emit sendParkBrakeCmd(false);
}


void VehicleWireControl::on_btnSendThrottleCmd_clicked()
{
    double throttle_value = ui->editThrottleValue->text().toDouble();
    double brake_value = ui->editBrakeValue->text().toDouble();

//    std::cout << "throttle_value vlaue is " << throttle_value << std::endl;
    if (throttle_value > 100 || throttle_value < 0)
    {
      QMessageBox::information(Q_NULLPTR, tr("提示"), tr("超出阈值10%"));
      return;
    }

    if (brake_value > 0)
    {
      emit sendTargetThrottleAndBrakePct(0, brake_value);
    }
    else if (throttle_value > 0)
    {
      emit sendTargetThrottleAndBrakePct(throttle_value, 0);
    }
    else
    {
      emit sendTargetThrottleAndBrakePct(0, 0);
    }
}

void VehicleWireControl::on_btnSendSteerWheelCmd_clicked()
{
    double steer_value = ui->editSteerWheelValue->text().toDouble();
    double steer_speed_value = ui->editSteerWheelSpeedValue->text().toDouble();

    // std::cout << "steer vlaue is " << steer_value << std::endl;
    if (steer_value > 1000 || steer_value < -1000)
    {
      QMessageBox::information(Q_NULLPTR, tr("提示"), tr("角度超出阈值"));
      return;
    }

    if (steer_speed_value > 250 || steer_speed_value < 0)
    {
      QMessageBox::information(Q_NULLPTR, tr("提示"), tr("角速度超出阈值"));
      return;
    }

    emit sendTargetSteeringAngleAndAngleSpeedCmd(steer_value, steer_speed_value);

}



// 接收底盘状态信息，并将其转发给pad
void VehicleWireControl::onVehicleChassisState(VehicleChassisStateST vehicle_chassis_state_msg)
{
  // 驾驶模式
//  ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
//  ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");      // eps
//  ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");    // drive
//  ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}"); // bcm
//  ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");     // gear
//  ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");    // brake
//  ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");      // estop

  if (0 == vehicle_chassis_state_msg.driving_mode)
  {
    ui->labDrivingMode->setText("人控模式");
    ui->labDrivingMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else if (vehicle_chassis_state_msg.driving_mode == 5){
    ui->labDrivingMode->setText("单纵向驾驶模式");
    ui->labDrivingMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else if (vehicle_chassis_state_msg.driving_mode == 4){
    ui->labDrivingMode->setText("单横向驾驶模式");
    ui->labDrivingMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }  
  else if (vehicle_chassis_state_msg.driving_mode == 1)
  {
      if(vehicle_chassis_state_msg.mode_flag == 0) //auto
      {
          ui->labDrivingMode->setText("自动驾驶模式");
          ui->labDrivingMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//          ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      }
      else if(vehicle_chassis_state_msg.mode_flag == 1) //remote
      {
          ui->labDrivingMode->setText("遥控驾驶模式");
//          ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      }
      else if(vehicle_chassis_state_msg.mode_flag == 2) //reverse
      {
          ui->labDrivingMode->setText("人工反向驾驶模式");
//          ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
//          ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
//          ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      }
  }
  else
  {
//    if (vehicle_chassis_state_msg.adu_mode == 1)
//    {
//      ui->labChasWorkMode->setText("纵向控制模式");
//      ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//      ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");    // drive
//      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); // bcm
//    }

//    if (vehicle_chassis_state_msg.eps_mode == 1)
//    {
//      ui->labChasWorkMode->setText("横向控制模式");
//      ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//      ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      // eps
//      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); // bcm
//    }
  }

  QString strText =
      QString("%1").arg((vehicle_chassis_state_msg.current_velocity * 3.6), 0, 'f', 1); /*底盘车速，20210419修改成km/h显示*/
  ui->labChasVehSpeed->setText(strText);

  strText = QString("%1").arg((vehicle_chassis_state_msg.steering_wheel_angle), 0, 'f', 2); /*底盘前轮转角*/
  ui->labSteerWheel->setText(strText);

  strText = QString("%1").arg((vehicle_chassis_state_msg.steering_wheel_angle_speed), 0, 'f', 1); /*底盘前轮转角速度*/
  ui->labSteerWheelSpeed->setText(strText);



  ui->labGear->setStyleSheet(
      "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");

  switch (vehicle_chassis_state_msg.gear_location) /*变速箱档位*/
  {
  case 0:
  {
    ui->labGear->setText("N");
    break;
  }
  case 7:
  {
    ui->labGear->setText("R1");
    break;
  }
  case 1:
  {
    ui->labGear->setText("D1");
    break;
  }
  case 2:
  {
    ui->labGear->setText("P");
    break;
  }
  default:
  {
    break;
  }
  }

  strText = QString("%1").arg(vehicle_chassis_state_msg.throttle_pedal); /*油门开度*/
  ui->labThrottle->setText(strText);

//  strText = QString("%1").arg(vehicle_chassis_state_msg.remote_button_status); /*远程按钮反馈*/
//  ui->labCHASRemote_button_status->setText(strText);

//  //    strText = QString("%1").arg(stChas2VCWorkData.ucTurnArmOpening);//转向开度
//  //    ui->labChasTurnArmOpen->setText(strText);

  strText = QString("%1").arg(vehicle_chassis_state_msg.brake_pedal); /*刹车开度*/
  ui->labBrake->setText(strText);

  // ui->labEngineWorkState->setStyleSheet(
  //     "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");

  // if (vehicle_chassis_state_msg.current_engine_torque > 0)
  // {
  //   ui->labEngineWorkState->setText("启动完成");
  // }
  // else
  // {
  //   ui->labEngineWorkState->setText("停止完成");
  // }

  // std::cout << vehicle_chassis_state_msg.steer_intervene << std::endl;

  if (vehicle_chassis_state_msg.is_ready)
  {
    ui->labReady->setText("准备好");
    ui->labReady->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");

  }
  else
  {
    ui->labReady->setText("未准备好");
    ui->labReady->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:red; color:white}");

  }

//  if (vehicle_chassis_state_msg.steer_intervene)
//  {
//    ui->labCHASSteer_intervene->setText("干预");
//    ui->labCHASSteer_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
//  }
//  else
//  {
//    ui->labCHASSteer_intervene->setText("无干预");
//    ui->labCHASSteer_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//  }

//  if (vehicle_chassis_state_msg.brake_intervene)
//  {
//    ui->labCHASBrake_intervene->setText("干预");
//    ui->labCHASBrake_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
//  }
//  else
//  {
//    ui->labCHASBrake_intervene->setText("无干预");
//    ui->labCHASBrake_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//  }

//  if (vehicle_chassis_state_msg.estop_intervene)
//  {
//    ui->labCHASEstop_intervene->setText("干预");
//    ui->labCHASEstop_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
//  }
//  else
//  {
//    ui->labCHASEstop_intervene->setText("无干预");
//    ui->labCHASEstop_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//  }

//  if (vehicle_chassis_state_msg.timeout_status)
//  {
//    ui->labCHASTimeout_status->setText("超时");
//    ui->labCHASTimeout_status->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
//  }
//  else
//  {
//    ui->labCHASTimeout_status->setText("未超时");
//    ui->labCHASTimeout_status->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
//  }

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.FuelCapacity); /*油箱油量*/
  //  ui->labOilRemain->setText(strText);

  //  if (true == vehicle_chassis_state_msg.EmcyBarkeSts) /*底盘紧急制动状态*/
  //  {
  //    ui->labChasEmerencyBrakState->setText("紧急制动中");
  //    ui->labChasEmerencyBrakState->setStyleSheet(
  //        "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  //  }
  //  else
  //  {
  //    ui->labChasEmerencyBrakState->setText("未紧急制动");
  //    ui->labChasEmerencyBrakState->setStyleSheet(
  //        "QLabel{font:11pt '仿宋' bold; background-color:rgb(35,35,35); color:rgb(21,197,212);}");
  //  }

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.AirTankPressure); /**/
  //  ui->labChasBatVolt->setText(strText);

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.Range); /*底盘里程*/
  //  ui->labChasMileage->setText(strText);

  if (vehicle_chassis_state_msg.parking_brake)
  {
    ui->labParkingBrake->setText("驻车制动");
    ui->labParkingBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else
  {
    ui->labParkingBrake->setText("解除驻车");
    ui->labParkingBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

//  m_uiSendChassisNo++;
//  if (m_uiSendChassisNo == m_uiSendChassisNoThres)
//  {
//    PadChassisToRemoteControlReportST pad_chassis_send_msg;
//    pad_chassis_send_msg.stNetHeader.usMsgType = TANK_TO_REMOTE_CONTROL_STATE_MSG;
//    pad_chassis_send_msg.stNetHeader.usMsgLen = sizeof(PadChassisToRemoteControlReportST);
//    pad_chassis_send_msg.stNetHeader.uiMsgTime = 0;
//    pad_chassis_send_msg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
//    pad_chassis_send_msg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
//    pad_chassis_send_msg.stNetHeader.usAck = NET_MSG_NOACK;

//    pad_chassis_send_msg.fTimeStampS = 0;                                                  // 单位s，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间
//    pad_chassis_send_msg.fTimeStampNs = 0.;                                                // 纳秒
//    pad_chassis_send_msg.usErrorState = 0x00;                                              // 底盘故障状态
//    pad_chassis_send_msg.usErrorCode = 0;                                                  // 底盘故障编码
//    pad_chassis_send_msg.us_total_kilometres = vehicle_chassis_state_msg.total_kilometres; // 里程

//    if (vehicle_chassis_state_msg.current_engine_speed > 500) // 发动机状态   转速大于500：启动1  转速小于500：关闭0
//    {
//      pad_chassis_send_msg.ucEngineState = 1;
//    }
//    else
//    {
//      pad_chassis_send_msg.ucEngineState = 0;
//    }
//    // std::cout<<vehicle_chassis_state_msg.gear_location<<std::endl;
//    pad_chassis_send_msg.usParkingBrake = vehicle_chassis_state_msg.parking_brake; // 驻车制动
//    pad_chassis_send_msg.usGearLocation = vehicle_chassis_state_msg.gear_location;
//    pad_chassis_send_msg.usLeftLight = vehicle_chassis_state_msg.left_light;
//    pad_chassis_send_msg.usRightLight = vehicle_chassis_state_msg.right_light;
//    pad_chassis_send_msg.usOilPercent = vehicle_chassis_state_msg.remaining_oil;
//    pad_chassis_send_msg.usVehicleSpeed = vehicle_chassis_state_msg.current_velocity * 3.6; // 前面单位km
//    pad_chassis_send_msg.usEngineSpeed = vehicle_chassis_state_msg.current_engine_speed;

//    pad_chassis_send_msg.usDrivingMode = vehicle_chassis_state_msg.driving_mode; // 反馈驾驶模式
//    pad_chassis_send_msg.bRecTask = rec_task_;                                   // 收到任务文件反馈
//    // std::cout << "rec_task is" << rec_task_ << std::endl;

//    emit SendNetMsg(QByteArray((char *)&pad_chassis_send_msg, sizeof(PadChassisToRemoteControlReportST)),
//                    sizeof(PadChassisToRemoteControlReportST), QHostAddress(REMOTE_CONTROL_IP),
//                    TANK_TASKPOINTS_PORT);

//    m_uiSendChassisNo = 0;
//  }
}


void VehicleWireControl::on_btnResetVehicleState_clicked()
{
    emit sendVehicleResetCmd();

}

void VehicleWireControl::on_btnMoveStart_clicked()
{
    sendMotionStart(1);
}

void VehicleWireControl::on_btnMoveStop_clicked()
{
    sendMotionStart(0);
}

void VehicleWireControl::on_btnEmgencyStop_clicked()
{
    sendEmcyBrakeCmd(true);
}

