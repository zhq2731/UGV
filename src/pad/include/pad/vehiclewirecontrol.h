#ifndef VEHICLEWIRECONTROL_H
#define VEHICLEWIRECONTROL_H

#include <QDialog>
#include "pad/alltypes.h"
#include <QMessageBox>
namespace Ui {
class VehicleWireControl;
}

class VehicleWireControl : public QDialog
{
    Q_OBJECT

public:
    explicit VehicleWireControl(QWidget *parent = nullptr);
    ~VehicleWireControl();


signals:
    void sendDriveModeCmd(uint8 drive_mode);
    void sendTargetGearCmd(uint8 gear_cmd);
    void sendEngineCmd(bool engine_cmd);
    void sendLowVoltageCmd(bool cmd);
    void sendHighVoltageCmd(bool cmd);
    void sendTargetThrottleAndBrakePct(float throttle_pct, float brake_pct);
    void sendTargetSteeringAngleAndAngleSpeedCmd(float target_angle, float angle_speed);
    void sendParkBrakeCmd(bool cmd);
    void sendVehicleResetCmd();
    void sendMotionStart(uint8 cmd);
    void sendEmcyBrakeCmd(bool cmd);

private slots:

    void on_btnDrivingModeSet_clicked();

    void on_btnEngineOn_clicked();

    void on_btnEngineOff_clicked();

    void on_btnLowVoltageOn_clicked();

    void on_btnLowVoltageOff_clicked();

    void on_btnHighVoltageOn_clicked();

    void on_btnHighVoltageOff_clicked();

    void on_btnGearSet_clicked();

    void on_btnParkingBrakeOn_clicked();

    void on_btnParkingBrakeOff_clicked();

    void on_btnSendThrottleCmd_clicked();

    void on_btnSendSteerWheelCmd_clicked();

    void onVehicleChassisState(VehicleChassisStateST vehicle_chassis_state_msg);

    void on_btnResetVehicleState_clicked();

    void on_btnMoveStart_clicked();

    void on_btnMoveStop_clicked();

    void on_btnEmgencyStop_clicked();


private:
    Ui::VehicleWireControl *ui;
};

#endif // VEHICLEWIRECONTROL_H
