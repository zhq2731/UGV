#ifndef CHASSIS_LIGHT_HORN_WIPER_DLG_H
#define CHASSIS_LIGHT_HORN_WIPER_DLG_H
#include "pad/alltypes.h"
#include <QDialog>

namespace Ui
{
    class ChassisLightHornWiperDialog;
}

class ChassisLightHornWiperDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ChassisLightHornWiperDialog(QWidget *parent = nullptr);
    ~ChassisLightHornWiperDialog();

private:
    Ui::ChassisLightHornWiperDialog *ui;

signals:
    void sendLeftLightCmd(bool cmd);
    void sendRightLightCmd(bool cmd);
    void sendEmcyFlasherCmd(bool cmd);
    void sendLowBeamCmd(bool cmd);
    void sendHighBeamCmd(bool cmd);
    void sendHornCmd(bool cmd);
    void sendBrakeLightCmd(bool cmd);
    void sendFrontFoggyLightCmd(bool cmd);
    void sendRearFoggyLightCmd(bool cmd);
    void sendPositionLightCmd(bool cmd);
    void sendReverseLightCmd(bool cmd);
    void sendWiperCmd(bool cmd);
    void sendHeadLightCmd(bool cmd);
private slots:
    void onLightHornWiperTopicCallback(ChassisLightHornWiper state);
    void on_btnHeadLight_clicked();
    void on_btnLeftLight_clicked();
    void on_btnRightLight_clicked();
    void on_btnHighBeam_clicked();
    void on_btnLowBeam_clicked();
    void on_btnBrakeLight_clicked();
    void on_btnEmergencyLight_clicked();
    void on_btnFrontFoggyLight_clicked();
    void on_btnRearFoggyLight_clicked();
    void on_btnReverseLight_clicked();
    void on_btnWiper_clicked();
    void on_btnPositionLight_clicked();
    void on_btnHorn_clicked();

private:
    ChassisLightHornWiper state_;

    bool left_light_open_ = false;
    bool right_light_open_ = false;
    bool high_light_open_ = false;
    bool low_light_open_ = false;
    bool brake_light_open_ = false;
    bool emergency_light_open_ = false;
    bool front_foggy_light_open_ = false;
    bool rear_foggy_light_open_ = false;
    bool position_light_open_ = false;
    bool reverse_light_open_ = false;
    bool horn_open_ = false;
    bool wiper_open_ = false;
    bool head_light_open_ = false;
};

#endif // ChassisLightHornWiperDialog_H
