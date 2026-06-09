#ifndef PLATOONDLG_H
#define PLATOONDLG_H

#include <QDialog>
#include <QMessageBox>
#include "pad/qnode.h"
#include <set>
#include <cmath>

namespace Ui
{
    class platoondlg;
}

class platoondlg : public QDialog
{
    Q_OBJECT

public:
    explicit platoondlg(QWidget *parent = nullptr);
    ~platoondlg();
    void connectQnode(QRosNode *qnode);
    void disconnectQnode(QRosNode *qnode);
    double DistanceCalculate(double x1, double y1, double x2, double y2);

signals:
    void sendPlatoonBuildCmd();
    void sendPlatoonDissloveCmd();
    void sendPlatoonJoinCmd(int carnum);
    void sendPlatoonLeaveCmd(int carnum);
    void sendPlatoonColumnCmd();
    void sendPlatoonDiamondCmd();
//    void sendPlatoonMotionStartCmd();
//    void sendPlatoonMotionStopCmd();

private slots:
    void on_btnBuild_clicked();
    void on_btnDisslove_clicked();
    void on_btnJoin_clicked();
    void on_btnLeave_clicked();
    void on_btnColumn_clicked();
    void on_btnDiamond_clicked();

    void Onplatoonmembers(const platoon_msgs::PlatoonMember::ConstPtr& msg);
    void Onplatoonmission(const platoon_msgs::PlatoonMission::ConstPtr& msg);

//    void on_btnSMotionStart_clicked();

//    void on_btnMotionStop_clicked();

    void closeEvent(QCloseEvent *event); //todo

private:
    Ui::platoondlg *ui;

    std::map<uint8, QString> platoontype;
    std::map<uint8, QString> driving_mode;
    std::map<uint8, QString> gear;
    std::map<uint8, int> car_num_list;
    std::vector<uint8> vehicle_num_list;
};

#endif // PLATOONDLG_H
