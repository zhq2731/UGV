#include "pad/platoondlg.h"
#include "ui_platoondlg.h"
#include "pad/qnode.h"
platoondlg::platoondlg(QWidget *parent) : QDialog(parent),
                                          ui(new Ui::platoondlg)
{
    ui->setupUi(this);
    QPalette p = this->palette();
    p.setColor(QPalette::Window, QColor(35, 35, 35));
    this->setPalette(p); /*设置窗口背景颜色*/
    platoontype[0] = QString("NONE");
    platoontype[1] = QString("BUILD");
    platoontype[2] = QString("JOIN");
    platoontype[3] = QString("LEAVE");
    platoontype[4] = QString("DISSOLVE");
    platoontype[5] = QString("COLUMN");
    platoontype[6] = QString("DIAMOND");
    platoontype[7] = QString("RUNNING");
    driving_mode[0] = QString("人工");
    driving_mode[1] = QString("自动");
    gear[0] = QString("N");
    gear[1] = QString("D1");
    gear[7] = QString("R");
}

platoondlg::~platoondlg()
{
    delete ui;
}
void platoondlg::closeEvent(QCloseEvent *event)
{
}

void platoondlg::connectQnode(QRosNode *qnode)
{
    connect(this, SIGNAL(sendPlatoonBuildCmd()), qnode, SLOT(OnSendPlatoonBuildCmd()));
    connect(this, SIGNAL(sendPlatoonDissloveCmd()), qnode, SLOT(OnSendPlatoonDissloveCmd()));
    connect(this, SIGNAL(sendPlatoonJoinCmd(int)), qnode, SLOT(OnSendPlatoonJoinCmd(int)));
    connect(this, SIGNAL(sendPlatoonLeaveCmd(int)), qnode, SLOT(OnSendPlatoonLeaveCmd(int)));
    connect(this, SIGNAL(sendPlatoonColumnCmd()), qnode, SLOT(OnSendPlatoonColumnCmd()));
    connect(this, SIGNAL(sendPlatoonDiamondCmd()), qnode, SLOT(OnSendPlatoonDiamondCmd()));

    connect(qnode, SIGNAL(emitplatoonmembers(const platoon_msgs::PlatoonMember::ConstPtr)),
            this, SLOT(Onplatoonmembers(const platoon_msgs::PlatoonMember::ConstPtr)));
    connect(qnode, SIGNAL(emitplatoonmission(const platoon_msgs::PlatoonMission::ConstPtr)),
            this, SLOT(Onplatoonmission(const platoon_msgs::PlatoonMission::ConstPtr)));
//    connect(this, SIGNAL(sendPlatoonMotionStartCmd()), qnode, SLOT(OnSendPlatoonMotionStartCmd()));
//    connect(this, SIGNAL(sendPlatoonMotionStopCmd()), qnode, SLOT(OnSendPlatoonMotionStopCmd()));
}
void platoondlg::disconnectQnode(QRosNode *qnode)
{
    disconnect(this, SIGNAL(sendPlatoonBuildCmd()), qnode, SLOT(OnSendPlatoonBuildCmd()));
    disconnect(this, SIGNAL(sendPlatoonDissloveCmd()), qnode, SLOT(OnSendPlatoonDissloveCmd()));
    disconnect(this, SIGNAL(sendPlatoonJoinCmd()), qnode, SLOT(OnSendPlatoonJoinCmd()));
    disconnect(this, SIGNAL(sendPlatoonLeaveCmd()), qnode, SLOT(OnSendPlatoonLeaveCmd()));
    disconnect(this, SIGNAL(sendPlatoonColumnCmd()), qnode, SLOT(OnSendPlatoonColumnCmd()));
    disconnect(this, SIGNAL(sendPlatoonDiamondCmd()), qnode, SLOT(OnSendPlatoonDiamondCmd()));
//    disconnect(this, SIGNAL(sendPlatoonMotionStartCmd()), qnode, SLOT(OnSendPlatoonMotionStartCmd()));
//    disconnect(this, SIGNAL(sendPlatoonMotionStopCmd()), qnode, SLOT(OnSendPlatoonMotionStopCmd()));
}

void platoondlg::on_btnBuild_clicked()
{
    emit sendPlatoonBuildCmd();
}

void platoondlg::on_btnDisslove_clicked()
{
    emit sendPlatoonDissloveCmd();
}

void platoondlg::on_btnJoin_clicked()
{
    uint8 carnum;
    if(ui->radioButton->isChecked()){
        for (const auto& it : car_num_list){
            printf("carnum: %d, carnumlist: %d\n", it.first, it.second);
            if(it.second == 2) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) == vehicle_num_list.end()){
            printf("carnum: %d\n", carnum);
            emit sendPlatoonJoinCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车1已在编队中。", QMessageBox::Ok);
        }
    }
    else if (ui->radioButton_2->isChecked()) {
        for (const auto& it : car_num_list){
            if(it.second == 3) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) == vehicle_num_list.end()){
            emit sendPlatoonJoinCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车2已在编队中。", QMessageBox::Ok);
        }
    }
    else if (ui->radioButton_3->isChecked()) {
        for (const auto& it : car_num_list){
            if(it.second == 4) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) == vehicle_num_list.end()){
            emit sendPlatoonJoinCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车3已在编队中。", QMessageBox::Ok);
        }
    }
    else{
        QMessageBox::warning(NULL, "警告", "请选中车辆。", QMessageBox::Ok);
    }
}

void platoondlg::on_btnLeave_clicked()
{
    uint8 carnum;
    if(ui->radioButton->isChecked()){
        for (const auto& it : car_num_list){
            if(it.second == 2) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) != vehicle_num_list.end()){
            emit sendPlatoonLeaveCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车1不在编队中。", QMessageBox::Ok);
        }
    }
    else if (ui->radioButton_2->isChecked()) {
        for (const auto& it : car_num_list){
            if(it.second == 3) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) != vehicle_num_list.end()){
            emit sendPlatoonLeaveCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车2不在编队中。", QMessageBox::Ok);
        }
    }
    else if (ui->radioButton_3->isChecked()) {
        for (const auto& it : car_num_list){
            if(it.second == 4) carnum = it.first;
        }
        if(std::find(vehicle_num_list.begin(), vehicle_num_list.end(), carnum) != vehicle_num_list.end()){
            emit sendPlatoonLeaveCmd(carnum);
        }
        else{
            QMessageBox::warning(NULL, "警告", "跟随车3不在编队中。", QMessageBox::Ok);
        }
    }
    else{
        QMessageBox::warning(NULL, "警告", "请选中车辆。", QMessageBox::Ok);
    }
}

void platoondlg::on_btnColumn_clicked()
{
    emit sendPlatoonColumnCmd();
}

void platoondlg::on_btnDiamond_clicked()
{
    emit sendPlatoonDiamondCmd();
}

void platoondlg::Onplatoonmembers(const platoon_msgs::PlatoonMember::ConstPtr& msg)
{
    int car_num;
    QString str_temp;
    double lastcar_x, lastcar_y, distance;
    auto it = car_num_list.find(msg->num);
    if (it != car_num_list.end()){
        car_num = it->second;
    }
    else{
        car_num = car_num_list.size() + 1;
        car_num_list[msg->num] = car_num;
    }
    switch (car_num) {
    case 1:
        ui->carlable_display_1->setText(QString::fromStdString(msg->id));
        ui->positionX_display_1->setText(QString::number(msg->position.x, 'g', 10));
        ui->positionY_display_1->setText(QString::number(msg->position.y, 'g', 10));
        ui->heading_display_1->setText(QString::number(msg->heading, 'g', 6));
        ui->linear_velocity_display_1->setText(QString::number(msg->linear_velocity, 'g', 6));
        ui->spacing_distance_display_1->setText(QString::number(msg->spacing_distance, 'g', 6));
        ui->gear_display_1->setText(gear[msg->gear]);
        ui->driving_mode_display_1->setText(driving_mode[msg->driving_mode]);
        ui->command_type_display_1->setText(platoontype[msg->type]);
        break;
    case 2:
        ui->carlable_display_2->setText(QString::fromStdString(msg->id));
        ui->positionX_display_2->setText(QString::number(msg->position.x, 'g', 10));
        ui->positionY_display_2->setText(QString::number(msg->position.y, 'g', 10));
        ui->heading_display_2->setText(QString::number(msg->heading, 'g', 6));
        ui->linear_velocity_display_2->setText(QString::number(msg->linear_velocity, 'g', 6));
        str_temp = ui->positionX_display_1->text();
        lastcar_x = str_temp.toFloat();
        str_temp = ui->positionY_display_1->text();
        lastcar_y = str_temp.toFloat();
        distance = DistanceCalculate(lastcar_x, lastcar_y, msg->position.x, msg->position.y);
        ui->spacing_distance_display_2->setText(QString::number(distance, 'g', 6));
        ui->gear_display_2->setText(gear[msg->gear]);
        ui->driving_mode_display_2->setText(driving_mode[msg->driving_mode]);
        ui->command_type_display_2->setText(platoontype[msg->type]);
        break;
    case 3:
        ui->carlable_display_3->setText(QString::fromStdString(msg->id));
        ui->positionX_display_3->setText(QString::number(msg->position.x, 'g', 10));
        ui->positionY_display_3->setText(QString::number(msg->position.y, 'g', 10));
        ui->heading_display_3->setText(QString::number(msg->heading, 'g', 6));
        ui->linear_velocity_display_3->setText(QString::number(msg->linear_velocity, 'g', 6));
        str_temp = ui->positionX_display_2->text();
        lastcar_x = str_temp.toFloat();
        str_temp = ui->positionY_display_2->text();
        lastcar_y = str_temp.toFloat();
        distance = DistanceCalculate(lastcar_x, lastcar_y, msg->position.x, msg->position.y);
        ui->spacing_distance_display_3->setText(QString::number(distance, 'g', 6));
        ui->gear_display_3->setText(gear[msg->gear]);
        ui->driving_mode_display_3->setText(driving_mode[msg->driving_mode]);
        ui->command_type_display_3->setText(platoontype[msg->type]);
        break;
    case 4:
        ui->carlable_display_4->setText(QString::fromStdString(msg->id));
        ui->positionX_display_4->setText(QString::number(msg->position.x, 'g', 10));
        ui->positionY_display_4->setText(QString::number(msg->position.y, 'g', 10));
        ui->heading_display_4->setText(QString::number(msg->heading, 'g', 6));
        ui->linear_velocity_display_4->setText(QString::number(msg->linear_velocity, 'g', 6));
        str_temp = ui->positionX_display_3->text();
        lastcar_x = str_temp.toFloat();
        str_temp = ui->positionY_display_3->text();
        lastcar_y = str_temp.toFloat();
        distance = DistanceCalculate(lastcar_x, lastcar_y, msg->position.x, msg->position.y);
        ui->spacing_distance_display_4->setText(QString::number(distance, 'g', 6));
        ui->gear_display_4->setText(gear[msg->gear]);
        ui->driving_mode_display_4->setText(driving_mode[msg->driving_mode]);
        ui->command_type_display_4->setText(platoontype[msg->type]);
        break;
    }
}

void platoondlg::Onplatoonmission(const platoon_msgs::PlatoonMission::ConstPtr& msg)
{
    switch (msg->command_type) {
    case 1:
        vehicle_num_list = msg->vehicle_list;
        for (size_t i = 0; i < vehicle_num_list.size(); i++){
            car_num_list[vehicle_num_list[i]] = i+1;
        }
        ui->carnum_1->setText("引导车");
        ui->carnum_2->setText("跟随车1");
        ui->carnum_3->setText("跟随车2");
        ui->carnum_4->setText("跟随车3");
        break;
    case 2:{
        vehicle_num_list.push_back(msg->num);
        for (size_t i = 0; i < vehicle_num_list.size(); i++){
            car_num_list[vehicle_num_list[i]] = i+1;
        }
    }
        break;
    case 3:
        vehicle_num_list.erase(std::remove(vehicle_num_list.begin(), vehicle_num_list.end(), msg->num), vehicle_num_list.end());
        break;
    case 4:
        vehicle_num_list.clear();
        ui->carnum_1->setText("车辆1");
        ui->carnum_2->setText("车辆2");
        ui->carnum_3->setText("车辆3");
        ui->carnum_4->setText("车辆4");
        break;
    default:
        break;
    }
}

double platoondlg::DistanceCalculate(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}
//void platoondlg::on_btnSMotionStart_clicked()
//{
//    emit sendPlatoonMotionStartCmd();
//}

//void platoondlg::on_btnMotionStop_clicked()
//{
//    emit sendPlatoonMotionStopCmd();
//}
