#ifndef UPDATEFILE_H
#define UPDATEFILE_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QCloseEvent>
#include <QPlainTextEdit>

#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <map>
#include <fstream>

#include <unistd.h>

namespace Ui {
class UpdateFile;
}

struct MsgStruct{
    unsigned int sendtype;
    int sendnum;
    int sendlen;
    char senddata[512];
    unsigned char CRCcheck;
};

struct MsgRespond{
    unsigned int msgrespond;
};

class UpdateFile : public QDialog
{
    Q_OBJECT

public:
    explicit UpdateFile(QWidget *parent = nullptr);
    void closeEvent(QCloseEvent *event);
    ~UpdateFile();

private slots:
    void on_pushButton_choosefile_clicked();
    void on_pushButton_clicked();
    void handle_showmessage(int types, QString messagestr);
    void on_pushButton_2_clicked();

private:
    Ui::UpdateFile *ui;
    QString fileName;
    QString workspace_path;
    int MAXLEN;
    std::vector<std::string> carID;

    int local_port;
    const char* local_ip;
    struct sockaddr_in local_sin;
    struct sockaddr_in to_sin;
    int local_sock;
    volatile int recvlen;
    FILE *send_fp;
    FILE *recv_fp;
    int filelen;
    int recvmsgnum;
    volatile int recv_totallen;
    volatile int sendcheck;
    std::string save_path;

    std::thread thHandle;
    std::thread sendHandle;
    volatile bool is_stop;

    void getCarID();
    void thRecvFunc();
    void SendFunc(char* senddata);
    void SendRecvFunc(char* senddata);
    void HandleRecvMsg(MsgStruct recvdata);
    void HandleSendMsg();
    unsigned char getCheckSum(char* data, int size);

signals:
    void closedialog();
    void showmessage(int types, QString messagestr);
};

#endif // UPDATEFILE_H
