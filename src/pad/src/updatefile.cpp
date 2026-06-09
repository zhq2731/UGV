#include "pad/updatefile.h"
#include "ui_updatefile.h"

UpdateFile::UpdateFile(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UpdateFile)
{
    ui->setupUi(this);
    ui->plainTextEdit_info->setReadOnly(true);
    workspace_path = QDir::currentPath();//获取当前程序工作路径
    getCarID();//获取所有车辆IP地址
    MAXLEN = 512;//设置一次传输字节数，修改要同时修改MsgStruct里senddata大小
    recvmsgnum = 0;
    connect(this, &UpdateFile::showmessage, this, &UpdateFile::handle_showmessage);

    local_port = 9012;//设置端口号
    recvlen = sizeof(struct sockaddr_in);
    std::string localID = carID[0];
    size_t pos = localID.find(':');
    localID = localID.substr(pos + 1);
    local_ip = localID.c_str();//获取本车IP
    printf("local car ID : %s \n", localID.c_str());
    memset(&local_sin, 0, sizeof(local_sin));
    local_sin.sin_family = AF_INET;
    local_sin.sin_addr.s_addr = inet_addr(local_ip);
    local_sin.sin_port = htons(local_port);//设置接收IP和端口信息

    memset(&to_sin, 0, sizeof(to_sin));
    to_sin.sin_family = AF_INET;
    to_sin.sin_port = htons(local_port);

    local_sock = socket( AF_INET, SOCK_DGRAM, 0 );//设置为UDP传输
    int bind_state = bind(local_sock, (struct sockaddr*)&local_sin, sizeof(local_sin) );//绑定端口
    if(bind_state == -1){
        perror("bind error");
        exit(-1);
        }
    thHandle = std::thread(&UpdateFile::thRecvFunc,this);//开启接收线程
    is_stop = true;
}

UpdateFile::~UpdateFile()
{
    is_stop = false;
    shutdown(local_sock, 2);
    thHandle.join();
    printf("delete thRecvFunc\n");
    delete ui;
}

//选择文件路径
void UpdateFile::on_pushButton_choosefile_clicked()
{
    fileName = QFileDialog::getOpenFileName(
                this,
                tr("Open File"),
                "/home",
                tr("All Files (*)")
            );
        if (!fileName.isEmpty())
        {
            // 用户选择了一个文件夹，可以在这里进行相应的操作
            ui->lineEdit->setText(fileName);
        }
        else{
            QMessageBox::warning(this,u8"文件夹路径有误",u8"请确认路径");
        }
}

void UpdateFile::closeEvent(QCloseEvent *event)
{
    emit closedialog();
}

void UpdateFile::on_pushButton_clicked()
{
    QByteArray arr = fileName.toUtf8();
    char* ptr = arr.data();//将文件路径转为char*格式
    //打开文件
    if((send_fp=fopen(ptr,"r"))==NULL)
    {
        QMessageBox::warning(NULL, "警告", "无法打开文件，请核对文件路径。", QMessageBox::Ok);
        printf("file open failure!!\n");
        return;
    }
    else{
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(false);
        emit showmessage(0, QString("文件传输开始"));
        sendHandle = std::thread(&UpdateFile::HandleSendMsg,this);//开启文件发送线程
        sendHandle.detach();//线程分离
    }
}

void UpdateFile::on_pushButton_2_clicked()
{
    ui->pushButton->setEnabled(false);
    ui->pushButton_2->setEnabled(false);
    MsgStruct sendmsg;
    memset(&sendmsg, 0, sizeof (MsgStruct));
    if(carID.size() < 2){
        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        std::string sendstr = "未读取到目标车辆IP，请检查IP配置文件";
        emit showmessage(1, QString::fromStdString(sendstr));
        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        return;
    }
    //对每一个车辆IP分别传输文件
    for (std::vector<std::string>::iterator it = carID.begin()+1; it != carID.end(); it++){
        std::string toID = *it;
        size_t pos = toID.find(':');
        std::string car_number = toID.substr(0, pos);
        toID = toID.substr(pos + 1);//读取目标车辆IP
        to_sin.sin_addr.s_addr = inet_addr(toID.c_str());//将发送目标IP设为目标车辆IP
        const char* compilestr = "compilecommand";
        sendmsg.sendtype = 99;
        memcpy(&sendmsg.senddata, compilestr, strlen(compilestr));
        SendFunc((char*)&sendmsg);
    }
    std::string sendstr = "编译命令已发出";
    emit showmessage(0, QString::fromStdString(sendstr));
    ui->pushButton->setEnabled(true);
    ui->pushButton_2->setEnabled(true);
}

void UpdateFile::HandleSendMsg()
{
    int totallen;//发送的总字节数
    int sendmsgnum = 0;
    MsgStruct sendmsg;
    std::string file_path = fileName.toStdString();
    int stringdex = file_path.rfind("UGV/");
    if(stringdex == -1){
        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        std::string sendstr = "文件未放入UGV文件下对应位置，传输失败";
        emit showmessage(2, QString::fromStdString(sendstr));
        return;
    }
    file_path = file_path.substr(stringdex+4);//提取文件在UGV文件夹下的路径
    std::string sendstr = "文件保存路径：UGV/"+file_path;
    emit showmessage(0, QString::fromStdString(sendstr));
    //未获取到其他车辆IP，结束发送
    if(carID.size() < 2){
        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        std::string sendstr = "未读取到目标车辆IP，请检查IP配置文件";
        emit showmessage(1, QString::fromStdString(sendstr));
        return;
    }
    //对每一个车辆IP分别传输文件
    for (std::vector<std::string>::iterator it = carID.begin()+1; it != carID.end(); it++){
        std::string toID = *it;
        size_t pos = toID.find(':');
        std::string car_number = toID.substr(0, pos);
        toID = toID.substr(pos + 1);//读取目标车辆IP
        to_sin.sin_addr.s_addr = inet_addr(toID.c_str());//将发送目标IP设为目标车辆IP
        printf("send to car ID : %s \n", toID.c_str());
        sendstr = "开始传输第"+car_number+"辆车的文件";
        emit showmessage(0, QString::fromStdString(sendstr));
        totallen = 0;//发送开始前将发送字节数置0
        memset(&sendmsg, 0, sizeof (MsgStruct));
        fseek(send_fp,0,SEEK_END);
        filelen = ftell(send_fp);//读取发送文件的大小
        fseek(send_fp,0,SEEK_SET);//将文件指针设置为文件开始位置
        sendmsg.sendtype = 1;//sendtype=1表示文件传输开始
        sendmsgnum = 1;
        sendmsg.sendnum = sendmsgnum;
        sendmsg.sendlen = file_path.length();
        strcpy(sendmsg.senddata, file_path.c_str());
        SendFunc((char*)&sendmsg);//发送文件存储位置
        if(sendcheck == 0){
            sendstr = "第"+car_number+"辆车发送失败，请检查IP是否正确或网络是否连接";
            emit showmessage(1, QString::fromStdString(sendstr));
            continue;
        }
        while(1){
            memset(&sendmsg, 0, sizeof (MsgStruct));
            fread(sendmsg.senddata,MAXLEN,1,send_fp);//每次读取MAXLEN长度的文件数据
            //通过判断已发送数据totallen大小，判断是发送文件的头包、中间包或者尾包。如果文件大小小于MAXLEN，则将文件一次发送
            if(totallen == 0 && filelen <=  MAXLEN){
                sendmsg.sendtype = 2;//sendtype为2，代表发送文件的头包
                sendmsg.sendlen = filelen;
                sendmsgnum += 1;
                sendmsg.sendnum = sendmsgnum;
                sendmsg.CRCcheck = getCheckSum(sendmsg.senddata, sendmsg.sendlen);
                SendFunc((char*)&sendmsg);
                totallen = filelen;
                if(sendcheck == 0){
                    sendstr = "与第"+car_number+"辆车通信断开，传输失败";
                    emit showmessage(1, QString::fromStdString(sendstr));
                    break;
                }
                else if (sendcheck == 3) {
                    sendstr = "文件路径错误，传输失败，请检查";
                    emit showmessage(3, QString::fromStdString(sendstr));
                    break;
                }
                else{
                    sendstr = "文件传输中，进度：100%";
                    emit showmessage(0, QString::fromStdString(sendstr));
                }
            }
            //如果文件大小大于MAXLEN，则分开发送
            else if (totallen == 0 && filelen > MAXLEN) {
                sendmsg.sendtype = 2;
                sendmsg.sendlen = MAXLEN;
                sendmsgnum += 1;
                sendmsg.sendnum = sendmsgnum;
                sendmsg.CRCcheck = getCheckSum(sendmsg.senddata, sendmsg.sendlen);
                SendFunc((char*)&sendmsg);
                totallen += MAXLEN;//每次发送，totallen就加上发送的数据大小
                if(sendcheck == 0){
                    sendstr = "与第"+car_number+"辆车通信断开，传输失败";
                    emit showmessage(3, QString::fromStdString(sendstr));
                    break;
                }
                else if (sendcheck == 3) {
                    sendstr = "文件路径错误，传输失败，请检查";
                    emit showmessage(3, QString::fromStdString(sendstr));
                    break;
                }
                else{
                    sendstr = "文件传输中，进度："+std::to_string(totallen*100/filelen)+"%";
                    emit showmessage(0, QString::fromStdString(sendstr));
                }
            }
            else if (totallen>0 && totallen+MAXLEN<filelen) {
                sendmsg.sendtype = 3;//sendtype为3，代表发送文件的中间包
                sendmsg.sendlen = MAXLEN;
                sendmsgnum += 1;
                sendmsg.sendnum = sendmsgnum;
                if(sendmsgnum > 10000) sendmsgnum = 1;
                sendmsg.CRCcheck = getCheckSum(sendmsg.senddata, sendmsg.sendlen);
                SendFunc((char*)&sendmsg);
                totallen += MAXLEN;
                if(sendcheck == 0){
                    sendstr = "与第"+car_number+"辆车通信断开，传输失败";
                    emit showmessage(1, QString::fromStdString(sendstr));
                    break;
                }
                else{
                    sendstr = "文件传输中，进度："+std::to_string(totallen/(filelen/100))+"%";
                    emit showmessage(4, QString::fromStdString(sendstr));
                }
            }
            else{
                sendmsg.sendtype = 4;//sendtype为4，代表发送文件的尾包
                sendmsg.sendlen = filelen - totallen;
                sendmsgnum += 1;
                sendmsg.sendnum = sendmsgnum;
                if(sendmsgnum > 10000) sendmsgnum = 1;
                sendmsg.CRCcheck = getCheckSum(sendmsg.senddata, sendmsg.sendlen);
                SendFunc((char*)&sendmsg);
                totallen = filelen;//文件所有数据发送完成
                if(sendcheck == 0){
                    sendstr = "与第"+car_number+"辆车通信断开，传输失败";
                    emit showmessage(1, QString::fromStdString(sendstr));
                    break;
                }
                else{
                    sendstr = "文件传输中，进度：100%";
                    emit showmessage(4, QString::fromStdString(sendstr));
                }
            }
            //如果文件所有数据发送完成，发送结束信号
            if(totallen == filelen){
                memset(&sendmsg, 0, sizeof (MsgStruct));
                sendmsg.sendtype = 5;//sendtype=5表示文件传输结束
                sendmsg.sendlen = filelen;
                sendmsgnum += 1;
                sendmsg.sendnum = sendmsgnum;
                if(sendmsgnum > 10000) sendmsgnum = 1;
                SendFunc((char*)&sendmsg);
                if(sendcheck == 0){
                    sendstr = "第"+car_number+"辆车传输失败";
                    emit showmessage(1, QString::fromStdString(sendstr));
                    break;
                }
                sendstr = "第"+car_number+"辆车文件传输完成,传输文件大小："+std::to_string(totallen)+"bytes";
                emit showmessage(0, QString::fromStdString(sendstr));
                break;
            }
        }
    }
    fclose(send_fp);//关闭打开的文件指针
    ui->pushButton->setEnabled(true);
    ui->pushButton_2->setEnabled(true);
}

void UpdateFile::SendFunc(char* senddata)
{
    sendcheck = 0;//是否发送成功的标志位
    int send_num = 0;//发送时间计数
    //循环300次，时间一共为3s，如果3s期间未收到回应，则认为网络连接断开，停止发送
    while(send_num < 300){
        //发送开始时发送一次数据
        if(send_num == 0){
            int len = sendto(local_sock, senddata, sizeof(MsgStruct), 0,(struct sockaddr *)&to_sin,recvlen);
            if(len < 0)
            {
                std::string sendstr = "发送失败，请检查IP是否正确或网络是否连接";
                emit showmessage(2, QString::fromStdString(sendstr));
                break;
            }
        }
        //每500毫秒重新发送一次数据
        else if (send_num % 50 == 0) {
            printf("wait 500ms, send again\n");
            int len = sendto(local_sock, senddata, sizeof(MsgStruct), 0,(struct sockaddr *)&to_sin,recvlen);
            if(len < 0)
            {
                std::string sendstr = "发送失败，请检查IP是否正确或网络是否连接";
                emit showmessage(2, QString::fromStdString(sendstr));
                break;
            }
        }
        usleep(10);//等待10ms
        send_num += 1;
        //sendcheck置1表示发送成功，sendcheck置3表示文件路径错误，退出发送循环
        if(sendcheck == 1 || sendcheck == 3){
            break;
        }
        //sendcheck置2表示接收端文件校验错误，重新发送
        else if (sendcheck == 2) {
            send_num = 0;
            continue;
        }
    }
}

void UpdateFile::SendRecvFunc(char* senddata)
{
    int len = sendto(local_sock, senddata, sizeof(MsgStruct), 0,(struct sockaddr *)&to_sin,recvlen);
    if(len < 0)
    {
        std::string sendstr = "发送失败，请检查IP是否正确或网络是否连接";
        emit showmessage(2, QString::fromStdString(sendstr));
    }
}

void UpdateFile::thRecvFunc()
{
    while(is_stop)
        {
        char recv_buf[sizeof(MsgStruct)];
        memset(&recv_buf, 0, sizeof(recv_buf));
        MsgStruct recvmsg1;
        //监听端口接收信息
        int len=recvfrom(local_sock,recv_buf,sizeof(MsgStruct),0,(struct sockaddr*)&local_sin, (socklen_t*)&recvlen);
        memcpy(&recvmsg1, &recv_buf, sizeof(MsgStruct));
        HandleRecvMsg(recvmsg1);//处理接收到的信息
    }
}

void UpdateFile::HandleRecvMsg(MsgStruct recvdata)
{
    MsgStruct recvmsg;
    memset(&recvmsg, 0, sizeof(recvmsg));
    //接收到开始发送指令，如果此时recv_totallen不等于0，则上一次文件传输失败，将recv_totallen重新置0并关闭文件指针
    if(recvdata.sendtype == 1){
        if(recvmsgnum == recvdata.sendnum) return;
        recvmsgnum = recvdata.sendnum;
        if(recv_totallen != 0){
            recv_totallen = 0;
            fclose(recv_fp);
        }
        std::string recvstr = recvdata.senddata;
        //处理文件存储路径，将程序工作路径与接收到的UGV文件夹下的路径合并
        save_path = workspace_path.toStdString();
        int index2 = save_path.rfind("UGV");
        if(index2 != -1){
            save_path = save_path.substr(0,index2+3);
            save_path = save_path  + "/" + recvstr;
        }
        else{
            save_path = save_path + "/UGV/" + recvstr;
        }
        recvmsg.sendtype = 99;
        const char* respondstr = "ReceiveSuccess";
        memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
        SendRecvFunc((char*)&recvmsg);
        return;
    }
    //接收到文件的头包
    else if (recvdata.sendtype == 2) {
        if(recvmsgnum == recvdata.sendnum) return;
        recvmsgnum = recvdata.sendnum;
        printf("save_path is %s\n", save_path.c_str());
        recv_fp = fopen(save_path.c_str(),"w");//打开文件指针，设置为可写入模式
        if(recv_fp == NULL){
            printf("myrecv.txt open failure!!\n");
            //文件打开失败，发送回接收端，停止发送
            recvmsg.sendtype = 99;
            const char* respondstr = "filepatherror";
            memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
            SendRecvFunc((char*)&recvmsg);
            return;
        }
        else{
            recv_totallen += recvdata.sendlen;//计算接收到的数据大小
            unsigned char recvcheck = getCheckSum(recvdata.senddata, recvdata.sendlen);//接收到的数据进行校验
            if(recvcheck == recvdata.CRCcheck){
                fwrite(recvdata.senddata,recvdata.sendlen,1,recv_fp);//接收到的数据写入文件
                recvmsg.sendtype = 99;
                const char* respondstr = "ReceiveSuccess";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
            else{
                //接收到的数据校验失败则发送Receivefail，让接收端重新发送
                recvmsg.sendtype = 99;
                const char* respondstr = "Receivefail";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
        }
    }
    else if (recvdata.sendtype ==3) {
        if(recvmsgnum == recvdata.sendnum) return;
        recvmsgnum = recvdata.sendnum;
        if(recv_fp == NULL){
            printf("file open failure!!\n");
        }
        else{
            recv_totallen += recvdata.sendlen;
            unsigned char recvcheck = getCheckSum(recvdata.senddata, recvdata.sendlen);
            if(recvcheck == recvdata.CRCcheck){
                fwrite(recvdata.senddata,recvdata.sendlen,1,recv_fp);
                recvmsg.sendtype = 99;
                const char* respondstr = "ReceiveSuccess";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
            else{
                recvmsg.sendtype = 99;
                const char* respondstr = "Receivefail";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
        }
    }
    else if (recvdata.sendtype == 4) {
        if(recvmsgnum == recvdata.sendnum) return;
        recvmsgnum = recvdata.sendnum;
        if(recv_fp == NULL){
            printf("file open failure!!\n");
        }
        else{
            recv_totallen += recvdata.sendlen;
            unsigned char recvcheck = getCheckSum(recvdata.senddata, recvdata.sendlen);
            if(recvcheck == recvdata.CRCcheck){
                fwrite(recvdata.senddata,recvdata.sendlen,1,recv_fp);
                recvmsg.sendtype = 99;
                const char* respondstr = "ReceiveSuccess";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
            else{
                recvmsg.sendtype = 99;
                const char* respondstr = "Receivefail";
                memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
                SendRecvFunc((char*)&recvmsg);
                return;
            }
        }
    }
    else if (recvdata.sendtype == 5) {
        if(recvmsgnum == recvdata.sendnum) return;
        recvmsgnum = recvdata.sendnum;
        //接收到结束信号，如果接收数据总数与发送数据总数一致，则认为接收成功
        if(recvdata.sendlen == recv_totallen){
            fclose(recv_fp);//关闭文件指针，将传输的文件写入硬盘
            printf("receive sussess, receive total %d byte\n", recv_totallen);
            recv_totallen = 0;//接收文件总数重新置0
            recvmsg.sendtype = 99;
            const char* respondstr = "ReceiveSuccess";
            memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
            SendRecvFunc((char*)&recvmsg);
            //如果是压缩包则进行解压
            if((save_path.find(".tar.xz") != save_path.npos)){
                int index = save_path.rfind('/');
                std::string command = save_path.substr(0,index+1);
                command = "tar -xJf "+save_path+" -C "+command+" --overwrite-dir";
                printf("%s\n", command.c_str());
                int ret = system(command.c_str());
                if(ret == 0){
                    command = "rm "+save_path;
                    system(command.c_str());
                }
            }
            else if((save_path.find(".tar.gz") != save_path.npos) || (save_path.find(".tar") != save_path.npos)){
                int index = save_path.rfind('/');
                std::string command = save_path.substr(0,index+1);
                command = "tar -xzf "+save_path+" -C "+command+" --overwrite-dir";
                printf("%s\n", command.c_str());
                int ret = system(command.c_str());
                if(ret == 0){
                    command = "rm "+save_path;
                    system(command.c_str());
                }
            }
            else if(save_path.find(".zip") != save_path.npos){
                int index = save_path.rfind('/');
                std::string command = save_path.substr(0,index+1);
                command = "unzip -o "+save_path+" -d "+command;
                printf("%s\n", command.c_str());
                int ret = system(command.c_str());
                if(ret == 0){
                    command = "rm "+save_path;
                    system(command.c_str());
                }
            }
        }
        else{
            printf("recv failed, ftruncate(recv_fp), fclose\n");
            ftruncate(fileno(recv_fp), 0);
            fclose(recv_fp);
        }
        recvmsgnum = 0;
        return;
    }
    else if (recvdata.sendtype == 99) {
        if(strncmp(recvdata.senddata,"ReceiveSuccess",14)==0){
            sendcheck = 1;//如果收到ReceiveSuccess，将发送标志位置1，跳出发送循环
        }
        else if (strncmp(recvdata.senddata,"Receivefail",11)==0) {
            sendcheck = 2;//如果收到Receivefail，将发送标志位置2，重新发送当前数据
        }
        else if (strncmp(recvdata.senddata,"filepatherror",13)==0) {
            sendcheck = 3;//如果收到filepatherror，将发送标志位置3，说明文件存储路径错误，结束发送
        }
        else if(strncmp(recvdata.senddata,"compilecommand",14)==0){
            recvmsg.sendtype = 99;
            const char* respondstr = "ReceiveSuccess";
            memcpy(&recvmsg.senddata, respondstr, strlen(respondstr));
            SendRecvFunc((char*)&recvmsg);
            std::string work_path = workspace_path.toStdString();
            int index2 = work_path.rfind("UGV");
            if(index2 != -1){
                work_path = work_path.substr(0,index2+3);
                std::string command = "gnome-terminal --window -e 'bash -c \"cd "+work_path+";source compile.sh;exec bash\"\'";
                printf("%s\n", command.c_str());
                system(command.c_str());
            }
            else{
                work_path = work_path + "/UGV";
                std::string command = "gnome-terminal --window -e 'bash -c \"cd "+work_path+";source compile.sh;exec bash\"\'";
                printf("%s\n", command.c_str());
                system(command.c_str());
            }
        }
        return;
    }
}

//数据校验
unsigned char UpdateFile::getCheckSum(char *data, int size)
{
    unsigned char checkVal = 0;
    for(int i = 0; i < size; i++) {
        checkVal ^= data[i];
    }
    return checkVal;
}

//获取发送车辆的ID和IP信息
void UpdateFile::getCarID()
{
    std::string filepath = workspace_path.toStdString();
    int index2 = filepath.rfind("UGV");
    if(index2 != -1){
        filepath = filepath.substr(0,index2+3);
        filepath = filepath + "/src/pad/carID.txt";
    }
    else{
        filepath = filepath + "/UGV/src/pad/carID.txt";
    }
        // 打开配置文件
        //std::cout <<"fffff-------   "<<filepath<<std::endl;
	
		//printf("ffffffff------------------------%s\n", filepath.c_str());
	    // filepath = std::string("/home/nvidia/Desktop/UGV202601261/src/pad/carID.txt");
        std::ifstream config_file(filepath.c_str());
		
        if (!config_file.is_open())
        {
            QMessageBox::warning(NULL, "警告", "车辆IP配置文件无法打开。", QMessageBox::Ok);
        }
        // 逐行读取并解析数据
        std::string line;
        while (getline(config_file, line))
        {
            // 如果是注释或空行，则忽略
            if (line.empty() || line[0] == '#')
                continue;
            carID.push_back(line);
        }
        // 关闭文件
        config_file.close();
}

//QT界面显示
void UpdateFile::handle_showmessage(int types, QString messagestr)
{
    QTextCharFormat fmt;
    switch (types) {
    case 0:
        fmt.setForeground(QBrush("black"));
        ui->plainTextEdit_info->mergeCurrentCharFormat(fmt);
        ui->plainTextEdit_info->appendPlainText(messagestr);
        break;
    case 1:
        fmt.setForeground(QBrush("red"));
        ui->plainTextEdit_info->mergeCurrentCharFormat(fmt);
        ui->plainTextEdit_info->appendPlainText(messagestr);
        QMessageBox::warning(NULL, "警告", "文件更新失败。", QMessageBox::Ok);
        break;
   case 2:
        fmt.setForeground(QBrush("red"));
        ui->plainTextEdit_info->mergeCurrentCharFormat(fmt);
        ui->plainTextEdit_info->appendPlainText(messagestr);
        QMessageBox::warning(NULL, "警告", "请将文件放入UGV文件夹下对应位置。", QMessageBox::Ok);
        break;
    case 3:
         fmt.setForeground(QBrush("red"));
         ui->plainTextEdit_info->mergeCurrentCharFormat(fmt);
         ui->plainTextEdit_info->appendPlainText(messagestr);
         QMessageBox::warning(NULL, "警告", "请将文件放入UGV文件夹下对应位置。", QMessageBox::Ok);
         break;
     case 4:
        QTextCursor cursor = ui->plainTextEdit_info->textCursor();
        cursor.movePosition(QTextCursor::End); // 移动光标到文本末尾
        int lastLinePos = cursor.position();   // 获取最后一行的位置

         //如果不是在文本开始位置，那么就删除最后一行
        if (lastLinePos > 0) {
            cursor.setPosition(lastLinePos); // 设置光标位置到最后一行的开始
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor); // 选中最后一行
            cursor.removeSelectedText(); // 删除最后一行
        }
        fmt.setForeground(QBrush("black"));
        ui->plainTextEdit_info->mergeCurrentCharFormat(fmt);
        ui->plainTextEdit_info->insertPlainText(messagestr);
//        ui->plainTextEdit_info->appendPlainText(messagestr);
        break;
     }
}
