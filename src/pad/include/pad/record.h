#ifndef RECORD_H
#define RECORD_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QDateTime>
#include <QList>
#include <QSemaphore>
#include <QMutex>

#include "pad/allheader.h"

class Record : public QThread
{
    Q_OBJECT
public:
    explicit Record(QObject *parent = Q_NULLPTR);
    ~Record();

    void run();/*虚函数run*/
    bool createFile(QString filePath, QString fileName);/*创建记录文件*/
    void Init();

    QString m_strAssignFileName;/*指定文件名*/
    bool m_bQuit;/*程序停止运行标志*/
    UINT32 m_uiRecordFileNo;/*本次记录文件序号*/
    QString m_strStartTime;/*程序启动时间，文件名中使用*/
    QFile m_objRecorFile;/*记录文件*/
    QList<QString> m_msg;/*记录队列*/
    QSemaphore m_synSem;/*同步信号量，同步队列写入和文本写入线程*/
    QMutex m_mutex;/*互斥信号量，互斥写队列线程和读队列线程*/

public slots:
    void write(const QString &strInfo, UINT8 ucFlag = RECORD_UNKNOWN);/*写记录*/
};

#endif // RECORD_H
