#include "pad/record.h"

Record::Record(QObject *parent):
    QThread(parent),
    m_synSem(0)
{
    m_strAssignFileName.clear();/*指定文件名*/
    m_bQuit = false;/*程序停止运行标志*/
    m_uiRecordFileNo = 1;/*本次记录文件序号*/
    m_msg.clear();/*清空队列*/
}

Record::~Record()
{
    m_synSem.release();/*假如当前没有记录产生，那么手动释放一次信号量，使记录线程可以停止*/
    m_bQuit = true;/*停止记录run线程*/
    msleep(100);/*等待记录线程停止*/
    if(true == m_objRecorFile.isOpen())
    {
        m_objRecorFile.close();/*关闭记录文件*/
    }

    qDebug()<<"~Record()";
}

void Record::Init()
{
    QDateTime dateTime = QDateTime::currentDateTime();
    m_strStartTime = dateTime.toString("yyyyMMdd hhmmss");
    QString strfileName;
    if(true == m_strAssignFileName.isEmpty())
    {
        QString strNo = QString("-%1").arg(m_uiRecordFileNo);/*10MB一个文件，本次启动的第几次记录*/
        strfileName = m_strStartTime + strNo + ".txt";
    }
    else
    {
        strfileName = m_strStartTime + m_strAssignFileName + ".txt";
    }
    if(false == createFile("D:/Record", strfileName))/*创建记录文件*/
    {
        qDebug()<<"记录文件创建失败";
    }
}

/********************************虚函数********************************/
void Record::run()
{
    while(false == m_bQuit)
    {
        m_synSem.acquire();/*无限等待有记录写入*/

        m_mutex.lock();
        if(true == m_msg.isEmpty())
        {
            m_mutex.unlock();
            continue;
        }
        QString strInfo = m_msg.front();/*取出队首*/
        m_msg.pop_front();/*删除队首*/
        m_mutex.unlock();

        if(true == m_objRecorFile.isOpen())/*确认文件已打开*/
        {
            QTextStream text_stream(&m_objRecorFile);
            text_stream<<strInfo;
            m_objRecorFile.flush();

            if(true == m_strAssignFileName.isEmpty())
            {
                if((10*1024*1024) < m_objRecorFile.size())/*文件大小超过10MB*/
                {
                    m_objRecorFile.close();/*关闭文件*/
                    m_uiRecordFileNo++;/*文件名序号改变*/

                    QString strNo = QString("-%1").arg(m_uiRecordFileNo);/*10MB一个文件，本次启动的第几次记录*/
                    QString strfileName;
                    strfileName = m_strStartTime + strNo + ".txt";
                    if(false == createFile("D:/Record", strfileName))/*创建记录文件*/
                    {
                        qDebug()<<"记录文件创建失败";
                    }
                }
            }
        }
    }
//    quit();/*退出事件循环*/
//    qDebug("记录线程停止");
}

/********************************写文件信息********************************/
void Record::write(const QString &strInfo, UINT8 ucFlag)
{
    QString strData;
    strData.clear();

    QDateTime dateTime = QDateTime::currentDateTime();
    QString strTime = dateTime.toString("yyyyMMdd hh:mm:ss:zzz");

    UINT32 uiTime = dateTime.time().msec() + ((dateTime.time().hour()*60 + dateTime.time().minute())*60 + dateTime.time().second())*1000;
    QString strAbsTime = QString("%1").arg(uiTime);

    switch(ucFlag)
    {
        case RECORD_PROCESS:
        {
            strData = strTime + " " + strAbsTime + " " + "PROCESS" + " ";
            break;
        }
        case RECORD_DATA:
        {
            strData = strTime + " " + strAbsTime + " " + "DATA" + " ";
            break;
        }
        case RECORD_ERROR:
        {
            strData = strTime + " " + strAbsTime + " " + "ERROR" + " ";
            break;
        }
        default:
        {
            strData = strTime + " " + strAbsTime + " " + "UNKNOWN" + " ";
            break;
        }
    }

    strData += strInfo;
    strData += "\n";

    m_mutex.lock();
    m_msg.push_back(strData);
    m_mutex.unlock();
    m_synSem.release();/*通知run线程，可以写入文件了*/
}

/********************************创建记录文件********************************/
bool Record::createFile(QString filePath, QString fileName)
{
    QDir tempDir;
    QString currentDir = QDir::currentPath();/*临时保存程序当前路径*/
    if(false == tempDir.exists(filePath))/*不存在记录文件的存储路径则创建*/
    {
        if(false == tempDir.mkpath(filePath))
        {
            return false;
        }
    }
    if(false == QDir::setCurrent(filePath))/*打开filePath路径*/
    {
        return false;
    }

    if(true == tempDir.exists(fileName))/*查询路径下文件是否存在*/
    {
        qDebug()<<"文件存在";
        return true;
    }
    m_objRecorFile.setFileName(fileName);/*在当前路径下创建文件*/
    if(false == m_objRecorFile.open(QIODevice::ReadWrite | QIODevice::Text))
    {
        qDebug()<<"创建文件失败";
        return false;
    }
    QDir::setCurrent(currentDir);/*将程序当前路径设为原来的路径*/
    return true;
}
