#ifndef ASTAR_H
#define ASTAR_H

#include <QObject>

#include <QMessageBox>
#include <math.h>
#include <QTextCodec>
#include "cmath"
#include "iostream"
#include "pad/allheader.h"
#include "pad/alldeclare.h"

class AStar : public QObject
{
    Q_OBJECT
public:
    explicit AStar(QObject *parent = Q_NULLPTR);
    ~AStar();

    INT32 m_iMapColNum;/*地图在列上面的栅格数*/
    INT32 m_iMapRowNum;/*地图在行上面的栅格数*/
    float m_fResolution;/*分辨率,1个栅格对应多少米*/

    float m_fMapENU_XMin;/*栅格地图坐标系X即栅格宽上的最小值，本地东北天坐标系下，单位m*/
    float m_fMapENU_XMax;/*栅格地图坐标系X即栅格宽上的最大值，本地东北天坐标系下，单位m*/
    float m_fMapENU_YMin;/*栅格地图坐标系Y即栅格宽上的最小值，本地东北天坐标系下，单位m*/
    float m_fMapENU_YMax;/*栅格地图坐标系Y即栅格宽上的最大值，本地东北天坐标系下，单位m*/

public slots:
    void ReadMapFile(const QString strFilePath, const QString strFileName);/*读取地图文件*/

signals:
    void ReadMapFileRslt(bool bRslt);/*读取地图文件结果信号*/
    void MapOriginSet(LongLatHeightST stLLHOrigin);/*设定了地图原点信号*/
    void MsgTableDispSign(QString strCol2, QString strCol3, UINT8 ucDispType = DISP_PROCESS);/*界面报文表格显示*/
};

#endif // ASTAR_H
