#include "pad/astar.h"

AStar::AStar(QObject *parent) : QObject(parent)
{
    m_iMapColNum = 0;/*地图在列上面的栅格数*/
    m_iMapRowNum = 0;/*地图在行上面的栅格数*/
    m_fResolution = 0;/*分辨率,1个栅格对应多少米*/

    m_fMapENU_XMin = 0;/*栅格地图坐标系X即栅格宽上的最小值，本地东北天坐标系下，单位m*/
    m_fMapENU_XMax = 0;/*栅格地图坐标系X即栅格宽上的最大值，本地东北天坐标系下，单位m*/
    m_fMapENU_YMin = 0;/*栅格地图坐标系Y即栅格宽上的最小值，本地东北天坐标系下，单位m*/
    m_fMapENU_YMax = 0;/*栅格地图坐标系Y即栅格宽上的最大值，本地东北天坐标系下，单位m*/
}

AStar::~AStar()
{
       qDebug()<<"~AStar()";
}

/*******************************读取栅格地图信息文件********************************/
void AStar::ReadMapFile(const QString strFilePath, const QString strFileName)/*点击地图文件打开按钮，才会到这儿*/
{
    QFile objMapFile;/*栅格地图文件*/
    QString strFile = strFilePath+strFileName;/*完整地图文件路径*/
    objMapFile.setFileName(strFile);/*设置文件名称*/
    if(false == objMapFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        emit ReadMapFileRslt(false);/*发送地图读取失败信号*/
    }
    else
    {
        /*读取地图文件中信息*/
        QByteArray array;
        array.clear();
        int m_uiReadRow = 0;/*已读取行数*/

        while(false == objMapFile.atEnd())/*未到文件尾*/
        {
            m_uiReadRow++;/*已读取行数*/
            if(11 > m_uiReadRow)
            {
                array = objMapFile.readLine();/*读取一行*/
                QString str;
                str.prepend(array);
                if(true == str.contains("image:"))
                {
                    /*不用处理*/
                }
                else if(true == str.contains("原点"))
                {
                    INT32 iPos[21] = {0};
                    INT32 iPosTemp, iCnt = 0;
                    /*第二参数是指从当前索引位置开始搜索,第1次从0开始搜索，后面每次都从上次找到的TAB键的下一个位置开始搜索*/
                    while(-1 != (iPosTemp = str.indexOf(" ", ((0==iCnt)?0:iPos[iCnt-1]+1))))/*查找空格键*/
                    {
                        iPos[iCnt] = iPosTemp;
                        iCnt++;/*查找到TAB键的次数*/
                        if(iCnt > 20)/*保证iPos[iCnt]不会越界*/
                        {
                            break;
                        }
                    }

                    LongLatHeightST stLLHOrigin;
                    memset(&stLLHOrigin, 0, sizeof(LongLatHeightST));
                    QString strLongOrigin = str.mid(iPos[1]+1, iPos[2]-iPos[1]-1);/*取经度值*/
                    stLLHOrigin.fLongitude = strLongOrigin.toDouble();

                    QString strLatOrigin = str.mid(iPos[3]+1, iPos[4]-iPos[3]-1);/*取纬度值*/
                    stLLHOrigin.fLatitude = strLatOrigin.toDouble();

                    QString strHeightOrigin = str.right(str.length()-iPos[5]-1);/*取高度值*/
                    strHeightOrigin = strHeightOrigin.left(strHeightOrigin.length()-1);/*去掉换行符*/
                    stLLHOrigin.fHeight = strHeightOrigin.toFloat();

                    emit MapOriginSet(stLLHOrigin);/*发送地图原点设置信号*/
                }
                else if(true == str.contains("坐标轴X"))/*坐标轴X的最小值用于计算当前点对应栅格的列号*/
                {
                    INT32 iPos[21] = {0};
                    INT32 iPosTemp, iCnt = 0;
                    while(-1 != (iPosTemp = str.indexOf(" ", ((0==iCnt)?0:iPos[iCnt-1]+1))))/*查找TAB键*/
                    {
                        iPos[iCnt] = iPosTemp;
                        iCnt++;/*查找到TAB键的次数*/
                        if(iCnt > 20)/*保证iPos[iCnt]不会越界*/
                        {
                            break;
                        }
                    }

                    QString strX = str.mid(iPos[0]+1, iPos[1]-iPos[0]-1);/*取X值*/
                    m_fMapENU_XMin = strX.toFloat();/*栅格地图坐标系X即栅格宽上的最小值，本地东北天坐标系下，单位m*/

                    QString strY = str.right(str.length()-iPos[1]-1);/*取Y值*/
                    strY = strY.left(strY.length()-1);/*删除换行符*/
                    m_fMapENU_XMax = strY.toFloat();/*栅格地图坐标系X即栅格宽上的最大值，本地东北天坐标系下，单位m*/
                }
                else if(true == str.contains("坐标轴Y"))/*坐标轴Y的最大值用于计算当前点对应栅格的行号*/
                {
                    INT32 iPos[21] = {0};
                    INT32 iPosTemp, iCnt = 0;
                    while(-1 != (iPosTemp = str.indexOf(" ", ((0==iCnt)?0:iPos[iCnt-1]+1))))/*查找TAB键*/
                    {
                        iPos[iCnt] = iPosTemp;
                        iCnt++;/*查找到TAB键的次数*/
                        if(iCnt > 20)/*保证iPos[iCnt]不会越界*/
                        {
                            break;
                        }
                    }

                    QString strX = str.mid(iPos[0]+1, iPos[1]-iPos[0]-1);/*取X值*/
                    m_fMapENU_YMin = strX.toFloat();/*栅格地图坐标系Y即栅格宽上的最小值，本地东北天坐标系下，单位m*/

                    QString strY = str.right(str.length()-iPos[1]-1);/*取Y值*/
                    strY = strY.left(strY.length()-1);/*删除换行符*/
                    m_fMapENU_YMax = strY.toFloat();/*栅格地图坐标系Y即栅格宽上的最大值，本地东北天坐标系下，单位m*/
                }
                else if(true == str.contains("resolution:"))
                {
                    QString strResolution = str.right(str.length()-11);
                    m_fResolution = strResolution.toFloat();
                }
                else if(true == str.contains("width:"))
                {
                    QString strWidth = str.right(str.length()-6);
                    m_iMapColNum = strWidth.toInt();
                }
                else if(true == str.contains("height:"))
                {
                    QString strHeight = str.right(str.length()-7);
                    m_iMapRowNum = strHeight.toInt();
                }
                else
                {
                }
            }
        }
        objMapFile.close();/*先关闭打开的文件*/
        emit ReadMapFileRslt(true);/*发送地图读取成功信号*/
    }
}

