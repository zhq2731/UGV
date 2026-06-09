#include "pad/mapfilegeneration.h"
#include "ui_mapfilegeneration.h"
#include <QFileDialog>
#include <QtDebug>
#include <QMessageBox>
#include "pad/alltypes.h"

MapFileGeneration::MapFileGeneration(QWidget *parent) :
QDialog(parent),
ui(new Ui::MapFileGeneration)
{
ui->setupUi(this);

 m_fLat = 0;
 m_fHeight = 0;
 m_fLon = 0;
 m_fLatInit = 0;
 m_fLonInit =0;
 m_fHeightInit=0;

 QString strText = QString("%1").arg(m_fLon, 0, 'f', 7);
 ui->leLon->setText(strText);
 strText = QString("%1").arg(m_fLat, 0, 'f', 7);
 ui->leLat->setText(strText);
 strText = QString("%1").arg(m_fHeight, 0, 'f', 3);
 ui->leHeight->setText(strText);
 ui->leMaxX->setText(QString("3000"));
 ui->leMinX->setText(QString("-3000"));
 ui->leMaxY->setText(QString("3000"));
 ui->leMinY->setText(QString("-3000"));
 ui->leImageWidth->setText(QString("12000"));
 ui->leImageHeight->setText(QString("12000"));
 ui->leResolution->setText(QString("0.5"));

 ui->btGetMapOrigin->setEnabled(false);
 setUIState(true);

 QPalette p = this->palette();
 p.setColor(QPalette::Window, QColor(50,50,50));
 this->setPalette(p);/*设置窗口背景颜色*/

}

MapFileGeneration::~MapFileGeneration()
{
delete ui;
qDebug() << "~MapFileGeneration()";
}

void MapFileGeneration::setMapOrigin(const double fLat, const double fLon, const float fHeight)
{

    m_fLatInit = fLat;
    m_fHeightInit = fHeight;
    m_fLonInit = fLon;
}

void MapFileGeneration::on_btGetMapOrigin_clicked()/*获取地图原点*/
{
    QString strText = QString("%1").arg(m_fLonInit, 0, 'f', 7);
    ui->leLon->setText(strText);

    strText = QString("%1").arg(m_fLatInit, 0, 'f', 7);
    ui->leLat->setText(strText);

    strText = QString("%1").arg(m_fHeightInit, 0, 'f', 3);
    ui->leHeight->setText(strText);
    ui->btGetMapOrigin->setEnabled(false);
}

void MapFileGeneration::on_btSaveMapFile_clicked()/*保存地图文件*/
{
    QString fileName = QFileDialog::getSaveFileName(this,tr("保存文件"),"D:/CF/2 MapFile/",tr("地图文件(*.txt)"));
    QFile file(fileName);

    if(!file.open(QIODevice::WriteOnly|QIODevice::Text))
    {
        QMessageBox::critical(this,"critical",tr("地图保存失败"),QMessageBox::Yes,QMessageBox::Yes);
        return ;
    }
    else
    {
        //在这个界面里头修改参数，在这里写入保存后的文件参数
        QTextStream stream(&file);
        stream.setCodec("utf-8");

        stream << tr("原点 经度 ")<<ui->leLon->text()<<tr(" 纬度 ")<<ui->leLat->text()
               <<tr(" 高度 ")<< ui->leHeight->text() <<"\n";

        stream << tr("坐标轴X ")<<ui->leMinX->text() <<" "<<ui->leMaxX->text()<<"\n" ;
        stream << tr("坐标轴Y ")<<ui->leMinY->text() <<" "<<ui->leMaxY->text()<<"\n" ;
        stream << tr("resolution:")<<ui->leResolution->text()<<"\n" ;
        stream << tr("width:")<<ui->leImageWidth->text()<<"\n" ;
        stream << tr("height:")<<ui->leImageHeight->text() <<"\n" ;

        stream.flush();
        file.close();
     }

     setUIState(true);

}

void MapFileGeneration::on_pushButton_3_clicked() /*打开地图文件*/
{
    QString fileName = QFileDialog::getOpenFileName(this,tr("打开文件"),"",tr("地图文件(*.txt)"));
    QFile objMapFile(fileName);

    if(!objMapFile.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QMessageBox::critical(this,"critical",tr("打开地图文件失败"),QMessageBox::Yes,QMessageBox::Yes);
        return;
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


                    QString strLongOrigin = str.mid(iPos[1]+1, iPos[2]-iPos[1]-1);/*取经度值*/
                    m_fLon = strLongOrigin.toDouble();

                    QString strLatOrigin = str.mid(iPos[3]+1, iPos[4]-iPos[3]-1);/*取纬度值*/
                    m_fLat = strLatOrigin.toDouble();

                    QString strHeightOrigin = str.right(str.length()-iPos[5]-1);/*取高度值*/
                    strHeightOrigin = strHeightOrigin.left(strHeightOrigin.length()-1);/*去掉换行符*/
                    m_fHeight = strHeightOrigin.toFloat();
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
                    m_fMinX = strX.toFloat();/*栅格地图坐标系X即栅格宽上的最小值，本地东北天坐标系下，单位m*/

                    QString strY = str.right(str.length()-iPos[1]-1);/*取Y值*/
                    strY = strY.left(strY.length()-1);/*删除换行符*/
                    m_fMaxX = strY.toFloat();/*栅格地图坐标系X即栅格宽上的最大值，本地东北天坐标系下，单位m*/
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
                    m_fMinY = strX.toFloat();/*栅格地图坐标系Y即栅格宽上的最小值，本地东北天坐标系下，单位m*/

                    QString strY = str.right(str.length()-iPos[1]-1);/*取Y值*/
                    strY = strY.left(strY.length()-1);/*删除换行符*/
                    m_fMaxY = strY.toFloat();/*栅格地图坐标系Y即栅格宽上的最大值，本地东北天坐标系下，单位m*/
                }
                else if(true == str.contains("resolution:"))
                {
                    QString strResolution = str.right(str.length()-11);
                    m_fResolution = strResolution.toFloat();
                }
                else if(true == str.contains("width:"))
                {
                    QString strWidth = str.right(str.length()-6);
                    m_fImageWidth = strWidth.toInt();
                }
                else if(true == str.contains("height:"))
                {
                    QString strHeight = str.right(str.length()-7);
                    m_fImageHeight = strHeight.toInt();
                }
                else
                {
                    //不用处理
                }
            }
        }
        objMapFile.close();/*先关闭打开的文件*/

        QString strText = QString("%1").arg(m_fLon, 0, 'f', 7);
        ui->leLon->setText(strText);

        strText = QString("%1").arg(m_fLat, 0, 'f', 7);
        ui->leLat->setText(strText);

        strText = QString("%1").arg(m_fHeight, 0, 'f', 3);
        ui->leHeight->setText(strText);
        ui->leMaxX->setText(QString::number(m_fMaxX));
        ui->leMinX->setText(QString::number(m_fMinX));
        ui->leMaxY->setText(QString::number(m_fMaxY));
        ui->leMinY->setText(QString::number(m_fMinY));
        ui->leImageWidth->setText(QString::number(m_fImageWidth));
        ui->leImageHeight->setText(QString::number(m_fImageHeight));
        ui->leResolution->setText(QString::number(m_fResolution));

        setUIState(true);
    }
}

void MapFileGeneration::on_btModifyMapFile_clicked() /*地图文件修改*/
{
    setUIState(false);
    ui->btGetMapOrigin->setEnabled(true);


}

void MapFileGeneration::setUIState(bool bFlag)
{
    ui->leLon->setReadOnly(bFlag);
    ui->leLat->setReadOnly(bFlag);
    ui->leHeight->setReadOnly(bFlag);
    ui->leMaxX->setReadOnly(bFlag);
    ui->leMinX->setReadOnly(bFlag);
    ui->leMaxY->setReadOnly(bFlag);
    ui->leMinY->setReadOnly(bFlag);
    ui->leImageWidth->setReadOnly(bFlag);
    ui->leImageHeight->setReadOnly(bFlag);
    ui->leResolution->setReadOnly(bFlag);

}
