#include "pad/llh2enu.h"
#include <cmath>
WGS84CorST LLH2WGSXYZ(LongLatHeightST stLLHPos)/*经纬高转WGS84坐标系xyz*/
{
    WGS84CorST stWGS84PosOut;/*返回的WGS84坐标系下的位置点*/

    double fEarthRMajor = 6378137.0000;/*地球半径最大值*/
    double fEarthRMinor = 6356752.3142;/*地球半径最小值*/
    double e = sqrt(1 - (fEarthRMinor/fEarthRMajor)*(fEarthRMinor/fEarthRMajor));
    double fsinLat = sin(stLLHPos.fLatitude*PI/180);
    double fcosLat = cos(stLLHPos.fLatitude*PI/180);
    double ftan2Lat = tan(stLLHPos.fLatitude*PI/180)*tan(stLLHPos.fLatitude*PI/180);

    double fsinLong = sin(stLLHPos.fLongitude*PI/180);
    double fcosLong = cos(stLLHPos.fLongitude*PI/180);

    double tmp = 1 - e*e;
    double tmpden = sqrt(1 + tmp*ftan2Lat);

    double x = (fEarthRMajor*fcosLong)/tmpden + stLLHPos.fHeight*fcosLong*fcosLat;
    double y = (fEarthRMajor*fsinLong)/tmpden + stLLHPos.fHeight*fsinLong*fcosLat;/*0126此处之前写错了*/

    double tmp2 = sqrt(1 - e*e*fsinLat*fsinLat);

    double z = (fEarthRMajor*tmp*fsinLat)/tmp2 + stLLHPos.fHeight*fsinLat;

    stWGS84PosOut.x = x;
    stWGS84PosOut.y = y;
    stWGS84PosOut.z = z;
    return stWGS84PosOut;
}

ENUCorST XYZ2ENU(WGS84CorST stWGSOrigin, LongLatHeightST stLLHOrigin, WGS84CorST stWGSPos)
{
    ENUCorST stENUPosOut;/*输出的ENU位置*/
    memset(&stENUPosOut,0,sizeof(ENUCorST));

    WGS84CorST sttmpWGSPos = stWGSPos;
    WGS84CorST sttmpWGSOrig = stWGSOrigin;

    double fdifx = sttmpWGSPos.x - sttmpWGSOrig.x;
    double fdify = sttmpWGSPos.y - sttmpWGSOrig.y;
    double fdifz = sttmpWGSPos.z - sttmpWGSOrig.z;

    double fsinLat = sin(stLLHOrigin.fLatitude*PI/180);
    double fcosLat = cos(stLLHOrigin.fLatitude*PI/180);
    double fsinLong = sin(stLLHOrigin.fLongitude*PI/180);
    double fcosLong = cos(stLLHOrigin.fLongitude*PI/180);

    double fVector[3][3] = {{-fsinLong, fcosLong, 0},\
                           {-fsinLat*fcosLong, -fsinLat*fsinLong, fcosLat},\
                           {fcosLat*fcosLong, fcosLat*fsinLong, fsinLat}};

    stENUPosOut.x = fVector[0][0] * fdifx + fVector[0][1] * fdify + fVector[0][2] * fdifz;
    stENUPosOut.y = fVector[1][0] * fdifx + fVector[1][1] * fdify + fVector[1][2] * fdifz;
    stENUPosOut.z = fVector[2][0] * fdifx + fVector[2][1] * fdify + fVector[2][2] * fdifz;
    return stENUPosOut;
}

ENUCorST LLH2ENU(LongLatHeightST stLLHPos, LongLatHeightST stLLHOriginPos)/*当前车辆经纬位置转成本地坐标系下的坐标*/
{
    ENUCorST stENUPosOut;/*返回的ENU位置*/

    WGS84CorST stWGSPos = LLH2WGSXYZ(stLLHPos);/*该点经纬高转WGS-84坐标系*/
    WGS84CorST stWGSOrigin = LLH2WGSXYZ(stLLHOriginPos);/*获取原点WGS-84坐标系*/
    stENUPosOut = XYZ2ENU(stWGSOrigin, stLLHOriginPos, stWGSPos);/*转本地局部坐标系, 以起点为原点起点东北天坐标系*/
    return stENUPosOut;
}

GaoSiCorST LLH2GS(LongLatHeightST stLLH)/*经纬度坐标系转高斯坐标系*/
{
    GaoSiCorST stGaoSi;
    memset(&stGaoSi, 0, sizeof(GaoSiCorST));
    //double Re = 6378135.072;//地球半径最大值
    double Re = 6378137;/*地球半径最大值，对应84坐标系*/
    double ee = 1.0/298.257223563;/*对应84坐标系*/
//    double Re = 6378245;//地球半径最大值，对应54坐标系
//    double ee = 1.0/298.3;//对应54坐标系
    double Lat = 0.0;
    double Long = 0.0;
    double Height = 0.0;
    double c=0.0,e1=0.0,e22=0.0,a=0.0,b=0.0,f=0.0;
    double zone=0.0;
    double L0=0.0,L=0.0;
    double t=0.0,yita=0.0,N=0.0,a0=0.0,a2=0.0,a4=0.0,a6=0.0,a8=0.0,X0=0.0;
    double m0=0.0,m2=0.0,m4=0.0,m6=0.0,m8=0.0;
    Long = stLLH.fLongitude*PI/180;
    Lat = stLLH.fLatitude*PI/180;
    Height = stLLH.fHeight;

    a = Re;/*地球半径最大值*/
    f = ee;/*地球偏心率*/
    b = a - a*f;/*地球半径最小值，6356749.875*/
    e1 = sqrt((a*a-b*b)/(a*a));
    e22 = sqrt((a*a-b*b)/(b*b));
    c = pow(a,2)/b;
    zone = floor(int(stLLH.fLongitude)/6) + 1;/*区号，1~60*/
    L0 = (zone*6-3)*PI/180;/*带宽为6度，每个带的中央经线*/
//    zone = floor((Pos[0]-1.5)/3.0) + 1;//区号，1~120
//    L0 = zone*3*PI/180;//带宽为3度
    L = Long - L0;/*相对于中央经线的相对经线值*/

    t = tan(Lat);
    yita = e22*cos(Lat);
    N = c/sqrt(1+yita*yita);
    m0 = a*(1-e1*e1);/*m0 = pow(b,2)/a*/
    m2 = (3.0/2.0)*e1*e1*m0;
    m4 = (5.0/4.0)*e1*e1*m2;
    m6 = (7.0/6.0)*e1*e1*m4;
    m8 = (9.0/8.0)*e1*e1*m6;

    a0 = m0 + m2/2.0 + 3.0*m4/8.0 + 5.0*m6/16.0 +35.0*m8/128.0;
    a2 = m2/2.0 + m4/2.0 + 15.0*m6/32.0 + 7.0*m8/16.0;
    a4 = m4/8.0 + 3.0*m6/16.0 + 7*m8/32.0;
    a6 = m6/32.0 + m8/16.0;
    a8 = m8/128.0;

    X0 = a0*Lat - a2/2.0*sin(2*Lat) + a4/4.0*sin(4*Lat) - a6/6.0*sin(6*Lat) + a8/8*sin(8*Lat);

    stGaoSi.x = (N*cos(Lat)*L+N*pow(L,3)/6.0*pow(cos(Lat),3)*(1-pow(t,2)+pow(yita,2))\
                +N*pow(L,5)/120.0*pow(cos(Lat),5)*(5-18*pow(t,2)+pow(t,4)+14*pow(yita,2)-58*pow(t,2)*pow(yita,2)))\
                +500000.0;/*东E,UTM中的Y轴，经度值*/
//    stGaoSi.x = (stGaoSi.x-500000)*0.9996+500000.0;//此公式将高斯坐标系近似转换为UTM坐标系
    //stGaoSi.x = stGaoSi.x + 1000000*zone - 38000000;//此处的带号之前的系数，应为UTM每个带的米数，还需查下是多少
    stGaoSi.x = stGaoSi.x + 40075020.0/60.0*zone;

    stGaoSi.y = X0+N*pow(L,2)/2.0*sin(Lat)*cos(Lat)\
               +N*pow(L,4)/24.0*sin(Lat)*pow(cos(Lat),3)*(5.0-t*t+9.0*pow(yita,2)+4*pow(yita,4))\
               +N*pow(L,6)/720.0*sin(Lat)*pow(cos(Lat),5)*(61-58*pow(t,2)+pow(t,4));/*北N，UTM中的X轴，纬度值*/
//    stGaoSi.y = stGaoSi.y*0.9996;//此公式将高斯坐标系近似转换为UTM坐标系

    stGaoSi.z = Height;

    qDebug("zone:%f", zone);
    qDebug("E:%f", stGaoSi.x);
    qDebug("N:%f", stGaoSi.y);
    qDebug("H:%f", stGaoSi.z);
    return stGaoSi;
}

ENUCorST LLH2WorldAxis(LongLatHeightST stLLHPos, LongLatHeightST stLLHOriginPos)/*当前车辆经纬位置转成世界坐标系下的坐标*/
{
    ENUCorST stENUPosOut;/*返回的ENU位置*/
    memset(&stENUPosOut,0,sizeof(ENUCorST));
    GaoSiCorST stGSPos = LLH2GS(stLLHPos);/*该点经纬高转高斯坐标系*/
    GaoSiCorST stGSOrigin = LLH2GS(stLLHOriginPos);/*获取原点转高斯坐标系*/
    stENUPosOut.x = stGSPos.x - stGSOrigin.x;/*转本地局部高斯坐标系, 以起点为原点起点东北天坐标系*/
    stENUPosOut.y = stGSPos.y - stGSOrigin.y;
    stENUPosOut.z = stLLHPos.fHeight;
    return stENUPosOut;
}
