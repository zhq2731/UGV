#ifndef LLH2ENU_H
#define LLH2ENU_H

#include "pad/allheader.h"

extern ENUCorST LLH2WorldAxis(LongLatHeightST stLLHPos, LongLatHeightST stLLHOriginPos);/*当前车辆经纬位置转成世界坐标系下的坐标*/
extern GaoSiCorST LLH2GS(LongLatHeightST stLLH);
extern ENUCorST LLH2ENU(LongLatHeightST stLLHPos, LongLatHeightST stLLHOriginPos);/*当前车辆经纬位置转成本地坐标系下的坐标*/

#endif // LLH2ENU_H
