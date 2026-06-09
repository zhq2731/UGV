用途：修改osm图的id,将node  way  relation的负id改成正的。

依赖库：pugixml

使用方法：

编译：
cd shiftId
mkdir build
cd build
cmake ..
make

运行：

./shiftId 文件路径。

如：./shiftId  /home/lb/123.osm 执行过后的文件存为： /home/lb/123.osm.shiftId.osm 
