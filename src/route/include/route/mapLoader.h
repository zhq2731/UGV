#pragma once

#include "route/osmFileParser.h"

struct MapInfo{
	std::string map_file;
	double origin_lat;
	double origin_lon;
	double origin_alt;
};

class MapLoader {
 public:
	MapLoader(){}
	File* loadOsmMap(MapInfo &mapInfo);
	//IdPair FindNearestNode(double curX,double curY);
 private:
	File file;
};

