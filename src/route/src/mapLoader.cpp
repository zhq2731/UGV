#include "route/mapLoader.h"


File* MapLoader::loadOsmMap(MapInfo &mapInfo){

	projection::UtmProjector projector;
	geometry_msgs::Point map_origin;
	map_origin.x = mapInfo.origin_lat;
	map_origin.y = mapInfo.origin_lon;
	map_origin.z = mapInfo.origin_alt;
	projector = projection::UtmProjector(map_origin);
	pugi::xml_document doc;
	auto result = doc.load_file(mapInfo.map_file.c_str());
	OsmFileParser OsmFileParser_(projector);
	file =  OsmFileParser_.read(doc); 
	return &file;
}



