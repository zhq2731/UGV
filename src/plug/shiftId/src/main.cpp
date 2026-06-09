
#include "OsmFile.h"


int main(int argc ,char *argv[])
{    
    std::cout <<argv[1]<<std::endl;
    if (2 != argc || std::string(argv[1]) == std::string("-h") || std::string(argv[1]) == std::string("--help") )
    {
        std::cout <<"use: ./shiftId 'path to osm file' "<<std::endl;
		return 0;
    }

	
    std::string fileName = std::string(argv[1]);
	std::cout <<"fileName:: "<<fileName<<std::endl;
	lanelet::osm::File file = lanelet::osm::parse(fileName);
    std::cout <<"size "<<file.nodes.size()<<std::endl;
	for (auto &node:file.nodes){
          if(node.first < 0){
		  	  node.second.id = -node.second.id ;
          }
   	}

	for (auto &way:file.ways){
          if(way.first < 0){
		  	  way.second.id = -way.second.id ;
          }
   	}
	
	for (auto &relation:file.relations){
          if(relation.first < 0){
		  	  relation.second.id = -relation.second.id ;
          }
   	}
    std::string saveName = std::string(argv[1]) + std::string(".shiftid.osm");
	std::cout <<"save name  :   "<<saveName <<std::endl;
	lanelet::osm::write(file)->save_file(saveName.c_str(), "  ");
	
	return 0;
}

