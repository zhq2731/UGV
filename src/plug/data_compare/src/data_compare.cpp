
#include "data_compare.h"
#include <unistd.h>


DataCompare::DataCompare(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{	
	geometry_msgs::Point origin;
    private_nh_.param<double>("latitude", origin.x , 0.0);
	private_nh_.param<double>("longitude", origin.y , 0.0);
	private_nh_.param<double>("altitude", origin.z , 0.0);

    projector = projection::UtmProjector(origin);
	
	pub_diplay = nh_.advertise<visualization_msgs::MarkerArray>(
		"/data_compare", 1);

	std::vector<std::string> dataFileList;
    readDataFileList(dataFileList);
    for (auto & filename:dataFileList){
		std::cout <<"filename: "<<filename<<std::endl;
		planning_msgs::TrajectoryPointArray trajectory;
	    readGpsTxts(filename,trajectory);
	    trajectorys.push_back({filename,trajectory});
    }
	std::cout <<"read data from file finished "<<std::endl;
	timer = nh_.createTimer(ros::Duration(5.0),&DataCompare::callbackTimerReference,this);
}


void DataCompare::setColor(std_msgs::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void DataCompare::callbackTimerReference(const ros::TimerEvent &event)
{
    std::cout <<"start"<<std::endl;
	std::vector<std_msgs::ColorRGBA>	colors;
	colors.resize(5);
	setColor(&colors[0],1.0,1.0,1.0,1.0);
	setColor(&colors[1],1.0,0.85,0.5,1.0);
	setColor(&colors[2],1.0,0.0,0.0,1.0);
	setColor(&colors[3],0.0,1.0,0.0,1.0);
	setColor(&colors[4],0.0,0.0,1.0,1.0);

	visualization_msgs::MarkerArray markerArray;
	
	int num = 0;
    for (auto & trajectory:trajectorys){
		
		int index  = num%5;
		float r = colors[index].r;
		float g = colors[index].g;
		float b = colors[index].b;
		num++;
	   // markerArray.markers.push_back(DisPlay::generateLineMarker(1,trajectory.second.points,r,g,b,0.5,trajectory.first,false));
    }
	
    std::cout <<"end"<<std::endl;
	pub_diplay.publish(markerArray);
}

void DataCompare::readDataFileList(std::vector<std::string> &dataFileList)
{

	std::string record_data_dir  = ros::package::getPath("launch_node");
	std::string record_data_store_dir = record_data_dir + '/'+std::string("data_compare/");
	struct dirent **namelist;
	int n;
	n = scandir(record_data_store_dir.c_str(),&namelist,0,alphasort);
	if(n < 0)
	{ 
		std::cout << "scandir return false "<< n  << std::endl;
	}
	else
	{
		int index=0;
		std::string pureName;
		while(index < n)
		{
			if (namelist[index]->d_type == DT_REG)
			{
				 std::string filePath = record_data_store_dir +	namelist[index]->d_name;
				 dataFileList.push_back(filePath);
			}
			free(namelist[index]);
			index++;
		}
		free(namelist);
	}
}


bool DataCompare::readGpsTxts(std::string &fileName, planning_msgs::TrajectoryPointArray &trajectoryTotal)
{

    std::ifstream filename(fileName);
    if (!filename)
    {
        std::cout <<"ReferenceNode file open error: "<<fileName <<std::endl;
        return false;
    }
    std::string oneLine;
	getline(filename,oneLine);
	getline(filename,oneLine);
	int first = 0;
    while(getline(filename,oneLine))
    {
        planning_msgs::TrajectoryPoint  trajectPoint; 
		
        geometry_msgs::Point point_gps,point_utm;
        std::istringstream streamOneLine(oneLine);
        std::string ignore;
		streamOneLine >>ignore;
		streamOneLine >>ignore;
		streamOneLine >>ignore;
		streamOneLine >>ignore;
		streamOneLine >>ignore;
		if (fileName == "/home/lb/UGV/src/launch_node/data_compare/1312-0-03L001-230830.PosT" 
			|| fileName == "/home/lb/UGV/src/launch_node/data_compare/1.txt"){
           streamOneLine >>point_gps.x;
           streamOneLine >>point_gps.y;
		}
		else
		{
		    streamOneLine >>point_gps.y;
		    streamOneLine >>point_gps.x;
		}
		point_utm = projector.forward(point_gps);
		trajectPoint.x = point_utm.x;
		trajectPoint.y = point_utm.y;
        trajectoryTotal.points.push_back(trajectPoint);
    }
		
    filename.close();
    return true;
}

int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "dataCompare");
	ros::NodeHandle nh;
	ROS_INFO("dataCompare start.");

	DataCompare dataCompareNode(nh);
	ros::spin();
	ROS_INFO(" ReferenceNode The iteration end.");
	
	return 0;
}



