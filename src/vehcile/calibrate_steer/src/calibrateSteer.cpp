 
#include "calibrateSteer.h"
#include <unistd.h>



void CalibrateSteer::readDataFileList(std::vector<std::string> &dataFileList)
{

	std::string record_data_dir  = ros::package::getPath("launch_node");
	std::string record_data_store_dir = record_data_dir + '/'+std::string("data_calibrate_steer/");
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


CalibrateSteer::CalibrateSteer(ros::NodeHandle &nh)
:nh_(nh),private_nh_("~")
{	

	private_nh_.param<double>("wheel_base_m", wheel_base, 2.7);

    readDataFileList(dataFileList);
	std::cout <<"data file list :"<<std::endl;
	for (auto & fileName :dataFileList)
	    std::cout <<fileName<<std::endl;
	
	for (auto & fileName :dataFileList)
	{
	    std::vector<Eigen::Vector2d> circlePoints;
	    std::ifstream fileData(fileName);
	    if (!fileData)
	    {
	        std::cout <<"file open error :"<<fileName<<std::endl;
	        return ;
	    }
	    std::string oneLine;
		
		double steer;
		getline(fileData,oneLine);
		std::istringstream percentLine(oneLine);
		percentLine >> steer;
		steerDegree.push_back(steer);
		
	    while(getline(fileData,oneLine))
	    {
	        std::istringstream streamOneLine(oneLine);
	        double x,y;
			Eigen::Vector2d point;
	        streamOneLine >>x;
	        streamOneLine >>y;
			point << x,y;
	        circlePoints.push_back(point);
	    }
		circlesData.push_back(circlePoints);
	    fileData.close();
	}

	for (auto & percent:steerDegree)
    {
		steerRadian.push_back(percent/180.0 * 3.1415926);
	}
    assert(steerRadian.size() == circlesData.size());
	calibrate();
}


Eigen::Vector3d CalibrateSteer::computeCircle(std::vector<Eigen::Vector2d> &points)
{
    
    Eigen::MatrixXd A(points.size(),3);
    Eigen::VectorXd b(points.size(),1);
	for (unsigned int i = 0; i < points.size();i++)
	{
	    Eigen::Vector2d point = points[i];
	    A(i,0) = point(0);A(i,1) = point(1);A(i,2) = -1;
		b(i) = point(0)*point(0)+point(1)*point(1);
	}
	Eigen::MatrixXd AtA = A.transpose() * A;
	Eigen::Vector3d x = AtA.colPivHouseholderQr().solve(A.transpose()*b);
	return x;
}

Eigen::Vector3d CalibrateSteer::computeCubicPloy(std::vector<double> &x, std::vector<double> &y)
{
    assert(x.size()==y.size());
	if (x.size() < 3)
	{
	      double sum = 0;
         for (unsigned int i = 0;i < x.size() ;i++)
         {
             sum += y[i]/x[i];    
         }
		 sum = sum/x.size();
		 Eigen::Vector3d param;
		 param(0) = sum;param(1)=0;param(2)=0;
	}
	
    Eigen::MatrixXd A(x.size(),3);
    Eigen::VectorXd b(x.size(),1);
	for (unsigned int i = 0; i < x.size();i++)
	{
	    A(i,0) = x[i];A(i,1) = x[i]*x[i];A(i,2) = x[i]*x[i]*x[i];
		b(i) = y[i];
	}
	Eigen::MatrixXd AtA = A.transpose() * A;
	Eigen::Vector3d parameter = AtA.colPivHouseholderQr().solve(A.transpose()*b);
	return parameter;
}



void CalibrateSteer::calibrate()
{
     std::vector<double>  circlesRadius;
	 for (auto & circlePoints :circlesData)
	 {
	     Eigen::Vector3d circleParam;
		 double circleRadius;
	     circleParam = computeCircle(circlePoints); 
		 circleRadius = 0.5 * sqrt(circleParam[0]*circleParam[0] + circleParam[1]*circleParam[1]-4*circleParam[2]);
		 circlesRadius.push_back(circleRadius);
	 }

	 std::vector<double> deltas_f;
	 for (unsigned int i = 0;i < steerRadian.size();i++)
	 {
	     double delta_f = std::atan(wheel_base / circlesRadius[i]);
	     if (steerRadian[i] > 0)
	         deltas_f.push_back(delta_f);
		 else
		     deltas_f.push_back(-delta_f);
		 std::cout <<"steer radian"<<steerRadian[i]<<" wheel radian :" <<deltas_f.back()<<std::endl;
	 }

	 
	 Eigen::Vector3d steerToWheel = computeCubicPloy(steerRadian,deltas_f);
	 std::cout <<steerToWheel<<std::endl;

	 Eigen::Vector3d wheelToSteer = computeCubicPloy(deltas_f,steerRadian);
	 std::cout <<wheelToSteer<<std::endl;

	 return ;
}   


int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "calibrateSteer");
	ros::NodeHandle nh;
	ROS_INFO("calibrateSteer start.");
    CalibrateSteer calibrate(nh);
	ROS_INFO(" calibrateSteer end.");
	
	return 0;
}

