#include <string>
#include "platoon_common.h"


bool   platoonCheckNum(const std::vector<unsigned char> &vehicle_num_list,int num)
{
    int index = -1;
	for (unsigned iter_num = 0; iter_num < vehicle_num_list.size(); ++iter_num){
	
		if (vehicle_num_list[iter_num] == num)
			return true;
	}

	return false;
}


int   platoonGetNumIndex(const std::vector<unsigned char> &vehicle_num_list,int num)
{
    int index = -1;
	for (unsigned iter_num = 0; iter_num < vehicle_num_list.size(); ++iter_num){
	
		if (vehicle_num_list[iter_num] == num)
			index = iter_num;
	}

	return index;
}



bool  platoonEraseNum( std::vector<unsigned char> &vehicle_num_list,int num)
{
	for (auto iter = vehicle_num_list.begin(); iter != vehicle_num_list.end(); ++iter) {
		if (*iter == num){
			vehicle_num_list.erase(iter);
			return true;
		}
	}
	std::cout <<"num is not found in the vehicle_num_list"<<std::endl;
	return false;
}


double  platoonGetLoffset(const std::vector<unsigned char> &vehicle_num_list,int self_num,const double &config_l_offset)
{
    int  selfIndex = platoonGetNumIndex(vehicle_num_list,self_num);

	double l_offset = 0;

	switch(selfIndex)
	{
		case 0:l_offset = 0.0;break;
		case 1:l_offset = config_l_offset;break;
		case 2:l_offset = -config_l_offset;break;
		case 3:l_offset = 0.0;break;
	}
	
	return l_offset;
	
}


int  platoonFronterIndex(PlatoonType type,int selfIndex)
{
    assert(selfIndex >= 1);
    if (type != TRIANGLE)
		return selfIndex-1;
	
    int fronterIndex = -1;
	switch (selfIndex) {
	case 1:
	case 2:
		fronterIndex = 0;
		break;
	case 3:
		fronterIndex = 2;
		break;
	default:
		 std::cout <<"warn : wrong selfIndex ---"<< selfIndex<<std::endl;
		 break;
	}
	return fronterIndex;
}


