#include "route/topoGraph.h"


void ANode::Init(){
	node_g = MAX_INT;
	node_h = MAX_INT;
	node_attr = ANODE_ATTR_NULL;
	node_father = nullptr;
}

void NodeInfo::AddAdjacentNode(std::shared_ptr<ANode> &node){
	if (std::find(adjacentNodes.begin(), adjacentNodes.end(), node) == adjacentNodes.end()) {
		adjacentNodes.push_back(node);
	}
}

void NodeInfo::RmvAdjacentNode(std::shared_ptr<ANode> &node){
	for (auto it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it){
    	if (*it == node){
    		adjacentNodes.erase(it);
			break;
		}
	}
}

void NodeInfo::BuildWeight(std::shared_ptr<ANode> &node,std::vector<Node*> nodes){
	double value = 0;
	std::vector<UtmPoint> wayPoints;
	for(auto it:nodes){
		UtmPoint wayPoint = it->point;
		wayPoints.push_back(wayPoint);
	}
	for (auto i = 1 ;i < wayPoints.size();i++){
	    value += amathutils::distance2D(wayPoints[i],wayPoints[i-1]); 
	}
	auto item = nodeValues.find(node);
	if(item != nodeValues.end()){
		if(value < nodeValues[node]){
			nodeValues[node] = value;
		}
	}
	else
		nodeValues.insert(std::make_pair(node,value));
}

void NodeInfo::DeleteWeight(std::shared_ptr<ANode> &node){
 	auto item = nodeValues.find(node);
	if(item != nodeValues.end()){
		nodeValues.erase(item); 
	}
}

std::shared_ptr<ANode> TopoGraph::AddNode(int nodeId,double nodeX,double nodeY){
	auto it = topoGraphNodes.find(nodeId);
	if (it != topoGraphNodes.end()){
   		return it->second.theNode;
	}
	std::shared_ptr<ANode> node = std::make_shared<ANode>(nodeId,nodeX,nodeY);
	topoGraphNodes.emplace(nodeId,NodeInfo(node));
	return node;
}

void TopoGraph::AddEdge(std::shared_ptr<ANode> snode, std::shared_ptr<ANode> enode, 
		std::vector<Node*> nodes, Attributes attributes){
	topoGraphNodes[snode->node_id].AddAdjacentNode(enode);
	topoGraphNodes[snode->node_id].BuildWeight(enode,nodes);
	for(auto attribute:attributes){
		if(attribute.first == keyword::Direction&&
			attribute.second == keyword::Bidir){
			topoGraphNodes[enode->node_id].AddAdjacentNode(snode);
			topoGraphNodes[enode->node_id].BuildWeight(snode,nodes);
		}
	}
}

void TopoGraph::RmvEdge(std::shared_ptr<ANode> snode, std::shared_ptr<ANode> enode){
	topoGraphNodes[snode->node_id].RmvAdjacentNode(enode);
	topoGraphNodes[snode->node_id].DeleteWeight(enode);
	topoGraphNodes[enode->node_id].RmvAdjacentNode(snode);
	topoGraphNodes[enode->node_id].DeleteWeight(snode);
}


std::shared_ptr<ANode> TopoGraph::SetNodeAttr(int nodeId, ANodeAttr attr){
	auto it = topoGraphNodes.find(nodeId);
	if (it == topoGraphNodes.end()) {
	  return nullptr;
	}
	it->second.theNode->node_attr = attr;
	return it->second.theNode;
}

const std::vector<std::shared_ptr<ANode>>& TopoGraph::GetAdjacent(int nodeId){
	return topoGraphNodes[nodeId].adjacentNodes;
}

std::shared_ptr<ANode> TopoGraph::GetNode(int nodeId) const{
	auto it = topoGraphNodes.find(nodeId);
	if (it == topoGraphNodes.end()) {
	   return nullptr;
	}
	return it->second.theNode;
}

inline double TopoGraph::HFuc(std::shared_ptr<ANode> s, std::shared_ptr<ANode> e) const{
	return amathutils::distance2D(s->node_x,s->node_y,e->node_x,e->node_y);
}

void TopoGraph::InitGraph(){
	for (auto item : topoGraphNodes){
    	item.second.theNode->Init();
	}
}

void TopoGraph::ClearGraph(){
	for (auto item : topoGraphNodes) {
    	if (item.second.theNode != nullptr) {
        	item.second.theNode.reset();
        	item.second.theNode = nullptr;
    	}
	}
	topoGraphNodes.clear();
	osm2Topo.clear();
	osmSE2Topo.clear();
}

void TopoGraph::AddSEWay2Result(std::vector<UtmPoint> &singleUtmResult,WaySEId waySEId){
	auto sNode = GetNode(waySEId.sId);
	for(auto it : osmSE2Topo){
		auto trajectory = it.first;
		auto pointEqual = ArePoint2DEqual(waySEId, it.second);
		if (PathBuild(pointEqual,trajectory,singleUtmResult))
			break;
	}
}

void TopoGraph::AddWay2Result(std::vector<UtmPoint> &singleUtmResult,WaySEId waySEId){
	for(auto it : osm2Topo){
		auto pointEqual = ArePoint2DEqual(waySEId, it.second);
		auto trajectory = file_->ways[it.first].nodes;
		if (PathBuild(pointEqual,trajectory,singleUtmResult))
			break;
	}
}


bool TopoGraph::PathBuild(Point2DEqual pointEqual,std::vector<Node*> trajectory,std::vector<UtmPoint> &singleUtmResult){
	if(POINT2D_POSITIVE_EQUAL == pointEqual){
		PointDeal(trajectory,singleUtmResult);
		return true;
	}
	if(POINT2D_NEGATIVE_EQUAL == pointEqual){
		std::reverse(trajectory.begin(), trajectory.end());
		PointDeal(trajectory,singleUtmResult);
		return true;
	}
	return false;
}

void TopoGraph::PointDeal(std::vector<Node*> trajectory,std::vector<UtmPoint> &singleUtmResult){
	for(auto item : trajectory){
		singleUtmResult.push_back(item->point);
	}
	singleUtmResult.pop_back();
}

void TopoGraph::PrintGraph(std::vector<UtmPoint> &singleUtmResult , int e_id) {
	std::shared_ptr<ANode> enode = GetNode(e_id);
	std::shared_ptr<ANode> enode_ = enode;
	if (enode == nullptr)
    	return;
	//std::cout << "single path length = " << enode->node_g + enode->node_h << std::endl;
	while (enode != nullptr) {
    	//std::cout << enode->node_id << " <-- ";
    	enode = enode->node_father;
	}
	//std::cout<<" "<<std::endl;
	std::vector<Id> ids;
	while (enode_ != nullptr) {
		ids.push_back(enode_->node_id);
    	enode_ = enode_->node_father;
	}
	std::reverse(ids.begin(), ids.end());
	
	//std::cout<<"A*找寻的id序列"<<std::endl;
	//std::cout << "ids: ";
	//for (auto id : ids) std::cout << id << "->";
	//std::cout << std::endl;
	
	WaySEId waySEId_;
	if (ids.size() == 0)
		return;
	if (ids.size() == 1){
		auto utmPoint = file_->nodes[ids.back()].point;
		singleUtmResult.push_back(utmPoint);
	 	return;
	}
    waySEId_.sId = ids[0];
    waySEId_.eId = ids[1];
    AddSEWay2Result(singleUtmResult, waySEId_);
    for (size_t i = 0; i < ids.size() - 1; ++i) {
        waySEId_.sId = ids[i];
        waySEId_.eId = ids[i + 1];
        AddWay2Result(singleUtmResult, waySEId_);
    }
	if (ids.size() != 2){
	    waySEId_.sId = ids[ids.size() - 2];
	    waySEId_.eId = ids[ids.size() - 1];
	    AddSEWay2Result(singleUtmResult, waySEId_);
	}
	auto utmPoint = file_->nodes[ids.back()].point;
	singleUtmResult.push_back(utmPoint);
}



ErrCode TopoGraph::FindPath(int s_id, int e_id){
	auto compFunc = [](const std::shared_ptr<ANode> e1, const std::shared_ptr<ANode> e2) { 
		return (e1->node_g + e1->node_h) > (e2->node_g + e2->node_h); };

	std::shared_ptr<ANode> snode = GetNode(s_id);
	std::shared_ptr<ANode> enode = GetNode(e_id);

	if (snode == nullptr || enode == nullptr) {
	    return ERRCODE_NODE_NOT_EXSIT;
	}
	
	if (snode->node_attr == ANODE_ATTR_FORBID || enode->node_attr == ANODE_ATTR_FORBID) {
	    return ERRCODE_NODE_FORBID;
	}
	snode->node_g = 0;
	snode->node_h = HFuc(snode, enode);
	std::vector<std::shared_ptr<ANode>> openList;
	snode->node_attr = ANODE_ATTR_OPEN;
	openList.push_back(snode);
	while (1) {
	    if (openList.empty()) {
	        return ERRCODE_NO_PATH;
	    }
	    std::pop_heap(openList.begin(), openList.end(), compFunc);
	    std::shared_ptr<ANode> curNode = openList.back();
	    curNode->node_attr = ANODE_ATTR_CLOSE;
	    openList.pop_back();
	    if (curNode->node_id == enode->node_id) {
	        break;
	    }
	    auto &vt = GetAdjacent(curNode->node_id);
	    for (auto item : vt) {
	        if (item->node_attr == ANODE_ATTR_OPEN) {
	            double oldF = item->node_g + item->node_h;
	            double newG = curNode->node_g + topoGraphNodes[curNode->node_id].nodeValues[item];
	            double newH = HFuc(item, enode);
	            double newF = newG + newH;
	            if (newF < oldF) {
	                item->node_g = newG;
	                item->node_h = newH;
	                item->node_father = curNode;
	                std::make_heap(openList.begin(), openList.end(), compFunc);
	            }
	        } else if (item->node_attr == ANODE_ATTR_NULL) {
	            item->node_attr = ANODE_ATTR_OPEN;
	            item->node_g = curNode->node_g + topoGraphNodes[curNode->node_id].nodeValues[item];
	            item->node_h = HFuc(item, enode);
	            item->node_father = curNode;
	            openList.push_back(item);
	            std::push_heap(openList.begin(), openList.end(), compFunc);
	        }
	    }
	}
	return ERRCODE_SUCCESS;
}



void TopoGraph::BulidTopoGraph(File *file){
	WaySEId waySEId;
	for(const auto &it : file->ways){
		auto way_id = it.first;
		auto nodes = it.second.nodes;
		auto s_id = it.second.nodes.front()->id;
		auto s_node = AddNode(s_id, file->nodes[s_id].point.x, file->nodes[s_id].point.y);
		auto e_id = it.second.nodes.back()->id;
		auto e_node = AddNode(e_id, file->nodes[e_id].point.x, file->nodes[e_id].point.y);
		auto way_attributes = it.second.attributes;
		AddEdge(s_node,e_node,nodes,way_attributes);
		waySEId.sId = s_id;
		waySEId.eId = e_id;
		osm2Topo.emplace(way_id,waySEId);
	}
	
	for(const auto &item : topoGraphNodes){
		auto cout =
			item.second.adjacentNodes.size();
		if(cout >= 3)
			item.second.theNode->node_crossing = true;
	}
	file_ = file;
}


bool TopoGraph::BulidTmpTopoGraph(double sPointX, double sPointY, double ePointX, double ePointY ){
	//判断是否共线的：
	auto startId = file_->findNearestNode(sPointX, sPointY);
	auto endId = file_->findNearestNode(ePointX, ePointY);
	auto startNodeId = startId.second;
	auto endNodeId = endId.second;
	auto startWayId = startId.first;
	auto endWayId = endId.first;
	auto startNodeFind = topoGraphNodes.find(startNodeId);
	auto endNodeFind = topoGraphNodes.find(endNodeId);
	if (startNodeFind == topoGraphNodes.end() && endNodeFind == topoGraphNodes.end() && startWayId == endWayId) {
	    auto WayId = startWayId;
	    auto front_id = file_->ways[WayId].nodes.front()->id;
	    auto front_node = file_->ways[WayId].nodes.front();
	    auto front_aNode = topoGraphNodes[front_id].theNode;
	    auto back_node = file_->ways[WayId].nodes.back();
	    auto back_id = file_->ways[WayId].nodes.back()->id;
	    auto back_aNode = topoGraphNodes[back_id].theNode;
	    auto nodes = file_->ways[WayId].nodes;
		auto WayAtb = file_->ways[WayId].attributes;
		RmvEdge(front_aNode, back_aNode);
	    auto startNode = AddNode(startNodeId, file_->nodes[startNodeId].point.x, file_->nodes[startNodeId].point.y);
	    auto endNode = AddNode(endNodeId, file_->nodes[endNodeId].point.x, file_->nodes[endNodeId].point.y);
	    WaySEId waySEId;
	    Node* start_node_ptr = nullptr;
	    Node* end_node_ptr = nullptr;
	    for (const auto& iter : nodes) {
	        if (startNodeId == iter->id) start_node_ptr = iter;
	        if (endNodeId == iter->id) end_node_ptr = iter;
	    }
	    double dist_start = hypot(file_->nodes[startNodeId].point.x - front_node->point.x,
	                              file_->nodes[startNodeId].point.y - front_node->point.y);
	    double dist_end = hypot(file_->nodes[endNodeId].point.x - front_node->point.x,
	                            file_->nodes[endNodeId].point.y - front_node->point.y);
	    int64_t closer_node_id, farther_node_id;
	    Node* closer_node_ptr;
	    Node* farther_node_ptr;
	    std::shared_ptr<ANode> closer_aNode;
	    std::shared_ptr<ANode> farther_aNode;
	    if (dist_start <= dist_end) {
	        closer_node_id = startNodeId;
	        farther_node_id = endNodeId;
	        closer_node_ptr = start_node_ptr;
	        farther_node_ptr = end_node_ptr;
	        closer_aNode = startNode;
	        farther_aNode = endNode;
	    } else {
	        closer_node_id = endNodeId;
	        farther_node_id = startNodeId;
	        closer_node_ptr = end_node_ptr;
	        farther_node_ptr = start_node_ptr;
	        closer_aNode = endNode;
	        farther_aNode = startNode;
	    }
	    std::vector<Node*> cut1_nodes(nodes.begin(), std::find(nodes.begin(), nodes.end(), closer_node_ptr) + 1);
	    std::vector<Node*> cut2_nodes(std::find(nodes.begin(), nodes.end(), closer_node_ptr),
	                                  std::find(nodes.begin(), nodes.end(), farther_node_ptr) + 1);
	    std::vector<Node*> cut3_nodes(std::find(nodes.begin(), nodes.end(), farther_node_ptr), nodes.end());
	    waySEId.sId = front_id;
	    waySEId.eId = closer_node_id;
	    osmSE2Topo.emplace(cut1_nodes, waySEId);
	    waySEId.sId = closer_node_id;
	    waySEId.eId = farther_node_id;
	    osmSE2Topo.emplace(cut2_nodes, waySEId);
	    waySEId.sId = farther_node_id;
	    waySEId.eId = back_id;
	    osmSE2Topo.emplace(cut3_nodes, waySEId);
	    AddEdge(front_aNode, closer_aNode, cut1_nodes, WayAtb);
	    AddEdge(closer_aNode, farther_aNode, cut2_nodes, WayAtb);
	    AddEdge(farther_aNode, back_aNode, cut3_nodes, WayAtb);
		sId = startNodeId;
		eId = endNodeId;
		/**
		std::cout<<"-------------osmSETopo------------"<<std::endl;
		for (const auto& pair : osmSE2Topo) {
		    std::cout << "sId:" << pair.second.sId << ", eId:" << pair.second.eId << " | ";
		    for (size_t i = 0; i < pair.first.size(); ++i) {
		        std::cout << pair.first[i]->id;
		        if (i != pair.first.size() - 1) std::cout << "->";
		    }
		    std::cout << std::endl;
		}
		**/
	}
	else{
		sId = AddNode2TopoGraph(sPointX, sPointY);
		//std::cout<<sId<<std::endl;
		eId = AddNode2TopoGraph(ePointX, ePointY);
		//std::cout<<eId<<std::endl;
	}
	return true;
}

Point2DEqual TopoGraph::ArePoint2DEqual(const WaySEId& idPair1, const WaySEId idPair2)
{
	if(idPair1.sId == idPair2.sId && idPair1.eId == idPair2.eId){
		return POINT2D_POSITIVE_EQUAL;
	}
	else if(idPair1.sId == idPair2.eId && idPair1.eId == idPair2.sId){
		return POINT2D_NEGATIVE_EQUAL;
	}
	else
		return POINT2D_UNEQUAL;
}

Id TopoGraph::AddNode2TopoGraph(double pointX, double pointY){
	auto Id = file_->findNearestNode(pointX, pointY);
	auto NodeId = Id.second;
	WaySEId waySEId;
	auto it = topoGraphNodes.find(NodeId);
	if (it == topoGraphNodes.end()){
	   	auto WayId = Id.first;
		auto WayAtb = file_->ways[WayId].attributes;
		auto front_id = file_->ways[WayId].nodes.front()->id;
		auto front_node = file_->ways[WayId].nodes.front();
		auto front_aNode = topoGraphNodes[front_id].theNode;
		auto back_node = file_->ways[WayId].nodes.back();
		auto back_id = file_->ways[WayId].nodes.back()->id;
		auto back_aNode = topoGraphNodes[back_id].theNode;

		auto nodes = file_->ways[WayId].nodes;

		RmvEdge(front_aNode,back_aNode);

		auto node = AddNode(NodeId, file_->nodes[NodeId].point.x, file_->nodes[NodeId].point.y);
		
		Node* node_ptr;
		for(const auto &iter : nodes){
			if(NodeId == iter->id)
				node_ptr = iter;
		}
		std::vector<Node*> cut1_nodes(nodes.begin(),std::find(nodes.begin(),nodes.end(),node_ptr)+1);
		std::vector<Node*> cut2_nodes(std::find(nodes.begin(),nodes.end(),node_ptr),nodes.end());
		
		waySEId.sId = front_id;
		waySEId.eId = NodeId;
		osmSE2Topo.emplace(cut1_nodes,waySEId);
		waySEId.sId = NodeId;
		waySEId.eId = back_id;
		osmSE2Topo.emplace(cut2_nodes,waySEId);
		
		AddEdge(front_aNode,node,cut1_nodes,WayAtb);
		AddEdge(node,back_aNode,cut2_nodes,WayAtb);
	}
	return NodeId;
}


