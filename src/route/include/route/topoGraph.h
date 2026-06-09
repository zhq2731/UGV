#pragma once

#include "route/osmFileParser.h"
#include "amathutils_lib/geometry.hpp"

enum ANodeAttr {
    ANODE_ATTR_NULL = 0,
    ANODE_ATTR_OPEN,
    ANODE_ATTR_CLOSE,
    ANODE_ATTR_FORBID,
};

enum ErrCode {
    ERRCODE_SUCCESS = 0,
    ERRCODE_NODE_NOT_EXSIT = 1,
    ERRCODE_NODE_FORBID = 2,
    ERRCODE_NO_PATH = 3,
    ERRCODE_NODE_BEYOND_GRAPH = 4,
};

enum Point2DEqual{
	POINT2D_POSITIVE_EQUAL = 0,
	POINT2D_NEGATIVE_EQUAL = 1,
	POINT2D_UNEQUAL = 2,
};



class ANode {
public:
	ANode(int id,double x,double y):node_id(id),node_x(x),node_y(y){
		Init();
	}
	int node_id;
	double node_x;
    double node_y;
	double node_z;
    double node_g;
    double node_h;
	bool node_crossing = false;
	ANodeAttr node_attr;
    std::shared_ptr<ANode> node_father;
	void Init();
};

class NodeInfo {
public:
	NodeInfo(std::shared_ptr<ANode>    node = nullptr) : theNode(node)
    {
		std::vector<std::shared_ptr<ANode>>().swap(adjacentNodes);
        nodeValues.clear();
    }
    void AddAdjacentNode(std::shared_ptr<ANode> &node);
	void RmvAdjacentNode(std::shared_ptr<ANode> &node);
	void BuildWeight(std::shared_ptr<ANode> &node,std::vector<Node*> nodes);
	void DeleteWeight(std::shared_ptr<ANode> &node);
	std::shared_ptr<ANode> theNode;
	std::vector<std::shared_ptr<ANode>> adjacentNodes;
	std::map<std::shared_ptr<ANode>,double> nodeValues;
};



class TopoGraph {
	
public:
    TopoGraph(){ClearGraph();}
    ~TopoGraph(){ClearGraph();}
	Point2DEqual ArePoint2DEqual(const WaySEId& idPair1, const WaySEId idPair2);
	void BulidTopoGraph(File *file);
	bool BulidTmpTopoGraph(double sPointX, double sPointY, double ePointX, double ePointY );
	Id   AddNode2TopoGraph(double pointX, double pointY);
	std::shared_ptr<ANode> AddNode(int nodeId,double nodeX,double nodeY);
	void AddEdge(std::shared_ptr<ANode> snode, std::shared_ptr<ANode> enode, std::vector<Node*> nodes, Attributes attributes);
	void RmvEdge(std::shared_ptr<ANode> snode, std::shared_ptr<ANode> enode);
	void InitGraph();
	std::shared_ptr<ANode> SetNodeAttr(int nodeId, ANodeAttr attr);
	const std::vector<std::shared_ptr<ANode>> &GetAdjacent(int nodeId);
	std::shared_ptr<ANode> GetNode(int nodeId) const;
	double HFuc(std::shared_ptr<ANode> s, std::shared_ptr<ANode> e) const;
    void ClearGraph();
    ErrCode FindPath(int s_id, int e_id);
	void PrintGraph(std::vector<UtmPoint> &singleUtmResult , int e_id);
	void AddSEWay2Result(std::vector<UtmPoint> &singleUtmResult,WaySEId waySEId);
	void AddWay2Result(std::vector<UtmPoint> &singleUtmResult,WaySEId waySEId);
	bool PathBuild(Point2DEqual pointEqual,std::vector<Node*> trajectory,std::vector<UtmPoint> &singleUtmResult);
	void PointDeal(std::vector<Node*> trajectory,std::vector<UtmPoint> &singleUtmResult);
	std::map<int, NodeInfo> topoGraphNodes;
	File *file_;
	std::map<Id, WaySEId> osm2Topo;
	std::map<std::vector<Node*>,WaySEId> osmSE2Topo;
	Id sId;
	Id eId;
private:
	
};





