
#include "route/osmFileParser.h"
#include <amathutils_lib/amathutils.hpp>

Attributes tags(const pugi::xml_node& node) {
  Attributes attributes;
  for (auto tag = node.child(keyword::Tag); tag;
       tag = tag.next_sibling(keyword::Tag)) {
    if (std::string(tag.attribute(keyword::Key).value()) == keyword::Elevation) {
      continue;
    }
    attributes[tag.attribute(keyword::Key).value()] = tag.attribute(keyword::Value).value();
  }
  return attributes;
}

bool isDeleted(const pugi::xml_node& node) {
  auto action = node.attribute(keyword::Action);
  return action && std::string(action.value()) == keyword::Delete;
}

File OsmFileParser::read(const pugi::xml_node& fileNode) {
	File file;
	auto osmNode = fileNode.child(keyword::Osm);
	file.nodes = readNodes(osmNode);
	file.ways  = readWays(osmNode, file.nodes);
	return file;
}

Nodes OsmFileParser::readNodes(const pugi::xml_node& osmNode) {
  Nodes nodes;
  for (auto node = osmNode.child(keyword::Node); node;
	   node = node.next_sibling(keyword::Node)) {
	if (isDeleted(node)) {
	  continue;
	}
	const auto id = node.attribute(keyword::Id).as_llong(InvalId);
	const auto attributes = tags(node);
	const auto lat = node.attribute(keyword::Lat).as_double(0.);
	const auto lon = node.attribute(keyword::Lon).as_double(0.);
	const auto ele = node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation)
						 .attribute(keyword::Value)
						 .as_double(0.);
	geometry_msgs::Point gps_point,utm_point;
	gps_point.x = lat;
	gps_point.y = lon;
	gps_point.z = ele;
	utm_point = projector_.forward(gps_point);
	nodes[id] = Node{id, attributes, {utm_point.x,utm_point.y,utm_point.z}};
  }
  return nodes;
}

Ways OsmFileParser::readWays(const pugi::xml_node& osmNode, Nodes& nodes) {
    Ways ways;
    for (auto node = osmNode.child(keyword::Way); node;  // NOLINT
         node = node.next_sibling(keyword::Way)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto nodeIds = [&node] {
        Ids ids;
        for (auto refNode = node.child(keyword::Nd); refNode;  // NOLINT
             refNode = refNode.next_sibling(keyword::Nd)) {
          ids.push_back(refNode.attribute(keyword::Ref).as_llong());
        }
        return ids;
      }();
      std::vector<Node*> wayNodes;
      try {
        wayNodes = transform(nodeIds, [&nodes](const auto& elem) { return &nodes.at(elem); });
      } catch (std::out_of_range&) {
        reportParseError(id, "Way references nonexisting points");
      }
      ways[id] = Way{id, attributes, wayNodes};
    }
    return ways;
  }

void OsmFileParser::reportParseError(Id id, const std::string& what) {
	auto errstr = "Error reading primitive with id " + std::to_string(id) + " from file: " + what;
	errors_.push_back(errstr);
}


IdPair File::findNearestNode(double curX,double curY){
	std::map<IdPair,double> values;
	for(const auto &it : ways){
		auto nodes = it.second.nodes;
		auto wayId = it.first;
		for(const auto &iter : nodes){
			auto utmPoint = iter->point;
			auto pointId = iter->id;
			double d = amathutils::distance2D(utmPoint.x,utmPoint.y,curX,curY);
			values.insert(std::make_pair(std::make_pair(wayId,pointId),d));
		}
	}
	double minValue = DBL_MAX;
	Id nearestWayId = 0;
	Id nearestNodeId = 0;
	for(const auto &value : values){
		if(minValue > value.second){
			minValue = value.second;
			nearestWayId = value.first.first;
			nearestNodeId = value.first.second;
		}
	}
	return std::make_pair(nearestWayId,nearestNodeId);
}

/**
std::pair<Id, std::vector<Id>> File::findNearest(double curX, double curY) {
    std::vector<std::tuple<Id, Id, double>> node_way_dist;
    for (const auto& it : ways) {
        auto wayId = it.first;
        auto nodes = it.second.nodes;
        for (const auto& iter : nodes) {
            auto utmPoint = iter->point;
            auto pointId = iter->id;
            double d = amathutils::distance2D(utmPoint.x, utmPoint.y, curX, curY);
            node_way_dist.emplace_back(pointId, wayId, d);
        }
    }
    double minValue = DBL_MAX;
    for (const auto& tuple : node_way_dist) {
        double d = std::get<2>(tuple);
        if (d < minValue)
            minValue = d;
    }
    std::map<Id, std::vector<Id>> node_to_ways;
    for (const auto& tuple : node_way_dist) {
        Id nodeId = std::get<0>(tuple);
        Id wayId = std::get<1>(tuple);
        double d = std::get<2>(tuple);
        if (std::fabs(d - minValue) < 1e-9)
            node_to_ways[nodeId].push_back(wayId);
    }
    Id nearestNodeId = 0;
    std::vector<Id> nearestWayIds;
    if (!node_to_ways.empty()) {
        nearestNodeId = node_to_ways.begin()->first;
        nearestWayIds = node_to_ways.begin()->second;
    }
    return std::make_pair(nearestNodeId, nearestWayIds);
}
**/

