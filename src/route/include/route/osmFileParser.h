#pragma once

#include <map>
#include <vector>
#include "route/common.h"
#include <pugixml.hpp>
#include "utm/UTM.h"
#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"

namespace keyword {
constexpr const char* Osm = "osm";
constexpr const char* Tag = "tag";
constexpr const char* Key = "k";
constexpr const char* Value = "v";
constexpr const char* Node = "node";
constexpr const char* Way = "way";
constexpr const char* Relation = "relation";
constexpr const char* Member = "member";
constexpr const char* Role = "role";
constexpr const char* Type = "type";
constexpr const char* Nd = "nd";
constexpr const char* Ref = "ref";
constexpr const char* Id = "id";
constexpr const char* Lat = "lat";
constexpr const char* Lon = "lon";
constexpr const char* Version = "version";
constexpr const char* Visible = "visible";
constexpr const char* Elevation = "ele";
constexpr const char* Action = "action";
constexpr const char* Delete = "delete";
constexpr const char* Direction = "direction";
constexpr const char* Bidir = "bidir";
constexpr const char* Unidir = "unidir";
}


using Attributes = std::map<std::string, std::string>;
using Id = int64_t;
using Ids = std::vector<Id>;
using Errors = std::vector<std::string>;
using IdPair = std::pair<Id, Id>;

constexpr Id InvalId = 0;
const int MAX_INT = 0x0fffffff;


//Common abstract base class for all osm primitives. Provides id and attributes.
struct Primitive {
  Primitive() = default;
  Primitive(Primitive&& rhs) noexcept = default;
  Primitive& operator=(Primitive&& rhs) noexcept = default;
  Primitive(const Primitive& rhs) = delete;
  Primitive& operator=(const Primitive& rhs) = delete;
  virtual ~Primitive() = default;
  Primitive(Id id, Attributes attributes) : id{id}, attributes{std::move(attributes)} {}
  virtual std::string type() = 0;

  Id id{0};
  Attributes attributes;
};

//Osm node object
struct Node : public Primitive {
  Node() = default;
  Node(Id id, Attributes attributes, UtmPoint point)
  		: Primitive{id, std::move(attributes)}, point{point} {}
  std::string type() override { return "node"; }
  UtmPoint point;
};

//Osm way object
struct Way : public Primitive {
  Way() = default;
  Way(Id id, Attributes attributes, std::vector<Node*> nodes)
      : Primitive{id, std::move(attributes)}, nodes{std::move(nodes)} {}
  std::string type() override { return "way"; }
  std::vector<Node*> nodes;
};

struct WaySEId{
	Id sId{0};
	Id eId{0};
};


using Nodes = std::map<Id, Node>;
using Ways = std::map<Id, Way>;

struct File {
  File() noexcept = default;
  File(File&& rhs) noexcept = default;  // NOLINT
  File& operator=(File&& rhs) noexcept = default;
  File(const File& rhs) = delete;
  File& operator=(const File& rhs) = delete;
  ~File() noexcept = default;
  IdPair findNearestNode(double curX,double curY);
  Nodes nodes;
  Ways ways;
};


Attributes tags(const pugi::xml_node& node);
bool isDeleted(const pugi::xml_node& node);


template <typename Container, typename Func>
auto transform(Container&& c, Func f) {
  using RetT = std::decay_t<decltype(f(*c.begin()))>;
  std::vector<RetT> transformed;
  transformed.reserve(c.size());
  std::transform(c.begin(), c.end(), std::back_inserter(transformed), f);
  return transformed;
}



class OsmFileParser {
 public:
 	OsmFileParser() = default;
 	OsmFileParser(projection::UtmProjector projector){projector_ = projector;}
	File  read(const pugi::xml_node& fileNode);
	Nodes readNodes(const pugi::xml_node& osmNode);
 	Ways  readWays(const pugi::xml_node& osmNode, Nodes& nodes);
 	void  reportParseError(Id id, const std::string& what);
 private:
	projection::UtmProjector projector_;
	Errors errors_;
};



