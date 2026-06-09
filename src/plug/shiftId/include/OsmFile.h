#pragma once

#include <deque>
#include <map>
#include <pugixml.hpp>
#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

namespace lanelet {
namespace osm {


class GPSPoint {
 public:
  double lat{0.};  //! lat according to WGS84
  double lon{0.};  //! lon according to WGS84
  double ele{0.};  //! elevation according to WGS84 (m)
};


using Id =  long long ;
using Ids = std::vector<Id>;

struct Primitive;
using Attributes = std::map<std::string, std::string>;
using Role = std::pair<std::string, Primitive*>;
using Roles = std::deque<Role>;  // need iterator validity on push_back
using Errors = std::vector<std::string>;

//! Common abstract base class for all osm primitives. Provides id and attributes.
struct Primitive {
  Primitive() = default;
  Primitive(Primitive&& rhs) noexcept = default;  // NOLINT
  Primitive& operator=(Primitive&& rhs) noexcept = default;
  Primitive(const Primitive& rhs) = delete;
  Primitive& operator=(const Primitive& rhs) = delete;
  virtual ~Primitive() = default;
  Primitive(Id id, Attributes attributes) : id{id}, attributes{std::move(attributes)} {}
  virtual std::string type() = 0;

  Id id{0};
  Attributes attributes;
};

//! Osm node object
struct Node : public Primitive {
  Node() = default;
  Node(Id id, Attributes attributes, GPSPoint point) : Primitive{id, std::move(attributes)}, point{point} {}
  std::string type() override { return "node"; }
  GPSPoint point;
};

//! Osm way object
struct Way : public Primitive {
  Way() = default;
  Way(Id id, Attributes attributes, std::vector<Node*> nodes)
      : Primitive{id, std::move(attributes)}, nodes{std::move(nodes)} {}
  std::string type() override { return "way"; }
  std::vector<Node*> nodes;
};

//! Osm relation object
struct Relation : public Primitive {
  Relation() = default;
  Relation(Id id, Attributes attributes, Roles roles = Roles())
      : Primitive{id, std::move(attributes)}, members{std::move(roles)} {}
  std::string type() override { return "relation"; }
  Roles members;
};

using Nodes = std::map<Id, Node>;
using Ways = std::map<Id, Way>;
using Relations = std::map<Id, Relation>;

/**
 * @brief Intermediate representation of an osm file.
 *
 * Tries its best to cover the osm file specification.
 */
struct File {
  File() noexcept = default;
  File(File&& rhs) noexcept = default;  // NOLINT
  File& operator=(File&& rhs) noexcept = default;
  File(const File& rhs) = delete;
  File& operator=(const File& rhs) = delete;
  ~File() noexcept = default;

  Nodes nodes;
  Ways ways;
  Relations relations;
};




//! Reads an xml document into an osm file representation and optionally reports
//! parser errors.
File read(pugi::xml_document& node, lanelet::osm::Errors* errors = nullptr);

//! Creates an xml representation from an osm file representation. This is
//! guaranteed to work without errors.

inline bool operator==(const Node& lhs, const Node& rhs) { return lhs.id == rhs.id; }
bool operator==(const Way& lhs, const Way& rhs);
bool operator==(const Relation& lhs, const Relation& rhs);
bool operator==(const File& lhs, const File& rhs);

File parse(const std::string& filename) ;

std::unique_ptr<pugi::xml_document>  write(const File& osmFile) ;

template <typename Container, typename Func>
auto transform_detail(Container&& c, Func f) {
  using RetT = std::decay_t<decltype(f(*c.begin()))>;
  std::vector<RetT> transformed;
  transformed.reserve(c.size());
  std::transform(c.begin(), c.end(), std::back_inserter(transformed), f);
  return transformed;
}


template <typename Container, typename Func>
auto transform(Container&& c, Func f) {
  return transform_detail(std::forward<Container>(c), f);
}


}  // namespace osm
}  // namespace lanelet
