#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pugixml.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <route_msgs/InitPoint.h>
#include <route_msgs/MultiPoint.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <utm/UTM.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

struct CsvRow {
    std::unordered_map<std::string, std::string> values;
};

struct MapNode {
    std::string id;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct OfflinePoint {
    std::string point_id;
    std::string node_id;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct Edge {
    int from = 0;
    int to = 0;
    double distance = 0.0;
};

struct Vehicle {
    int id = 0;
    std::string start_point_id;
    double start_x = 0.0;
    double start_y = 0.0;
    double start_yaw = 0.0;
    std::string multi_point_topic;
    std::string init_point_topic;
};

struct Task {
    int id = 0;
    std::string point_id;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double deal_time = 0.0;
    std::string node_id;
};

struct RouteItem {
    Task task;
    double travel = 0.0;
    double start_time = 0.0;
    double finish_time = 0.0;
};

struct Solution {
    double fitness = std::numeric_limits<double>::infinity();
    double makespan = 0.0;
    double total_travel = 0.0;
    std::vector<double> vehicle_times;
    std::vector<std::vector<RouteItem>> routes;
};

struct Individual {
    std::vector<int> order;
    std::vector<int> assignment;
};

std::string trim(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> result;
    std::string item;
    std::stringstream ss(line);
    while (std::getline(ss, item, ',')) {
        result.push_back(trim(item));
    }
    return result;
}

std::vector<CsvRow> read_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot open CSV: " + path);
    }

    std::vector<std::string> headers;
    std::vector<CsvRow> rows;
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto values = split_csv_line(line);
        if (headers.empty()) {
            headers = values;
            continue;
        }
        CsvRow row;
        for (std::size_t i = 0; i < headers.size() && i < values.size(); ++i) {
            row.values[headers[i]] = values[i];
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

std::string get_string(const CsvRow& row, const std::string& key, const std::string& fallback = "") {
    const auto it = row.values.find(key);
    if (it == row.values.end() || it->second.empty()) {
        return fallback;
    }
    return it->second;
}

double get_double(const CsvRow& row, const std::string& key, double fallback = 0.0) {
    const std::string value = get_string(row, key);
    return value.empty() ? fallback : std::stod(value);
}

int get_int(const CsvRow& row, const std::string& key, int fallback = 0) {
    const std::string value = get_string(row, key);
    return value.empty() ? fallback : std::stoi(value);
}

bool file_exists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

double point_distance(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}

geometry_msgs::Pose make_pose(double x, double y, double yaw) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
}

std::unordered_map<std::string, double> read_origin_yaml(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot open global config: " + path);
    }

    std::unordered_map<std::string, double> result;
    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        const std::string key = trim(line.substr(0, colon));
        std::string value = trim(line.substr(colon + 1));
        value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
        value.erase(std::remove(value.begin(), value.end(), '\''), value.end());
        if ((key == "longitude" || key == "latitude" || key == "altitude") && !value.empty()) {
            result[key] = std::stod(value);
        }
    }
    return result;
}

class RoadNetwork {
public:
    void load_osm(const std::string& map_file, const std::string& global_config, bool use_osm_direction) {
        const auto origin = read_origin_yaml(global_config);
        geometry_msgs::Point map_origin;
        map_origin.x = origin.at("latitude");
        map_origin.y = origin.at("longitude");
        map_origin.z = origin.count("altitude") ? origin.at("altitude") : 0.0;
        projection::UtmProjector projector(map_origin);

        pugi::xml_document doc;
        const auto load_result = doc.load_file(map_file.c_str());
        if (!load_result) {
            throw std::runtime_error("cannot parse OSM: " + map_file);
        }

        nodes_.clear();
        node_to_index_.clear();
        edges_.clear();

        std::unordered_map<std::string, int> osm_to_index;
        const pugi::xml_node osm = doc.child("osm");
        for (pugi::xml_node node = osm.child("node"); node; node = node.next_sibling("node")) {
            const std::string id = node.attribute("id").value();
            geometry_msgs::Point gps;
            gps.x = node.attribute("lat").as_double();
            gps.y = node.attribute("lon").as_double();
            gps.z = 0.0;
            for (pugi::xml_node tag = node.child("tag"); tag; tag = tag.next_sibling("tag")) {
                if (std::string(tag.attribute("k").value()) == "ele") {
                    gps.z = tag.attribute("v").as_double();
                }
            }
            const geometry_msgs::Point utm = projector.forward(gps);
            osm_to_index[id] = static_cast<int>(nodes_.size());
            node_to_index_[id] = static_cast<int>(nodes_.size());
            nodes_.push_back({id, utm.x, utm.y, 0.0});
        }

        for (pugi::xml_node way = osm.child("way"); way; way = way.next_sibling("way")) {
            std::vector<int> way_nodes;
            for (pugi::xml_node nd = way.child("nd"); nd; nd = nd.next_sibling("nd")) {
                const std::string ref = nd.attribute("ref").value();
                const auto it = osm_to_index.find(ref);
                if (it != osm_to_index.end()) {
                    way_nodes.push_back(it->second);
                }
            }
            if (way_nodes.size() < 2) {
                continue;
            }

            bool bidirectional = !use_osm_direction;
            for (pugi::xml_node tag = way.child("tag"); tag; tag = tag.next_sibling("tag")) {
                if (std::string(tag.attribute("k").value()) == "direction" &&
                    std::string(tag.attribute("v").value()) == "bidir") {
                    bidirectional = true;
                }
            }

            for (std::size_t i = 1; i < way_nodes.size(); ++i) {
                const int from = way_nodes[i - 1];
                const int to = way_nodes[i];
                const double dist = point_distance(nodes_[from].x, nodes_[from].y, nodes_[to].x, nodes_[to].y);
                edges_.push_back({from, to, dist});
                if (bidirectional) {
                    edges_.push_back({to, from, dist});
                }
            }
        }

        update_node_yaws();
        build_adjacency();
    }

    int nearest_node(double x, double y) const {
        if (nodes_.empty()) {
            throw std::runtime_error("road network has no nodes");
        }
        int best = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            const double dist = point_distance(x, y, nodes_[i].x, nodes_[i].y);
            if (dist < best_dist) {
                best = static_cast<int>(i);
                best_dist = dist;
            }
        }
        return best;
    }

    int index_from_id(const std::string& node_id) const {
        const auto it = node_to_index_.find(node_id);
        if (it == node_to_index_.end()) {
            throw std::runtime_error("unknown map node id: " + node_id);
        }
        return it->second;
    }

    double distance(int from, int to) const {
        return shortest_distance(from, to);
    }

    const MapNode& node(int index) const {
        return nodes_.at(index);
    }

    std::vector<int> sample_node_indices(std::size_t count, unsigned int seed) const {
        if (count > nodes_.size()) {
            throw std::runtime_error("offline point count is larger than road network node count");
        }
        std::vector<int> indices(nodes_.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::mt19937 rng(seed);
        std::shuffle(indices.begin(), indices.end(), rng);
        indices.resize(count);
        return indices;
    }

    double shortest_distance(int from, int to) const {
        if (from == to) {
            return 0.0;
        }
        const int n = static_cast<int>(nodes_.size());
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
        dist[from] = 0.0;
        queue.push({0.0, from});
        while (!queue.empty()) {
            const auto [cost, current] = queue.top();
            queue.pop();
            if (current == to) {
                return cost;
            }
            if (cost > dist[current]) {
                continue;
            }
            for (const auto& [next, edge_cost] : adjacency_[current]) {
                const double alt = cost + edge_cost;
                if (alt < dist[next]) {
                    dist[next] = alt;
                    queue.push({alt, next});
                }
            }
        }
        throw std::runtime_error("unreachable node pair: " + nodes_[from].id + " -> " + nodes_[to].id);
    }

private:
    void update_node_yaws() {
        std::vector<bool> has_yaw(nodes_.size(), false);
        for (const auto& edge : edges_) {
            if (!has_yaw[edge.from]) {
                nodes_[edge.from].yaw = std::atan2(nodes_[edge.to].y - nodes_[edge.from].y,
                                                   nodes_[edge.to].x - nodes_[edge.from].x);
                has_yaw[edge.from] = true;
            }
        }
        for (const auto& edge : edges_) {
            if (!has_yaw[edge.to]) {
                nodes_[edge.to].yaw = std::atan2(nodes_[edge.to].y - nodes_[edge.from].y,
                                                 nodes_[edge.to].x - nodes_[edge.from].x);
                has_yaw[edge.to] = true;
            }
        }
    }

    void build_adjacency() {
        const int n = static_cast<int>(nodes_.size());
        adjacency_.assign(n, {});
        for (const auto& edge : edges_) {
            adjacency_[edge.from].push_back({edge.to, edge.distance});
        }
    }

    std::vector<MapNode> nodes_;
    std::vector<Edge> edges_;
    std::unordered_map<std::string, int> node_to_index_;
    std::vector<std::vector<std::pair<int, double>>> adjacency_;
};

std::unordered_map<std::string, OfflinePoint> load_offline_points(const std::string& path) {
    std::unordered_map<std::string, OfflinePoint> points;
    for (const auto& row : read_csv(path)) {
        OfflinePoint point;
        point.point_id = get_string(row, "point_id");
        point.node_id = get_string(row, "node");
        point.x = get_double(row, "x");
        point.y = get_double(row, "y");
        point.yaw = get_double(row, "yaw");
        if (point.point_id.empty()) {
            throw std::runtime_error("offline point row misses point_id");
        }
        points[point.point_id] = point;
    }
    if (points.empty()) {
        throw std::runtime_error("offline point map is empty: " + path);
    }
    return points;
}

std::vector<std::string> sorted_point_ids(const std::unordered_map<std::string, OfflinePoint>& offline_points) {
    std::vector<std::string> ids;
    ids.reserve(offline_points.size());
    for (const auto& item : offline_points) {
        ids.push_back(item.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

void save_offline_points(const std::string& path,
                         const RoadNetwork& network,
                         std::size_t count,
                         unsigned int seed) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot write offline point map: " + path);
    }

    file << "point_id,node,x,y,yaw\n";
    file << std::fixed << std::setprecision(6);
    const auto indices = network.sample_node_indices(count, seed);
    for (std::size_t i = 0; i < indices.size(); ++i) {
        const auto& node = network.node(indices[i]);
        file << 'P' << std::setw(3) << std::setfill('0') << (i + 1) << std::setfill(' ')
             << ',' << node.id
             << ',' << node.x
             << ',' << node.y
             << ',' << node.yaw
             << '\n';
    }
}

const OfflinePoint& offline_point(const std::unordered_map<std::string, OfflinePoint>& points,
                                  const std::string& point_id) {
    const auto it = points.find(point_id);
    if (it == points.end()) {
        throw std::runtime_error("unknown offline point id: " + point_id);
    }
    return it->second;
}

class OfflineDistanceTable {
public:
    void load(const std::string& path) {
        const auto rows = read_csv_matrix(path);
        if (rows.empty() || rows[0].size() < 2 || rows[0][0] != "point_id") {
            throw std::runtime_error("invalid offline distance table header: " + path);
        }

        point_ids_.assign(rows[0].begin() + 1, rows[0].end());
        point_to_index_.clear();
        for (std::size_t i = 0; i < point_ids_.size(); ++i) {
            point_to_index_[point_ids_[i]] = i;
        }

        distances_.assign(point_ids_.size(), std::vector<double>(point_ids_.size(), 0.0));
        if (rows.size() != point_ids_.size() + 1) {
            throw std::runtime_error("offline distance table size does not match header: " + path);
        }
        for (std::size_t r = 1; r < rows.size(); ++r) {
            if (rows[r].size() != point_ids_.size() + 1) {
                throw std::runtime_error("offline distance table row width mismatch: " + path);
            }
            const std::string row_id = rows[r][0];
            const auto row_it = point_to_index_.find(row_id);
            if (row_it == point_to_index_.end()) {
                throw std::runtime_error("unknown row point in distance table: " + row_id);
            }
            const std::size_t row_index = row_it->second;
            for (std::size_t c = 0; c < point_ids_.size(); ++c) {
                distances_[row_index][c] = std::stod(rows[r][c + 1]);
            }
        }
    }

    double distance(const std::string& from, const std::string& to) const {
        const auto from_it = point_to_index_.find(from);
        const auto to_it = point_to_index_.find(to);
        if (from_it == point_to_index_.end()) {
            throw std::runtime_error("distance table misses point: " + from);
        }
        if (to_it == point_to_index_.end()) {
            throw std::runtime_error("distance table misses point: " + to);
        }
        return distances_[from_it->second][to_it->second];
    }

private:
    static std::vector<std::vector<std::string>> read_csv_matrix(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("cannot open offline distance table: " + path);
        }
        std::vector<std::vector<std::string>> rows;
        std::string line;
        while (std::getline(file, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }
            rows.push_back(split_csv_line(line));
        }
        return rows;
    }

    std::vector<std::string> point_ids_;
    std::unordered_map<std::string, std::size_t> point_to_index_;
    std::vector<std::vector<double>> distances_;
};

void save_offline_distance_table(const std::string& path,
                                 const RoadNetwork& network,
                                 const std::unordered_map<std::string, OfflinePoint>& offline_points) {
    if (path.empty()) {
        return;
    }
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot write offline distance table: " + path);
    }

    const auto point_ids = sorted_point_ids(offline_points);
    file << "point_id";
    for (const auto& point_id : point_ids) {
        file << ',' << point_id;
    }
    file << '\n';
    file << std::fixed << std::setprecision(6);

    for (const auto& from_id : point_ids) {
        const auto& from = offline_point(offline_points, from_id);
        const int from_node = network.index_from_id(from.node_id);
        file << from_id;
        for (const auto& to_id : point_ids) {
            const auto& to = offline_point(offline_points, to_id);
            const int to_node = network.index_from_id(to.node_id);
            file << ',' << network.shortest_distance(from_node, to_node);
        }
        file << '\n';
    }
}

std::vector<Vehicle> read_vehicles(const std::string& path,
                                   const RoadNetwork&,
                                   const std::unordered_map<std::string, OfflinePoint>& offline_points) {
    std::vector<Vehicle> vehicles;
    for (const auto& row : read_csv(path)) {
        Vehicle vehicle;
        vehicle.id = get_int(row, "id");
        vehicle.start_point_id = get_string(row, "start_point_id");
        if (!vehicle.start_point_id.empty()) {
            const auto& point = offline_point(offline_points, vehicle.start_point_id);
            vehicle.start_x = point.x;
            vehicle.start_y = point.y;
            vehicle.start_yaw = point.yaw;
        } else {
            throw std::runtime_error("vehicle row must use start_point_id for offline distance table mode");
        }
        vehicle.multi_point_topic = get_string(row, "multi_point_topic", "/vehicle_" + std::to_string(vehicle.id) + "/multi_point_planning");
        vehicle.init_point_topic = get_string(row, "init_point_topic", "/vehicle_" + std::to_string(vehicle.id) + "/init_point");
        vehicles.push_back(vehicle);
    }
    return vehicles;
}

std::vector<Task> read_tasks(const std::string& path,
                             const RoadNetwork&,
                             const std::unordered_map<std::string, OfflinePoint>& offline_points) {
    std::vector<Task> tasks;
    for (const auto& row : read_csv(path)) {
        Task task;
        task.id = get_int(row, "id");
        task.point_id = get_string(row, "point_id");
        if (!task.point_id.empty()) {
            const auto& point = offline_point(offline_points, task.point_id);
            task.x = point.x;
            task.y = point.y;
            task.yaw = point.yaw;
            task.node_id = point.node_id;
        } else {
            throw std::runtime_error("task row must use point_id for offline distance table mode");
        }
        task.deal_time = get_double(row, "deal_time");
        tasks.push_back(task);
    }
    return tasks;
}

class GeneticScheduler {
public:
    GeneticScheduler(std::vector<Vehicle> vehicles,
                     std::vector<Task> tasks,
                     const OfflineDistanceTable& distance_table,
                     int population_size,
                     int generations,
                     unsigned int seed)
        : vehicles_(std::move(vehicles)),
          tasks_(std::move(tasks)),
          distance_table_(distance_table),
          population_size_(population_size),
          generations_(generations),
          rng_(seed) {
        if (vehicles_.empty()) {
            throw std::runtime_error("no vehicles configured");
        }
        if (tasks_.empty()) {
            throw std::runtime_error("no tasks configured");
        }
    }

    Solution solve() {
        std::vector<Individual> population;
        population.reserve(population_size_);
        for (int i = 0; i < population_size_; ++i) {
            population.push_back(random_individual());
        }

        Solution best;
        Individual best_individual;
        for (int generation = 0; generation < generations_; ++generation) {
            std::vector<Solution> evaluated;
            evaluated.reserve(population.size());
            for (const auto& individual : population) {
                evaluated.push_back(evaluate(individual));
            }
            const auto best_it = std::min_element(evaluated.begin(), evaluated.end(), [](const Solution& a, const Solution& b) {
                return a.fitness < b.fitness;
            });
            const int best_index = static_cast<int>(std::distance(evaluated.begin(), best_it));
            if (evaluated[best_index].fitness < best.fitness) {
                best = evaluated[best_index];
                best_individual = population[best_index];
            }

            std::vector<Individual> next;
            next.reserve(population_size_);
            next.push_back(best_individual);
            while (static_cast<int>(next.size()) < population_size_) {
                Individual a = tournament_select(population, evaluated);
                Individual b = tournament_select(population, evaluated);
                auto children = crossover(a, b);
                mutate(children.first);
                mutate(children.second);
                next.push_back(children.first);
                if (static_cast<int>(next.size()) < population_size_) {
                    next.push_back(children.second);
                }
            }
            population = std::move(next);
        }
        return best;
    }

private:
    Individual random_individual() {
        Individual individual;
        individual.order.resize(tasks_.size());
        std::iota(individual.order.begin(), individual.order.end(), 0);
        std::shuffle(individual.order.begin(), individual.order.end(), rng_);
        std::uniform_int_distribution<int> vehicle_dist(0, static_cast<int>(vehicles_.size()) - 1);
        individual.assignment.resize(tasks_.size());
        for (auto& item : individual.assignment) {
            item = vehicle_dist(rng_);
        }
        return individual;
    }

    Solution evaluate(const Individual& individual) const {
        Solution solution;
        solution.vehicle_times.assign(vehicles_.size(), 0.0);
        solution.routes.assign(vehicles_.size(), {});
        std::vector<std::string> vehicle_positions;
        for (const auto& vehicle : vehicles_) {
            vehicle_positions.push_back(vehicle.start_point_id);
        }

        for (const int task_index : individual.order) {
            const int vehicle_index = individual.assignment[task_index];
            const Task& task = tasks_[task_index];
            const double travel = distance_table_.distance(vehicle_positions[vehicle_index], task.point_id);
            const double start_time = solution.vehicle_times[vehicle_index] + travel;
            const double finish_time = start_time + task.deal_time;
            solution.routes[vehicle_index].push_back({task, travel, start_time, finish_time});
            solution.vehicle_times[vehicle_index] = finish_time;
            solution.total_travel += travel;
            vehicle_positions[vehicle_index] = task.point_id;
        }
        solution.makespan = *std::max_element(solution.vehicle_times.begin(), solution.vehicle_times.end());
        solution.fitness = solution.makespan + solution.total_travel * 0.01;
        return solution;
    }

    Individual tournament_select(const std::vector<Individual>& population, const std::vector<Solution>& evaluated) {
        std::uniform_int_distribution<int> index_dist(0, static_cast<int>(population.size()) - 1);
        int best = index_dist(rng_);
        for (int i = 1; i < 4; ++i) {
            const int candidate = index_dist(rng_);
            if (evaluated[candidate].fitness < evaluated[best].fitness) {
                best = candidate;
            }
        }
        return population[best];
    }

    std::pair<Individual, Individual> crossover(const Individual& a, const Individual& b) {
        if (tasks_.size() < 2) {
            return {a, b};
        }
        std::uniform_real_distribution<double> real_dist(0.0, 1.0);
        if (real_dist(rng_) > 0.85) {
            return {a, b};
        }

        const int size = static_cast<int>(tasks_.size());
        std::uniform_int_distribution<int> point_dist(0, size - 1);
        int left = point_dist(rng_);
        int right = point_dist(rng_);
        if (left > right) {
            std::swap(left, right);
        }

        Individual child_a;
        Individual child_b;
        child_a.order = order_crossover(a.order, b.order, left, right);
        child_b.order = order_crossover(b.order, a.order, left, right);
        child_a.assignment.resize(size);
        child_b.assignment.resize(size);
        for (int i = 0; i < size; ++i) {
            if (real_dist(rng_) < 0.5) {
                child_a.assignment[i] = a.assignment[i];
                child_b.assignment[i] = b.assignment[i];
            } else {
                child_a.assignment[i] = b.assignment[i];
                child_b.assignment[i] = a.assignment[i];
            }
        }
        return {child_a, child_b};
    }

    std::vector<int> order_crossover(const std::vector<int>& base, const std::vector<int>& donor, int left, int right) const {
        const int size = static_cast<int>(base.size());
        std::vector<int> child(size, -1);
        for (int i = left; i <= right; ++i) {
            child[i] = base[i];
        }
        int cursor = 0;
        for (const int task_index : donor) {
            if (std::find(child.begin(), child.end(), task_index) != child.end()) {
                continue;
            }
            while (child[cursor] != -1) {
                ++cursor;
            }
            child[cursor] = task_index;
        }
        return child;
    }

    void mutate(Individual& individual) {
        std::uniform_real_distribution<double> real_dist(0.0, 1.0);
        if (real_dist(rng_) < 0.08 && individual.order.size() >= 2) {
            std::uniform_int_distribution<int> point_dist(0, static_cast<int>(individual.order.size()) - 1);
            const int a = point_dist(rng_);
            const int b = point_dist(rng_);
            std::swap(individual.order[a], individual.order[b]);
        }
        std::uniform_int_distribution<int> vehicle_dist(0, static_cast<int>(vehicles_.size()) - 1);
        for (auto& item : individual.assignment) {
            if (real_dist(rng_) < 0.08) {
                item = vehicle_dist(rng_);
            }
        }
    }

    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    const OfflineDistanceTable& distance_table_;
    int population_size_ = 120;
    int generations_ = 400;
    std::mt19937 rng_;
};

void save_solution(const std::string& path,
                   const Solution& solution,
                   const std::vector<Vehicle>& vehicles) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot write solution: " + path);
    }
    file << "vehicle_id,multi_point_topic,sequence,task_id,point_id,node,x,y,yaw,travel,start_time,finish_time,deal_time\n";
    file << std::fixed << std::setprecision(6);
    for (std::size_t vehicle_index = 0; vehicle_index < solution.routes.size(); ++vehicle_index) {
        for (std::size_t sequence = 0; sequence < solution.routes[vehicle_index].size(); ++sequence) {
            const auto& item = solution.routes[vehicle_index][sequence];
            file << vehicles[vehicle_index].id << ','
                 << vehicles[vehicle_index].multi_point_topic << ','
                 << sequence << ','
                 << item.task.id << ','
                 << item.task.point_id << ','
                 << item.task.node_id << ','
                 << item.task.x << ','
                 << item.task.y << ','
                 << item.task.yaw << ','
                 << item.travel << ','
                 << item.start_time << ','
                 << item.finish_time << ','
                 << item.task.deal_time << '\n';
        }
    }
}

struct SolutionPublishers {
    std::vector<ros::Publisher> init_publishers;
    std::vector<ros::Publisher> multipoint_publishers;
    ros::Publisher task_marker_publisher;
};

class StartGate {
public:
    explicit StartGate(ros::NodeHandle& nh, const std::string& topic)
        : subscriber_(nh.subscribe(topic, 1, &StartGate::callback, this)) {}

    bool started() const {
        return started_;
    }

private:
    void callback(const std_msgs::Empty::ConstPtr&) {
        started_ = true;
        ROS_INFO("task scheduler start command received");
    }

    bool started_ = false;
    ros::Subscriber subscriber_;
};

std_msgs::ColorRGBA vehicle_color(int vehicle_id, double alpha) {
    std_msgs::ColorRGBA color;
    color.a = alpha;
    if (vehicle_id == 2) {
        color.r = 1.0;
        color.g = 0.18;
        color.b = 0.12;
    } else {
        color.r = 0.05;
        color.g = 0.35;
        color.b = 1.0;
    }
    return color;
}

visualization_msgs::Marker make_marker_base(const std::string& ns, int id, int type) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}

visualization_msgs::MarkerArray build_task_marker_array(const Solution& solution,
                                                        const std::vector<Vehicle>& vehicles) {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 1;
    for (std::size_t vehicle_index = 0; vehicle_index < solution.routes.size(); ++vehicle_index) {
        const int vehicle_id = vehicles[vehicle_index].id;
        const std::string vehicle_ns = "/task_scheduler/vehicle_" + std::to_string(vehicle_id);
        const auto color = vehicle_color(vehicle_id, 0.95);

        for (std::size_t sequence = 0; sequence < solution.routes[vehicle_index].size(); ++sequence) {
            const auto& item = solution.routes[vehicle_index][sequence];

            auto point = make_marker_base(vehicle_ns + "/tasks", marker_id++, visualization_msgs::Marker::CYLINDER);
            point.pose.position.x = item.task.x;
            point.pose.position.y = item.task.y;
            point.pose.position.z = 1.2;
            point.scale.x = 7.0;
            point.scale.y = 7.0;
            point.scale.z = 2.4;
            point.color = color;
            markers.markers.push_back(point);

            auto label = make_marker_base(vehicle_ns + "/labels", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING);
            label.pose.position.x = item.task.x;
            label.pose.position.y = item.task.y;
            label.pose.position.z = 8.0;
            label.scale.z = 5.0;
            label.color = color;
            label.color.a = 1.0;
            std::ostringstream text;
            text << "V" << vehicle_id << "-#" << (sequence + 1)
                 << "  T" << item.task.id << "(" << item.task.point_id << ")";
            label.text = text.str();
            markers.markers.push_back(label);
        }
    }
    return markers;
}

visualization_msgs::MarkerArray build_pending_task_marker_array(const std::vector<Task>& tasks) {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std_msgs::ColorRGBA yellow;
    yellow.r = 1.0;
    yellow.g = 0.82;
    yellow.b = 0.05;
    yellow.a = 0.95;

    int marker_id = 1;
    for (const auto& task : tasks) {
        auto point = make_marker_base("/task_scheduler/pending/tasks", marker_id++, visualization_msgs::Marker::CYLINDER);
        point.pose.position.x = task.x;
        point.pose.position.y = task.y;
        point.pose.position.z = 1.2;
        point.scale.x = 7.0;
        point.scale.y = 7.0;
        point.scale.z = 2.4;
        point.color = yellow;
        markers.markers.push_back(point);

        auto label = make_marker_base("/task_scheduler/pending/labels", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING);
        label.pose.position.x = task.x;
        label.pose.position.y = task.y;
        label.pose.position.z = 8.0;
        label.scale.z = 5.0;
        label.color = yellow;
        label.color.a = 1.0;
        std::ostringstream text;
        text << "T" << task.id << "(" << task.point_id << ")";
        label.text = text.str();
        markers.markers.push_back(label);
    }
    return markers;
}

bool wait_for_start_command(ros::NodeHandle& nh,
                            const std::string& start_topic,
                            const ros::Publisher& task_marker_publisher,
                            const std::vector<Task>& tasks,
                            double republish_interval_sec) {
    StartGate gate(nh, start_topic);
    ros::Rate wait_rate(20.0);
    ros::Time last_publish(0.0);
    ROS_INFO_STREAM("task scheduler waiting for start command on " << start_topic);
    while (ros::ok() && !gate.started()) {
        const ros::Time now = ros::Time::now();
        if (last_publish.isZero() || (now - last_publish).toSec() >= republish_interval_sec) {
            task_marker_publisher.publish(build_pending_task_marker_array(tasks));
            last_publish = now;
        }
        ros::spinOnce();
        wait_rate.sleep();
    }
    return ros::ok();
}

SolutionPublishers publish_solution(ros::NodeHandle& nh,
                                    const Solution& solution,
                                    const std::vector<Vehicle>& vehicles,
                                    double wait_for_subscribers_sec,
                                    double initial_delay_sec,
                                    int repeat_count,
                                    double repeat_interval_sec) {
    SolutionPublishers publishers;
    for (const auto& vehicle : vehicles) {
        publishers.init_publishers.push_back(nh.advertise<route_msgs::InitPoint>(vehicle.init_point_topic, 1, true));
        publishers.multipoint_publishers.push_back(nh.advertise<route_msgs::MultiPoint>(vehicle.multi_point_topic, 1, true));
    }
    publishers.task_marker_publisher =
        nh.advertise<visualization_msgs::MarkerArray>("/task_scheduler/task_markers", 1, true);

    const ros::Time start = ros::Time::now();
    ros::Rate wait_rate(20.0);
    while (ros::ok()) {
        bool all_connected = true;
        for (std::size_t i = 0; i < vehicles.size(); ++i) {
            all_connected = all_connected &&
                            publishers.init_publishers[i].getNumSubscribers() > 0 &&
                            publishers.multipoint_publishers[i].getNumSubscribers() > 0;
        }
        if (all_connected) {
            break;
        }
        if ((ros::Time::now() - start).toSec() >= wait_for_subscribers_sec) {
            ROS_WARN_STREAM("publish timeout: not all route subscribers connected, publishing latched task messages anyway");
            break;
        }
        wait_rate.sleep();
    }

    if (initial_delay_sec > 0.0) {
        ros::Duration(initial_delay_sec).sleep();
    }

    const int actual_repeat_count = std::max(1, repeat_count);
    for (int repeat = 0; repeat < actual_repeat_count && ros::ok(); ++repeat) {
        publishers.task_marker_publisher.publish(build_task_marker_array(solution, vehicles));
        for (std::size_t vehicle_index = 0; vehicle_index < vehicles.size(); ++vehicle_index) {
            const auto& vehicle = vehicles[vehicle_index];

            route_msgs::InitPoint init_msg;
            init_msg.header.stamp = ros::Time::now();
            init_msg.header.frame_id = "map";
            init_msg.pose = make_pose(vehicle.start_x, vehicle.start_y, vehicle.start_yaw);
            publishers.init_publishers[vehicle_index].publish(init_msg);

            route_msgs::MultiPoint task_msg;
            task_msg.header.stamp = ros::Time::now();
            task_msg.header.frame_id = "map";
            for (const auto& item : solution.routes[vehicle_index]) {
                task_msg.poses.push_back(make_pose(item.task.x, item.task.y, item.task.yaw));
            }
            publishers.multipoint_publishers[vehicle_index].publish(task_msg);
            ROS_INFO_STREAM("published " << task_msg.poses.size() << " tasks to " << vehicle.multi_point_topic
                                          << " (" << repeat + 1 << "/" << actual_repeat_count << ")");
        }
        if (repeat + 1 < actual_repeat_count && repeat_interval_sec > 0.0) {
            ros::Duration(repeat_interval_sec).sleep();
        }
    }
    return publishers;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_scheduler");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try {
        std::string map_file;
        std::string global_config;
        std::string vehicles_file;
        std::string tasks_file;
        std::string offline_points_file;
        std::string output_file;
        std::string distance_table_file;
        int offline_point_count = 50;
        int offline_point_seed = 20260622;
        int population = 120;
        int generations = 400;
        int seed = 7;
        bool regenerate_offline_points = false;
        bool regenerate_distance_table = false;
        bool publish_result = true;
        bool keep_alive_after_publish = true;
        bool use_osm_direction = true;
        bool wait_for_start = true;
        std::string start_topic = "/task_scheduler/start";
        double publish_wait_for_subscribers_sec = 5.0;
        double publish_initial_delay_sec = 5.0;
        double pending_marker_republish_interval_sec = 1.0;
        int publish_repeat_count = 5;
        double publish_repeat_interval_sec = 3.0;

        private_nh.param<std::string>("map_file", map_file, ros::package::getPath("launch_node") + "/data/cloudmap.osm");
        private_nh.param<std::string>("global_config", global_config, ros::package::getPath("launch_node") + "/param/global/global_config.yaml");
        private_nh.param<std::string>("vehicles_file", vehicles_file, ros::package::getPath("task_scheduler") + "/config/vehicles.csv");
        private_nh.param<std::string>("tasks_file", tasks_file, ros::package::getPath("task_scheduler") + "/config/tasks.csv");
        private_nh.param<std::string>("offline_points_file", offline_points_file, ros::package::getPath("task_scheduler") + "/config/offline_points_50.csv");
        private_nh.param<std::string>("output_file", output_file, "/tmp/ugv_schedule_solution.csv");
        private_nh.param<std::string>("distance_table_file", distance_table_file, ros::package::getPath("task_scheduler") + "/config/offline_distance_table_50.csv");
        private_nh.param("offline_point_count", offline_point_count, offline_point_count);
        private_nh.param("offline_point_seed", offline_point_seed, offline_point_seed);
        private_nh.param("population", population, population);
        private_nh.param("generations", generations, generations);
        private_nh.param("seed", seed, seed);
        private_nh.param("regenerate_offline_points", regenerate_offline_points, regenerate_offline_points);
        private_nh.param("regenerate_distance_table", regenerate_distance_table, regenerate_distance_table);
        private_nh.param("publish_result", publish_result, publish_result);
        private_nh.param("keep_alive_after_publish", keep_alive_after_publish, keep_alive_after_publish);
        private_nh.param("use_osm_direction", use_osm_direction, use_osm_direction);
        private_nh.param("wait_for_start", wait_for_start, wait_for_start);
        private_nh.param<std::string>("start_topic", start_topic, start_topic);
        private_nh.param("publish_wait_for_subscribers_sec", publish_wait_for_subscribers_sec, publish_wait_for_subscribers_sec);
        private_nh.param("publish_initial_delay_sec", publish_initial_delay_sec, publish_initial_delay_sec);
        private_nh.param("pending_marker_republish_interval_sec", pending_marker_republish_interval_sec, pending_marker_republish_interval_sec);
        private_nh.param("publish_repeat_count", publish_repeat_count, publish_repeat_count);
        private_nh.param("publish_repeat_interval_sec", publish_repeat_interval_sec, publish_repeat_interval_sec);

        RoadNetwork network;
        network.load_osm(map_file, global_config, use_osm_direction);
        if (regenerate_offline_points || !file_exists(offline_points_file)) {
            save_offline_points(offline_points_file, network, static_cast<std::size_t>(offline_point_count),
                                static_cast<unsigned int>(offline_point_seed));
            ROS_INFO_STREAM("offline point map saved: " << offline_points_file);
        }
        const auto offline_points = load_offline_points(offline_points_file);
        if (regenerate_offline_points || regenerate_distance_table || !file_exists(distance_table_file)) {
            save_offline_distance_table(distance_table_file, network, offline_points);
            ROS_INFO_STREAM("offline point distance table saved: " << distance_table_file);
        }
        OfflineDistanceTable distance_table;
        distance_table.load(distance_table_file);

        const auto vehicles = read_vehicles(vehicles_file, network, offline_points);
        const auto tasks = read_tasks(tasks_file, network, offline_points);
        ros::Publisher task_marker_publisher =
            nh.advertise<visualization_msgs::MarkerArray>("/task_scheduler/task_markers", 1, true);

        if (wait_for_start) {
            if (!wait_for_start_command(nh,
                                        start_topic,
                                        task_marker_publisher,
                                        tasks,
                                        pending_marker_republish_interval_sec)) {
                return 0;
            }
        } else {
            task_marker_publisher.publish(build_pending_task_marker_array(tasks));
        }

        GeneticScheduler scheduler(vehicles, tasks, distance_table, population, generations, static_cast<unsigned int>(seed));
        const Solution solution = scheduler.solve();
        save_solution(output_file, solution, vehicles);

        ROS_INFO_STREAM("task scheduler solution saved: " << output_file);
        ROS_INFO_STREAM("offline point distance table loaded: " << distance_table_file);
        ROS_INFO_STREAM("fitness=" << solution.fitness << ", makespan=" << solution.makespan
                                    << ", total_travel=" << solution.total_travel);

        SolutionPublishers publishers;
        if (publish_result) {
            publishers = publish_solution(nh,
                                          solution,
                                          vehicles,
                                          publish_wait_for_subscribers_sec,
                                          publish_initial_delay_sec,
                                          publish_repeat_count,
                                          publish_repeat_interval_sec);
            if (keep_alive_after_publish) {
                ros::spin();
            }
        }

        ros::spinOnce();
        return 0;
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("task_scheduler failed: " << ex.what());
        return 1;
    }
}
