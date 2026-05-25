#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "VRPH.h"

using namespace std;
namespace fs = std::filesystem;

constexpr double EPS = 1e-9;

struct WalkOption {
  int stop = -1;
  double distance = numeric_limits<double>::infinity();
  double time = numeric_limits<double>::infinity();
};

struct Route {
  vector<int> stops;
  vector<int> loads;
};

struct ProblemData {
  int stop_count = 0;
  int address_count = 0;
  int walk_count = 0;
  int capacity = 0;
  double max_walk_distance = 0;
  double max_journey_time = 0;
  double sec_per_passenger = 0;
  double sec_per_stop = 0;
  vector<int> passengers;
  vector<vector<double>> drive_time;
  vector<vector<WalkOption>> walk_options;

  static bool read(istream& in, ProblemData& data) {
    double max_extra = 0;
    double max_walk = 0;
    double max_journey_minutes = 0;
    if (!(in >> data.stop_count >> data.address_count >> data.walk_count >>
          max_extra >> max_walk >> max_journey_minutes >> data.capacity >>
          data.sec_per_passenger >> data.sec_per_stop)) {
      return false;
    }

    data.max_walk_distance = max_walk;
    data.max_journey_time = max_journey_minutes * 60.0;

    for (int i = 0; i < data.stop_count; i++) {
      double x = 0, y = 0;
      in >> x >> y;
    }

    data.passengers.assign(data.address_count, 0);
    for (int i = 0; i < data.address_count; i++) {
      double x = 0, y = 0;
      in >> x >> y >> data.passengers[i];
    }

    for (int i = 0; i < data.stop_count; i++) {
      for (int j = 0; j < data.stop_count; j++) {
        double ignored_distance = 0;
        in >> ignored_distance;
      }
    }

    data.drive_time.assign(data.stop_count,
                           vector<double>(data.stop_count, 0));
    for (int i = 0; i < data.stop_count; i++) {
      for (int j = 0; j < data.stop_count; j++) {
        in >> data.drive_time[i][j];
      }
    }

    data.walk_options.assign(data.address_count, {});
    for (int address = 0; address < data.address_count; address++) {
      int size = 0;
      in >> size;
      data.walk_options[address].reserve(size);
      for (int k = 0; k < size; k++) {
        WalkOption option;
        in >> option.stop >> option.distance >> option.time;
        if (0 <= option.stop && option.stop < data.stop_count) {
          data.walk_options[address].push_back(option);
        }
      }
    }

    return (bool)in;
  }

  int max_route_load_for_stop(int stop) const {
    if (stop <= 0 || stop >= stop_count) return 0;
    double available =
        max_journey_time - sec_per_stop - drive_time[stop][0];
    int by_time = (int)floor((available + EPS) / sec_per_passenger);
    return max(0, min(capacity, by_time));
  }

  bool can_assign(int address, int stop) const {
    for (const WalkOption& option : walk_options[address]) {
      if (option.stop == stop && option.distance <= max_walk_distance + EPS) {
        return true;
      }
    }
    return false;
  }

  double calculate_route_cost(double route_time, int load) const {
    if (load <= 0) return 0;
    const double vehicle_cost = max(10000000.0, max_journey_time * 10000.0);
    const double time_violation_cost = vehicle_cost * 100000.0;
    if (route_time > max_journey_time + EPS) {
      double violation = route_time - max_journey_time;
      return time_violation_cost + vehicle_cost + route_time +
             violation * vehicle_cost;
    }
    return vehicle_cost + route_time;
  }
};

struct Solution {
  vector<Route> routes;
  vector<int> assignment;
};

struct VirtualNode {
  int stop = 0;
  int load = 0;
  double service_time = 0;
};

struct EvalResult {
  bool valid = false;
  int bus_count = -1;
  double objective = -1.0;
  double solver_objective = -1.0;
  string message;
};

struct RunResult {
  int id = -1;
  bool ok = false;
  double elapsed_seconds = 0;
  fs::path output_path;
  fs::path log_path;
  EvalResult reference;
  EvalResult produced;
  string verdict;
};

struct BatchConfig {
  fs::path input_dir =
      R"(D:\OneDrive-ntdxl\prj\school-bus-routing-sciortino-2022\opt_10s)";
  fs::path out_dir = "sbr_outputs";
  fs::path reference_dir =
      R"(D:\OneDrive-ntdxl\prj\school-bus-routing-sciortino-2022\opt_10s)";
  int first = 0;
  int last = 19;
  double time_limit_seconds = 10.0;
  string out_ext = ".ans";
  string log_ext = ".log";
  string reference_ext = ".out";
  bool force = false;
};

static bool starts_with(const string& s, const string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

static string normalize_ext(string ext) {
  if (!ext.empty() && ext[0] != '.') ext = "." + ext;
  return ext;
}

static bool parse_int(const string& s, int& out) {
  char* end = nullptr;
  long value = strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  out = static_cast<int>(value);
  return true;
}

static bool parse_double(const string& s, double& out) {
  char* end = nullptr;
  double value = strtod(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  out = value;
  return true;
}

static string timestamp_string() {
  time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
  tm local_tm{};
#ifdef _WIN32
  localtime_s(&local_tm, &now);
#else
  localtime_r(&now, &local_tm);
#endif
  stringstream ss;
  ss << put_time(&local_tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

static void print_usage(const char* exe) {
  cerr << "Usage: " << exe
       << " [--dir INPUT_DIR] [--out-dir OUT_DIR]\n"
       << "       [--reference-dir REF_DIR] [--reference-ext .out]\n"
       << "       [--first N] [--last N] [--time-limit SECONDS]\n"
       << "       [--out-ext .ans]\n"
       << "       [--log-ext .log] [--force]\n\n"
       << "Default input DIR is "
       << R"(D:\OneDrive-ntdxl\prj\school-bus-routing-sciortino-2022\opt_10s)"
       << "\n"
       << "Default output DIR is .\\sbr_outputs\\<timestamp> inside the current project.\n"
       << "Default reference DIR is the input DIR, using .out files.\n"
       << "Default output names are 0.ans..19.ans.\n";
}

static bool parse_args(int argc, char** argv, BatchConfig& cfg) {
  for (int i = 1; i < argc; i++) {
    string arg = argv[i];
    if (arg == "--dir" && i + 1 < argc) {
      cfg.input_dir = argv[++i];
      continue;
    }
    if (starts_with(arg, "--dir=")) {
      cfg.input_dir = arg.substr(6);
      continue;
    }
    if (arg == "--out-dir" && i + 1 < argc) {
      cfg.out_dir = argv[++i];
      continue;
    }
    if (starts_with(arg, "--out-dir=")) {
      cfg.out_dir = arg.substr(10);
      continue;
    }
    if (arg == "--reference-dir" && i + 1 < argc) {
      cfg.reference_dir = argv[++i];
      continue;
    }
    if (starts_with(arg, "--reference-dir=")) {
      cfg.reference_dir = arg.substr(16);
      continue;
    }
    if (arg == "--reference-ext" && i + 1 < argc) {
      cfg.reference_ext = normalize_ext(argv[++i]);
      continue;
    }
    if (starts_with(arg, "--reference-ext=")) {
      cfg.reference_ext = normalize_ext(arg.substr(16));
      continue;
    }
    if (arg == "--first" && i + 1 < argc) {
      if (!parse_int(argv[++i], cfg.first)) return false;
      continue;
    }
    if (starts_with(arg, "--first=")) {
      if (!parse_int(arg.substr(8), cfg.first)) return false;
      continue;
    }
    if (arg == "--last" && i + 1 < argc) {
      if (!parse_int(argv[++i], cfg.last)) return false;
      continue;
    }
    if (starts_with(arg, "--last=")) {
      if (!parse_int(arg.substr(7), cfg.last)) return false;
      continue;
    }
    if (arg == "--time-limit" && i + 1 < argc) {
      if (!parse_double(argv[++i], cfg.time_limit_seconds)) return false;
      continue;
    }
    if (starts_with(arg, "--time-limit=")) {
      if (!parse_double(arg.substr(13), cfg.time_limit_seconds)) return false;
      continue;
    }
    if (arg == "--out-ext" && i + 1 < argc) {
      cfg.out_ext = normalize_ext(argv[++i]);
      continue;
    }
    if (starts_with(arg, "--out-ext=")) {
      cfg.out_ext = normalize_ext(arg.substr(10));
      continue;
    }
    if (arg == "--log-ext" && i + 1 < argc) {
      cfg.log_ext = normalize_ext(argv[++i]);
      continue;
    }
    if (starts_with(arg, "--log-ext=")) {
      cfg.log_ext = normalize_ext(arg.substr(10));
      continue;
    }
    if (arg == "--force") {
      cfg.force = true;
      continue;
    }
    cerr << "Unknown option: " << arg << "\n";
    return false;
  }

  cfg.out_ext = normalize_ext(cfg.out_ext);
  cfg.log_ext = normalize_ext(cfg.log_ext);
  cfg.reference_ext = normalize_ext(cfg.reference_ext);
  if (cfg.time_limit_seconds < 0) cfg.time_limit_seconds = 0;
  return true;
}

static double route_time(const ProblemData& data, const Route& route) {
  if (route.stops.empty()) return 0;
  double total = data.drive_time[route.stops.back()][0];
  for (int i = 1; i < (int)route.stops.size(); i++) {
    total += data.drive_time[route.stops[i - 1]][route.stops[i]];
  }
  for (int load : route.loads) {
    total += data.sec_per_stop + data.sec_per_passenger * load;
  }
  return total;
}

static int route_load(const Route& route) {
  int total = 0;
  for (int load : route.loads) total += load;
  return total;
}

static pair<int, double> solution_key(const ProblemData& data,
                                      const vector<Route>& routes) {
  double objective = 0;
  for (const Route& route : routes) objective += route_time(data, route);
  return {(int)routes.size(), objective};
}

static void assign_addresses_to_stops(const ProblemData& data,
                                      vector<int>& assignment,
                                      vector<int>& stop_load, ostream& log) {
  assignment.assign(data.address_count, 0);
  stop_load.assign(data.stop_count, 0);
  for (int address = 0; address < data.address_count; address++) {
    int chosen = -1;
    double best_distance = numeric_limits<double>::infinity();
    for (const WalkOption& option : data.walk_options[address]) {
      if (option.stop <= 0) continue;
      if (option.distance > data.max_walk_distance + EPS) continue;
      if (data.max_route_load_for_stop(option.stop) <= 0) continue;
      if (option.distance < best_distance) {
        best_distance = option.distance;
        chosen = option.stop;
      }
    }

    if (chosen < 0) {
      for (const WalkOption& option : data.walk_options[address]) {
        if (option.distance <= data.max_walk_distance + EPS) {
          chosen = option.stop;
          break;
        }
      }
    }
    if (chosen < 0 && !data.walk_options[address].empty()) {
      chosen = data.walk_options[address][0].stop;
      log << "warning=forced_assignment address=" << address
          << " stop=" << chosen << "\n";
    }
    if (chosen < 0) chosen = 0;

    assignment[address] = chosen;
    if (0 <= chosen && chosen < data.stop_count) {
      stop_load[chosen] += data.passengers[address];
    }
  }
}

static int append_load_limit(const ProblemData& data, const Route& route,
                             int stop) {
  int cap_left = data.capacity - route_load(route);
  if (cap_left <= 0) return 0;

  double base_time = 0;
  if (route.stops.empty()) {
    base_time = data.drive_time[stop][0] + data.sec_per_stop;
  } else {
    int last = route.stops.back();
    base_time = route_time(data, route) - data.drive_time[last][0] +
                data.drive_time[last][stop] + data.drive_time[stop][0] +
                data.sec_per_stop;
  }
  int by_time = (int)floor((data.max_journey_time - base_time + EPS) /
                           data.sec_per_passenger);
  return max(0, min(cap_left, by_time));
}

static vector<Route> build_routes_attempt(const ProblemData& data,
                                          const vector<int>& stop_load,
                                          mt19937& rng, bool randomized) {
  vector<int> remaining = stop_load;
  vector<Route> routes;

  auto has_remaining = [&]() {
    for (int stop = 1; stop < data.stop_count; stop++) {
      if (remaining[stop] > 0) return true;
    }
    return false;
  };

  while (has_remaining()) {
    vector<int> seeds;
    for (int stop = 1; stop < data.stop_count; stop++) {
      if (remaining[stop] > 0 && data.max_route_load_for_stop(stop) > 0) {
        seeds.push_back(stop);
      }
    }
    if (seeds.empty()) {
      for (int stop = 1; stop < data.stop_count; stop++) {
        if (remaining[stop] > 0) seeds.push_back(stop);
      }
    }

    sort(seeds.begin(), seeds.end(), [&](int a, int b) {
      if (remaining[a] != remaining[b]) return remaining[a] > remaining[b];
      return data.drive_time[a][0] > data.drive_time[b][0];
    });

    int seed = seeds.front();
    if (randomized && seeds.size() > 1) {
      int limit = min<int>((int)seeds.size(), 8);
      seed = seeds[uniform_int_distribution<int>(0, limit - 1)(rng)];
    }

    Route route;
    int seed_limit = append_load_limit(data, route, seed);
    if (seed_limit <= 0) seed_limit = data.capacity;
    int seed_load = min(remaining[seed], seed_limit);
    route.stops.push_back(seed);
    route.loads.push_back(seed_load);
    remaining[seed] -= seed_load;

    while (route_load(route) < data.capacity) {
      vector<tuple<double, int, int>> candidates;
      vector<char> used(data.stop_count, 0);
      for (int stop : route.stops) used[stop] = 1;

      for (int stop = 1; stop < data.stop_count; stop++) {
        if (remaining[stop] <= 0 || used[stop]) continue;
        int limit = append_load_limit(data, route, stop);
        if (limit <= 0) continue;
        int load = min(remaining[stop], limit);

        double before = route_time(data, route);
        Route trial = route;
        trial.stops.push_back(stop);
        trial.loads.push_back(load);
        double increase = route_time(data, trial) - before;
        double score = (double)load * 1000.0 - increase;
        candidates.push_back({score, stop, load});
      }

      if (candidates.empty()) break;
      sort(candidates.begin(), candidates.end(),
           [](const auto& a, const auto& b) { return get<0>(a) > get<0>(b); });

      int chosen_index = 0;
      if (randomized && candidates.size() > 1) {
        int limit = min<int>((int)candidates.size(), 8);
        chosen_index = uniform_int_distribution<int>(0, limit - 1)(rng);
      }
      int stop = get<1>(candidates[chosen_index]);
      int load = get<2>(candidates[chosen_index]);
      route.stops.push_back(stop);
      route.loads.push_back(load);
      remaining[stop] -= load;
    }

    routes.push_back(std::move(route));
  }

  return routes;
}

static Solution build_solution_fallback(const ProblemData& data, double time_limit,
                                        ostream& log) {
  Solution best;
  vector<int> stop_load;
  assign_addresses_to_stops(data, best.assignment, stop_load, log);

  if (stop_load[0] > 0) {
    log << "warning=positive_depot_load load=" << stop_load[0] << "\n";
  }

  auto deadline =
      chrono::steady_clock::now() + chrono::duration<double>(time_limit);
  mt19937 rng(1);
  best.routes = build_routes_attempt(data, stop_load, rng, false);
  auto best_key = solution_key(data, best.routes);
  int attempts = 1;
  int improvements = 0;

  while (time_limit > 0 && chrono::steady_clock::now() < deadline) {
    vector<Route> candidate = build_routes_attempt(data, stop_load, rng, true);
    attempts++;
    auto candidate_key = solution_key(data, candidate);
    if (candidate_key.first < best_key.first ||
        (candidate_key.first == best_key.first &&
         candidate_key.second + EPS < best_key.second)) {
      best.routes = std::move(candidate);
      best_key = candidate_key;
      improvements++;
    }
  }

  log << "search_attempts=" << attempts << "\n";
  log << "search_improvements=" << improvements << "\n";
  log << "best_routes=" << best_key.first << "\n";
  log << "best_checker_objective=" << fixed << setprecision(6)
      << best_key.second << "\n";
  return best;
}

static vector<VirtualNode> build_virtual_nodes(const ProblemData& data,
                                               const vector<int>& stop_load,
                                               ostream& log) {
  vector<VirtualNode> nodes;
  for (int stop = 1; stop < data.stop_count; stop++) {
    int remaining = stop_load[stop];
    int max_load = data.max_route_load_for_stop(stop);
    if (max_load <= 0 && remaining > 0) {
      max_load = data.capacity;
      log << "warning=vrph_time_infeasible_single_stop stop=" << stop << "\n";
    }
    while (remaining > 0) {
      int load = min(remaining, max_load);
      nodes.push_back(
          {stop, load, data.sec_per_stop + data.sec_per_passenger * load});
      remaining -= load;
    }
  }
  return nodes;
}

static void write_vrph_instance(const ProblemData& data,
                                const vector<VirtualNode>& nodes,
                                const fs::path& path) {
  ofstream out(path);
  int n = (int)nodes.size();
  out << "NAME: sbr_generated\n";
  out << "TYPE: CVRP\n";
  out << "DIMENSION: " << (n + 1) << "\n";
  out << "CAPACITY: " << data.capacity << "\n";
  out << "DISTANCE: " << fixed << setprecision(6) << data.max_journey_time
      << "\n";
  out << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
  out << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
  out << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i <= n; i++) {
    for (int j = 0; j <= n; j++) {
      double value = 0;
      if (i == 0 && j == 0) {
        value = 0;
      } else if (i == 0) {
        value = 0;
      } else if (j == 0) {
        value = data.drive_time[nodes[i - 1].stop][0];
      } else {
        value = data.drive_time[nodes[i - 1].stop][nodes[j - 1].stop];
      }
      out << fixed << setprecision(6) << value << (j == n ? '\n' : ' ');
    }
  }
  out << "DEMAND_SECTION\n";
  out << "1 0\n";
  for (int i = 0; i < n; i++) out << (i + 2) << " " << nodes[i].load << "\n";
  out << (n + 2) << " 0\n";
  out << "SVC_TIME_SECTION\n";
  out << "1 0\n";
  for (int i = 0; i < n; i++) {
    out << (i + 2) << " " << fixed << setprecision(6)
        << nodes[i].service_time << "\n";
  }
  out << "DEPOT_SECTION\n1\n-1\nEOF\n";
}

static vector<Route> routes_from_vrph_buffer(const vector<int>& sol_buff,
                                             const vector<VirtualNode>& nodes) {
  vector<Route> routes;
  Route current;
  for (int i = 1; i < (int)sol_buff.size(); i++) {
    int token = sol_buff[i];
    if (token == 0) break;
    if (token < 0) {
      if (!current.stops.empty()) routes.push_back(current);
      current = Route{};
      token = -token;
    }
    int node_index = token - 1;
    if (node_index < 1 || node_index > (int)nodes.size()) continue;
    const VirtualNode& vnode = nodes[node_index - 1];
    auto it = find(current.stops.begin(), current.stops.end(), vnode.stop);
    if (it == current.stops.end()) {
      current.stops.push_back(vnode.stop);
      current.loads.push_back(vnode.load);
    } else {
      int pos = (int)(it - current.stops.begin());
      current.loads[pos] += vnode.load;
    }
  }
  if (!current.stops.empty()) routes.push_back(current);
  return routes;
}

static bool solve_with_vrph(const ProblemData& data,
                            const vector<int>& stop_load,
                            const fs::path& work_dir, double time_limit,
                            Solution& solution, ostream& log) {
  vector<VirtualNode> nodes = build_virtual_nodes(data, stop_load, log);
  if (nodes.empty()) return true;

  fs::path vrp_path = work_dir / "vrph_instance.vrp";
  write_vrph_instance(data, nodes, vrp_path);

  VRP V((int)nodes.size());
  V.read_TSPLIB_file(vrp_path.string().c_str());

  vector<double> lambdas = {0.6, 1.4, 1.6};
  ClarkeWright cw((int)nodes.size());
  vector<int> best_sol(nodes.size() + 2, 0);
  double best_obj = numeric_limits<double>::infinity();
  int attempts = 0;
  int improvements = 0;
  auto deadline =
      chrono::steady_clock::now() + chrono::duration<double>(time_limit);

  do {
    double lambda = lambdas[attempts % lambdas.size()];
    if (attempts >= (int)lambdas.size()) {
      lambda = 0.5 + 1.5 * ((attempts * 1103515245u + 12345u) % 100000) /
                            100000.0;
    }
    V.reset();
    cw.Construct(&V, lambda, false);
    cw.has_savings_matrix = false;

    vector<int> candidate(nodes.size() + 2, 0);
    V.export_canonical_solution_buff(candidate.data());
    double objective = V.get_total_route_length();
    if (objective + EPS < best_obj) {
      best_obj = objective;
      V.export_canonical_solution_buff(best_sol.data());
      improvements++;
    }
    V.set_best_total_route_length(VRP_INFINITY);
    attempts++;
  } while (time_limit > 0 && chrono::steady_clock::now() < deadline);

  if (best_sol[0] <= 0) return false;
  solution.routes = routes_from_vrph_buffer(best_sol, nodes);
  for (const Route& route : solution.routes) {
    if (route_load(route) > data.capacity ||
        route_time(data, route) > data.max_journey_time + EPS) {
      log << "warning=vrph_infeasible_route_fallback"
          << " load=" << route_load(route)
          << " time=" << fixed << setprecision(6) << route_time(data, route)
          << "\n";
      return false;
    }
  }
  log << "vrph_attempts=" << attempts << "\n";
  log << "vrph_improvements=" << improvements << "\n";
  log << "vrph_nodes=" << nodes.size() << "\n";
  log << "vrph_objective=" << fixed << setprecision(6) << best_obj << "\n";
  return true;
}

static Solution build_solution(const ProblemData& data, const fs::path& work_dir,
                               double time_limit, ostream& log) {
  auto started = chrono::steady_clock::now();
  Solution solution;
  vector<int> stop_load;
  assign_addresses_to_stops(data, solution.assignment, stop_load, log);
  if (stop_load[0] > 0) {
    log << "warning=positive_depot_load load=" << stop_load[0] << "\n";
  }

  double vrph_budget = min(2.0, max(0.0, time_limit * 0.05));
  if (solve_with_vrph(data, stop_load, work_dir, vrph_budget, solution, log)) {
    return solution;
  }

  log << "warning=vrph_failed_using_fallback\n";
  double elapsed =
      chrono::duration<double>(chrono::steady_clock::now() - started).count();
  double remaining = max(0.0, time_limit - elapsed);
  return build_solution_fallback(data, remaining, log);
}

static bool validate_solution(const ProblemData& data, const Solution& sol,
                              ostream& log) {
  vector<int> route_load(data.stop_count, 0);
  for (const Route& route : sol.routes) {
    if (route.stops.empty() || route.stops.size() != route.loads.size()) {
      log << "error=invalid_route_shape\n";
      return false;
    }
    vector<char> used(data.stop_count, 0);
    int load_sum = 0;
    for (int i = 0; i < (int)route.stops.size(); i++) {
      int stop = route.stops[i];
      int load = route.loads[i];
      if (stop <= 0 || stop >= data.stop_count || used[stop]) {
        log << "error=invalid_route_stop stop=" << stop << "\n";
        return false;
      }
      if (load <= 0) {
        log << "error=invalid_route_load stop=" << stop
            << " load=" << load << "\n";
        return false;
      }
      used[stop] = 1;
      route_load[stop] += load;
      load_sum += load;
    }
    if (load_sum > data.capacity) {
      log << "error=route_capacity load=" << load_sum << "\n";
      return false;
    }
    double time = route_time(data, route);
    if (time > data.max_journey_time + EPS) {
      log << "error=route_time time=" << time << "\n";
      return false;
    }
  }

  vector<int> assigned_load(data.stop_count, 0);
  for (int address = 0; address < data.address_count; address++) {
    int stop = sol.assignment[address];
    if (stop < 0 || stop >= data.stop_count || !data.can_assign(address, stop)) {
      log << "error=invalid_assignment address=" << address
          << " stop=" << stop << "\n";
      return false;
    }
    assigned_load[stop] += data.passengers[address];
  }

  for (int stop = 0; stop < data.stop_count; stop++) {
    if (route_load[stop] != assigned_load[stop]) {
      log << "error=demand_mismatch stop=" << stop
          << " routed=" << route_load[stop]
          << " assigned=" << assigned_load[stop] << "\n";
      return false;
    }
  }
  return true;
}

static void write_solution(ostream& out, const Solution& sol) {
  out << sol.routes.size() << "\n";
  for (const Route& route : sol.routes) {
    out << route.stops.size();
    for (int i = 0; i < (int)route.stops.size(); i++) {
      out << " " << route.stops[i] << " " << route.loads[i];
    }
    out << "\n";
  }
  for (int stop : sol.assignment) out << stop << " ";
}

static EvalResult evaluate_solution(const ProblemData& data, istream& in,
                                    const string& label) {
  EvalResult result;
  int bus_count = 0;
  if (!(in >> bus_count)) {
    result.message = label + " missing bus_count";
    return result;
  }

  double checker_objective = 0;
  double solver_objective = 0;
  vector<int> route_load_by_stop(data.stop_count, 0);
  for (int r = 0; r < bus_count; r++) {
    int route_size = 0;
    if (!(in >> route_size)) {
      result.message = label + " missing route_size";
      return result;
    }
    if (route_size <= 0) {
      result.message = label + " empty route";
      return result;
    }
    vector<int> route(route_size, 0), weight(route_size, 0);
    vector<char> used(data.stop_count, 0);
    int route_load = 0;
    for (int i = 0; i < route_size; i++) {
      if (!(in >> route[i] >> weight[i])) {
        result.message = label + " missing route stop/weight";
        return result;
      }
      if (route[i] <= 0 || route[i] >= data.stop_count) {
        result.message = label + " invalid route stop";
        return result;
      }
      if (used[route[i]]) {
        result.message = label + " duplicated stop in one route";
        return result;
      }
      used[route[i]] = 1;
      route_load_by_stop[route[i]] += weight[i];
      route_load += weight[i];
    }
    if (route_load > data.capacity) {
      result.message = label + " capacity violation";
      return result;
    }

    double route_time = route.empty() ? 0 : data.drive_time[route.back()][0];
    for (int i = 1; i < route_size; i++) {
      route_time += data.drive_time[route[i - 1]][route[i]];
    }
    for (int i = 0; i < route_size; i++) {
      route_time += data.sec_per_stop;
      route_time += data.sec_per_passenger * weight[i];
    }
    if (route_time > data.max_journey_time + EPS) {
      result.message = label + " route time violation";
      return result;
    }
    checker_objective += route_time;
    solver_objective += data.calculate_route_cost(route_time, route_load);
  }

  vector<int> assigned_load_by_stop(data.stop_count, 0);
  for (int address = 0; address < data.address_count; address++) {
    int stop = -1;
    if (!(in >> stop)) {
      result.message = label + " missing assignment";
      return result;
    }
    if (stop < 0 || stop >= data.stop_count || !data.can_assign(address, stop)) {
      result.message = label + " walk violation";
      return result;
    }
    assigned_load_by_stop[stop] += data.passengers[address];
  }

  for (int stop = 0; stop < data.stop_count; stop++) {
    if (route_load_by_stop[stop] != assigned_load_by_stop[stop]) {
      result.message = label + " demand mismatch";
      return result;
    }
  }

  result.valid = true;
  result.bus_count = bus_count;
  result.objective = checker_objective;
  result.solver_objective = solver_objective;
  result.message = "ok";
  return result;
}

static string verdict_from(const EvalResult& reference,
                           const EvalResult& produced) {
  if (!reference.valid) return "invalid_reference";
  if (!produced.valid) return "invalid_output";
  if (produced.bus_count < reference.bus_count ||
      (produced.bus_count == reference.bus_count &&
       produced.objective < reference.objective)) {
    return "better";
  }
  if (produced.bus_count > reference.bus_count ||
      (produced.bus_count == reference.bus_count &&
       produced.objective > reference.objective)) {
    return "worse";
  }
  return "equal";
}

static RunResult run_one(const BatchConfig& cfg, int id) {
  auto started = chrono::steady_clock::now();
  RunResult result;
  result.id = id;

  string base = to_string(id);
  fs::path input_path = cfg.input_dir / (base + ".in");
  fs::path output_path = cfg.out_dir / (base + cfg.out_ext);
  fs::path log_path = cfg.out_dir / (base + cfg.log_ext);
  fs::path reference_path = cfg.reference_dir / (base + cfg.reference_ext);
  result.output_path = output_path;
  result.log_path = log_path;

  if (!fs::exists(input_path)) {
    cerr << "Missing input: " << input_path.string() << "\n";
    result.verdict = "missing_input";
    return result;
  }
  if (cfg.out_ext == ".out" && fs::exists(output_path) && !cfg.force) {
    cerr << "Refusing to overwrite " << output_path.string()
         << ". Add --force if this is intentional.\n";
    result.verdict = "refused_overwrite";
    return result;
  }

  ifstream input(input_path);
  ofstream log(log_path);
  if (!input) {
    cerr << "Cannot open input: " << input_path.string() << "\n";
    result.verdict = "cannot_open_input";
    return result;
  }
  if (!log) {
    cerr << "Cannot open log: " << log_path.string() << "\n";
    result.verdict = "cannot_open_log";
    return result;
  }

  log << "case=" << id << "\n";
  log << "input=" << input_path.string() << "\n";
  log << "output=" << output_path.string() << "\n";
  log << "reference=" << reference_path.string() << "\n";
  log << "time_limit_seconds=" << fixed << setprecision(3)
      << cfg.time_limit_seconds << "\n";

  ProblemData data;
  if (!ProblemData::read(input, data)) {
    cerr << "Failed to parse input: " << input_path.string() << "\n";
    result.verdict = "parse_error";
    return result;
  }

  auto algorithm_started = chrono::steady_clock::now();
  Solution solution = build_solution(data, cfg.out_dir, cfg.time_limit_seconds,
                                     log);
  double algorithm_elapsed =
      chrono::duration<double>(chrono::steady_clock::now() - algorithm_started)
          .count();
  log << "algorithm_elapsed_seconds=" << fixed << setprecision(6)
      << algorithm_elapsed << "\n";
  if (cfg.time_limit_seconds > 0 &&
      algorithm_elapsed > cfg.time_limit_seconds + 0.05) {
    log << "warning=algorithm_time_limit_exceeded\n";
  }
  if (!validate_solution(data, solution, log)) {
    cerr << "Built invalid solution for " << input_path.filename().string()
         << "; see " << log_path.string() << "\n";
    result.verdict = "invalid_constructed_solution";
    return result;
  }

  ofstream output(output_path);
  if (!output) {
    cerr << "Cannot open output: " << output_path.string() << "\n";
    result.verdict = "cannot_open_output";
    return result;
  }
  write_solution(output, solution);
  output.close();

  ifstream produced_stream(output_path);
  result.produced = evaluate_solution(data, produced_stream, "output");

  ifstream reference_stream(reference_path);
  if (reference_stream) {
    result.reference = evaluate_solution(data, reference_stream, "reference");
  } else {
    result.reference.message = "missing reference .out";
  }
  result.verdict = verdict_from(result.reference, result.produced);

  int total_passengers = 0;
  for (int p : data.passengers) total_passengers += p;
  log << "status=ok routes=" << solution.routes.size()
      << " passengers=" << total_passengers << "\n";
  log << "produced_valid=" << result.produced.valid
      << " produced_bus=" << result.produced.bus_count
      << " produced_checker_objective=" << fixed << setprecision(6)
      << result.produced.objective
      << " produced_solver_objective=" << fixed << setprecision(6)
      << result.produced.solver_objective << "\n";
  log << "reference_valid=" << result.reference.valid
      << " reference_bus=" << result.reference.bus_count
      << " reference_checker_objective=" << fixed << setprecision(6)
      << result.reference.objective
      << " reference_solver_objective=" << fixed << setprecision(6)
      << result.reference.solver_objective << "\n";
  log << "verdict=" << result.verdict << "\n";

  cerr << "Solved " << input_path.filename().string() << " -> "
       << output_path.filename().string() << "\n";
  result.elapsed_seconds =
      chrono::duration<double>(chrono::steady_clock::now() - started).count();
  log << "elapsed_seconds=" << fixed << setprecision(6)
      << result.elapsed_seconds << "\n";
  result.ok = result.produced.valid;
  return result;
}

static void write_summary(const fs::path& summary_path,
                          const vector<RunResult>& results,
                          const BatchConfig& cfg, double total_elapsed) {
  ofstream summary(summary_path);
  summary << "input_dir=" << cfg.input_dir.string() << "\n";
  summary << "output_dir=" << cfg.out_dir.string() << "\n";
  summary << "reference_dir=" << cfg.reference_dir.string() << "\n";
  summary << "reference_ext=" << cfg.reference_ext << "\n";
  summary << "case_range=" << cfg.first << ".." << cfg.last << "\n";
  summary << "time_limit_seconds=" << fixed << setprecision(3)
          << cfg.time_limit_seconds << "\n";
  summary << "total_elapsed_seconds=" << fixed << setprecision(6)
          << total_elapsed << "\n\n";

  summary << "case verdict output_bus output_checker_obj output_solver_obj "
             "reference_bus reference_checker_obj reference_solver_obj "
             "elapsed_seconds output_file log_file message\n";

  int ok_count = 0;
  int better_count = 0;
  int equal_count = 0;
  int worse_count = 0;
  int invalid_count = 0;
  for (const RunResult& r : results) {
    if (r.ok) ok_count++;
    if (r.verdict == "better") better_count++;
    else if (r.verdict == "equal") equal_count++;
    else if (r.verdict == "worse") worse_count++;
    else invalid_count++;

    summary << r.id << " " << r.verdict << " "
            << r.produced.bus_count << " " << fixed << setprecision(6)
            << r.produced.objective << " "
            << fixed << setprecision(6) << r.produced.solver_objective << " "
            << r.reference.bus_count << " " << fixed << setprecision(6)
            << r.reference.objective << " "
            << fixed << setprecision(6) << r.reference.solver_objective << " "
            << fixed << setprecision(6) << r.elapsed_seconds << " "
            << r.output_path.filename().string() << " "
            << r.log_path.filename().string() << " ";
    if (!r.produced.valid) summary << r.produced.message;
    else if (!r.reference.valid) summary << r.reference.message;
    else summary << "ok";
    summary << "\n";
  }

  summary << "\n";
  summary << "ok_count=" << ok_count << "\n";
  summary << "better_count=" << better_count << "\n";
  summary << "equal_count=" << equal_count << "\n";
  summary << "worse_count=" << worse_count << "\n";
  summary << "invalid_or_missing_count=" << invalid_count << "\n";
}

int main(int argc, char** argv) {
  ios::sync_with_stdio(false);
  cin.tie(nullptr);

  for (int i = 1; i < argc; i++) {
    string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  BatchConfig cfg;
  if (!parse_args(argc, argv, cfg)) return 1;

  if (!fs::is_directory(cfg.input_dir)) {
    cerr << "Input directory does not exist: " << cfg.input_dir.string()
         << "\n";
    return 1;
  }
  if (!fs::is_directory(cfg.reference_dir)) {
    cerr << "Reference directory does not exist: "
         << cfg.reference_dir.string() << "\n";
    return 1;
  }
  if (cfg.first > cfg.last) {
    cerr << "--first must be <= --last\n";
    return 1;
  }

  cfg.out_dir /= timestamp_string();
  fs::create_directories(cfg.out_dir);

  auto batch_started = chrono::steady_clock::now();
  vector<RunResult> results;
  results.reserve(max(0, cfg.last - cfg.first + 1));
  int failed = 0;
  for (int id = cfg.first; id <= cfg.last; id++) {
    RunResult result = run_one(cfg, id);
    if (!result.ok) failed++;
    results.push_back(result);
  }

  double total_elapsed =
      chrono::duration<double>(chrono::steady_clock::now() - batch_started)
          .count();
  fs::path summary_path = cfg.out_dir / "summary.log";
  write_summary(summary_path, results, cfg, total_elapsed);

  if (failed) {
    cerr << failed << " case(s) failed.\n";
    cerr << "Summary: " << summary_path.string() << "\n";
    return 1;
  }
  cerr << "Done.\n";
  cerr << "Output directory: " << cfg.out_dir.string() << "\n";
  cerr << "Summary: " << summary_path.string() << "\n";
  return 0;
}
