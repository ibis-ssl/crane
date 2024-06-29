// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__POSITION_ASSIGNMENTS_HPP_
#define CRANE_BASICS__POSITION_ASSIGNMENTS_HPP_

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "boost_geometry.hpp"

namespace crane
{
namespace math
{
// ref : https://kopricky.github.io/code/NetworkFlow/hungarian.html

template <typename T>
class Hungarian
{
private:
  const int U, V;
  std::vector<std::vector<int>> graph;
  std::vector<T> dual;
  std::vector<int> alloc, rev_alloc, prev;
  const std::vector<std::vector<T>> & cost;
  int matching_size;
  T diff(const int i, const int j) { return cost[i][j] - dual[i] - dual[U + j]; }
  void init_feasible_dual()
  {
    for (int i = 0; i < U; ++i) {
      dual[i] = 0;
      for (int j = 0; j < V; ++j) {
        dual[U + j] = std::min(dual[U + j], cost[i][j]);
      }
    }
  }
  void construct_graph()
  {
    for (int i = 0; i < U; ++i) {
      for (int j = 0; j < V; ++j) {
        graph[i][j] = (diff(i, j) == 0 && rev_alloc[j] != i);
      }
    }
  }
  bool find_augmenting_path(const int cur, const int prv, int & pos)
  {
    prev[cur] = prv;
    if (cur >= U) {
      if (rev_alloc[cur - U] < 0) return true;
      if (find_augmenting_path(rev_alloc[cur - U], cur, pos)) {
        graph[rev_alloc[cur - U]][cur - U] = 1;
        return true;
      }
    } else {
      const int MX = (alloc[cur] < 0 && pos == U) ? U : V;
      for (int i = 0; i < MX; ++i) {
        if (graph[cur][i] && prev[U + i] < 0 && find_augmenting_path(U + i, cur, pos)) {
          graph[cur][i] = 0, alloc[cur] = i, rev_alloc[i] = cur;
          return true;
        }
      }
      if (alloc[cur] < 0 && pos < U) {
        graph[cur][pos] = 0, alloc[cur] = pos, rev_alloc[pos] = cur, prev[U + pos] = cur;
        return ++pos, true;
      }
    }
    return false;
  }
  void update_dual(const T delta)
  {
    for (int i = 0; i < U; ++i)
      if (prev[i] >= 0) dual[i] += delta;
    for (int i = U; i < U + V; ++i)
      if (prev[i] >= 0) dual[i] -= delta;
  }
  void maximum_matching(bool initial = false)
  {
    int pos = initial ? V : U;
    for (bool update = false;; update = false) {
      fill(prev.begin(), prev.end(), -1);
      for (int i = 0; i < U; ++i) {
        if (alloc[i] < 0 && find_augmenting_path(i, 2 * U, pos)) {
          update = true, ++matching_size;
          break;
        }
      }
      if (!update) break;
    }
  }
  int dfs(const int cur, const int prv, std::vector<int> & new_ver)
  {
    prev[cur] = prv;
    if (cur >= U) {
      if (rev_alloc[cur - U] < 0)
        return cur;
      else
        return dfs(rev_alloc[cur - U], cur, new_ver);
    } else {
      new_ver.push_back(cur);
      for (int i = 0; i < V; ++i) {
        if (graph[cur][i] && prev[U + i] < 0) {
          const int res = dfs(U + i, cur, new_ver);
          if (res >= U) return res;
        }
      }
    }
    return -1;
  }
  int increase_matching(const std::vector<std::pair<int, int>> & vec, std::vector<int> & new_ver)
  {
    for (const auto & e : vec) {
      if (prev[e.first] < 0) {
        const int res = dfs(e.first, e.second, new_ver);
        if (res >= U) return res;
      }
    }
    return -1;
  }
  void hint_increment(int cur)
  {
    while (prev[cur] != 2 * U) {
      if (cur >= U) {
        graph[prev[cur]][cur - U] = 0, alloc[prev[cur]] = cur - U, rev_alloc[cur - U] = prev[cur];
      } else {
        graph[cur][prev[cur] - U] = 1;
      }
      cur = prev[cur];
    }
  }

public:
  explicit Hungarian(const std::vector<std::vector<T>> & _cost)
  : U((int)_cost.size()),
    V((int)_cost[0].size()),
    graph(U, std::vector<int>(U, 1)),
    dual(U + V, std::numeric_limits<T>::max()),
    alloc(U, -1),
    rev_alloc(U, -1),
    prev(2 * U),
    cost{_cost},
    matching_size(0)
  {
    assert(U >= V);
  }
  std::pair<T, std::vector<int>> solve()
  {
    init_feasible_dual(), construct_graph();
    bool end = false;
    maximum_matching(true);
    while (matching_size < U) {
      std::vector<std::pair<T, int>> cand(
        V, {std::numeric_limits<T>::max(), std::numeric_limits<int>::max()});
      for (int i = 0; i < U; ++i) {
        if (prev[i] < 0) continue;
        for (int j = 0; j < V; ++j) {
          if (prev[U + j] >= 0) continue;
          cand[j] = min(cand[j], {diff(i, j), i});
        }
      }
      while (true) {
        T delta = std::numeric_limits<T>::max();
        for (int i = 0; i < V; ++i) {
          if (prev[U + i] >= 0) continue;
          delta = std::min(delta, cand[i].first);
        }
        update_dual(delta);
        std::vector<std::pair<int, int>> vec;
        std::vector<int> new_ver;
        for (int i = 0; i < V; ++i) {
          if (prev[U + i] >= 0) continue;
          if ((cand[i].first -= delta) == 0) vec.emplace_back(U + i, cand[i].second);
        }
        int res = increase_matching(vec, new_ver);
        if (res >= U) {
          hint_increment(res);
          if (++matching_size == U)
            end = true;
          else
            construct_graph();
          break;
        } else {
          for (const int v : new_ver) {
            for (int i = 0; i < V; ++i) {
              if (prev[U + i] >= 0) continue;
              cand[i] = min(cand[i], {diff(v, i), v});
            }
          }
        }
      }
      if (!end) maximum_matching();
    }
    T total_cost = 0;
    for (int i = 0; i < U; ++i) {
      if (alloc[i] < V)
        total_cost += cost[i][alloc[i]];
      else
        alloc[i] = -1;
    }
    return std::make_pair(total_cost, alloc);
  }
};
}  // namespace math

inline std::vector<int> getOptimalAssignments(
  const std::vector<Point> robot_positions, const std::vector<Point> target_positions)
{
  assert(robot_positions.size() == target_positions.size());
  if (robot_positions.size() == 0) {
    return {};
  }
  if (robot_positions.size() == 1) {
    return {0};
  }

  // make cost
  std::vector<std::vector<double>> cost;
  for (const auto & robot_pos : robot_positions) {
    std::vector<double> square_distances;
    for (const auto & target_pos : target_positions) {
      square_distances.emplace_back((target_pos - robot_pos).squaredNorm());
    }
    cost.emplace_back(square_distances);
  }

  math::Hungarian<double> hungarian_solver(cost);
  auto solution = hungarian_solver.solve();

  return solution.second;
}

}  // namespace crane
#endif  // CRANE_BASICS__POSITION_ASSIGNMENTS_HPP_
