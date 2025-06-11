// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

module;  // Global module fragment

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <range/v3/all.hpp>  // **** Add this line ****
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_msgs/msg/float32.hpp>
#include <utility>  // **** Add this line ****
#include <vector>

// IMPORTANT: "capsule.hpp", "circle.hpp", "eigen_adapter.hpp" are dependencies
// for boost_geometry.hpp. They must be converted to partitions and imported
// by boost_geometry.hpp. Their #includes are NOT added to this global fragment.

export module crane_basics;

export import :boost_geometry;
export import :ball_info;
export import :geometry_operations;
export import :time;
export import :eigen_adapter;
export import :capsule;
export import :circle;
export import :ball_contact;
export import :ball_model;
// Add other partitions here as they are converted
