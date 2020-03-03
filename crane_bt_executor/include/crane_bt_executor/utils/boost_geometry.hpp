//
// Created by hans on 2019/09/26.
//

#pragma once

//-------------include----------------//
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <Eigen/Core>

//------------namespace---------------//
//--------------class-----------------//
namespace bg = boost::geometry;
using Point = Eigen::Vector2f;
using Velocity = Eigen::Vector2f;
using Accel = Eigen::Vector2f;
using Segment = bg::model::segment<Point>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
using ClosestPoint = bg::closest_point_result<Point>;

struct Circle{
    Point center;
    float radius;
};

namespace boost::geometry{
    template <typename Geometry1>
    float distance(const Circle &circle,const Geometry1 &geometry1){
        float dist = distance(circle.center,geometry1) - circle.radius;
        if(dist < 0){
            return 0.0f;
        }
        return dist;
    }
}


