//
// Created by hans on 2019/09/25.
//

#pragma once

//-------------include----------------//
#include <utils/boost_geometry.h>
//------------namespace---------------//
//--------------class-----------------//
namespace boost::geometry::traits {
    template<>
    struct tag<Point> {
        typedef point_tag type;
    };
    template<>
    struct coordinate_type<Eigen::Vector2f> {
        typedef float type;
    };

    template<>
    struct coordinate_system<Eigen::Vector2f> {
        typedef cs::cartesian type;
    };

    template<>
    struct dimension<Eigen::Vector2f> : boost::mpl::int_<2> {
    };

    template<>
    struct access<Eigen::Vector2f, 0> {
        static float get(Eigen::Vector2f const &p) {
            return p.x();
        }

        static void set(Eigen::Vector2f &p, float const &value) {
            p.x() = value;
        }
    };

    template<>
    struct access<Eigen::Vector2f, 1> {
        static float get(Eigen::Vector2f const &p) {
            return p.y();
        }

        static void set(Eigen::Vector2f &p, float const &value) {
            p.y() = value;
        }
    };
}