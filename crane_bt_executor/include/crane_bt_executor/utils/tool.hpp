//
// Created by hans on 2019/08/22.
//

#pragma once

//-------------include----------------//

#include <utils/eigen_adapter.h>
//------------namespace---------------//
//--------------class-----------------//
namespace geometry_msgs {
    template<class ContainerAllocator>
    struct Pose2D_;
    typedef ::geometry_msgs::Pose2D_<std::allocator<void> > Pose2D;
}

struct Pose2D;
struct BallInfo;


namespace tool {
    Segment getSegment(Point base, Point target);

    Point getPoint(const geometry_msgs::Pose2D &pose);

    geometry_msgs::Pose2D getPose2D(const Point &vec);

    geometry_msgs::Pose2D getPose2D(const Pose2D pose);

    Pose2D getPose2D(const geometry_msgs::Pose2D geo_pose);

    Point getPoint(const Eigen::Vector3f &vec3);

    float getDeg(float angle_rad);

    float getRad(float angle_deg);

    float getAngle(Point vec);

    /**
     * -pi~piの範囲にする
     */
    float normalizeAngle(float angle_rad);

    float getAngleDiff(float angle_rad1, float angle_rad2);

    float getIntermediateAngle(float angle_rad1, float angle_rad2);

    Point getVerticalVec(Point v);

    Point getUnitVec(float theta);

    Point getVec(float length, float theta);

    Point getDirectonNorm(Point base, Point target);

    Point getDirectonVec(Point base, Point target);

    float getReachTime(float distance, float v0, float acc, float max_vel = -1.f);

    Point getBallLineClosestPoint(Point from_pos, const BallInfo &ball);

    bool isInOurDefenseArea(Point pos);

    bool isInTheirDefenseArea(Point pos);
}