#include "spline.h"
#include <fstream>
#include <iostream>

using namespace std;

Spline2d::Spline2d() {
    controlPoints_.clear();
    type_ = Spline2d::LINEAR;
}

void Spline2d::AddPoint(const Eigen::Vector2d& pt) {
    controlPoints_.push_back(pt);
}

void Spline2d::RemoveLastPoint() {
    if (controlPoints_.size() != 0) {
        controlPoints_.erase(controlPoints_.end()-1);
    }
}

void Spline2d::RemoveAll() {
    controlPoints_.clear();
}

void Spline2d::SetSplineType(Spline2d::SplineType type) {
    type_ = type;
}

std::vector<Eigen::Vector2d> Spline2d::GetInterpolatedSpline() {
	std::vector<Eigen::Vector2d> interpolatedSpline;
    interpolatedSpline.clear();
    
    if (type_ == Spline2d::LINEAR) {
        LinearInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::BEZIER) {
        BezierInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::DEBOOR) {
        DeBoorInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::CATMULL) {
        CatmullInterpolate(interpolatedSpline);
    }
    
    return interpolatedSpline;
}

std::vector<Eigen::Vector2d> Spline2d::GetControlPoints() {
    return controlPoints_;
}

void Spline2d::LinearInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    interpolatedSpline = controlPoints_;
}

void Spline2d::BezierInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for bezier interpolation
}

void Spline2d::DeBoorInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for deboor interpolation
}

void Spline2d::CatmullInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for catmull-rom interpolation
}






