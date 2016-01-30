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

	/*
	//Iterate through control points
	for (int i = 0; i < controlPoints_.size(); i++) {
		std::cout << "Point" << controlPoints_.at(i) << endl;
		std::cout << "XVal" << controlPoints_.at(i)(0) << endl;
		std::cout << "YVal" << controlPoints_.at(i)(1) << endl;
	}

	//controlPoints_.at(0)(0) += 50;
	*/

	double tInc = 0.1;
	
	int i = 3;
	//Size one more than index
	while (controlPoints_.size() > i) {

		BezierInterpolateHelper(interpolatedSpline, i, tInc);

		//Increment requirement for next leg of Bezier Curve
		i += 3;
	}
	
}

void Spline2d::BezierInterpolateHelper(std::vector<Eigen::Vector2d>& interpolatedSpline, int i, double tInc) {
	/*
	double p1X = controlPoints_.at(i - 3)(0);
	double p1Y = controlPoints_.at(i - 3)(1);

	double p2X = controlPoints_.at(i - 2)(0);
	double p2Y = controlPoints_.at(i - 2)(1);

	double p3X = controlPoints_.at(i - 1)(0);
	double p3Y = controlPoints_.at(i - 1)(1);

	double p4X = controlPoints_.at(i)(0);
	double p4Y = controlPoints_.at(i)(1);
	*/

	Eigen::MatrixXd points(4, 2); // (controlPoints_.at(i - 3), controlPoints_.at(i - 2), controlPoints_.at(i - 1), controlPoints_.at(i));
	points << controlPoints_.at(i - 3)(0), controlPoints_.at(i - 3)(1),
		controlPoints_.at(i - 2)(0), controlPoints_.at(i - 2)(1),
		controlPoints_.at(i - 1)(0), controlPoints_.at(i - 1)(1),
		controlPoints_.at(i)(0), controlPoints_.at(i)(1);

	Eigen::MatrixXd mB(4, 4);
	mB << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 3, 0, 0,
		1, 0, 0, 0;

	double t = 0;
	while (t < 1) {

		Eigen::RowVector4d tVec(std::pow(t, 3), std::pow(t, 2), t, 1);

		t += tInc;

		Eigen::RowVector4d bB = tVec * mB;

		Eigen::Vector2d qT = bB * points;

		interpolatedSpline.push_back(qT);
	}
}

void Spline2d::DeBoorInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for deboor interpolation
}

void Spline2d::CatmullInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for catmull-rom interpolation
}






