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

	if(controlPoints_.size() > 0) controlPoints_.push_back(controlPoints_.front());
    
    if (type_ == Spline2d::LINEAR) {
        LinearInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::BEZIER) {
        BezierInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::DEBOOR) {
        DeBoorInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::CATMULL) {
        CatmullInterpolate(interpolatedSpline);
    }

	if(controlPoints_.size() > 0) controlPoints_.pop_back();
    
    return interpolatedSpline;
}

std::vector<Eigen::Vector2d> Spline2d::GetControlPoints() {
    return controlPoints_;
}

void Spline2d::LinearInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    interpolatedSpline = controlPoints_;
}

void Spline2d::BezierInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {

	double tInc = 0.05;
	
	int i = 3;
	//Size one more than index
	while (controlPoints_.size() > i) {

		BezierInterpolateHelper(interpolatedSpline, i, tInc);

		//Increment requirement for next leg of Bezier Curve
		i += 3;
	}
	
}

void Spline2d::BezierInterpolateHelper(std::vector<Eigen::Vector2d>& interpolatedSpline, int i, double tInc) {

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

		Eigen::RowVector4d bB = tVec * mB;

		Eigen::Vector2d qT = bB * points;

		interpolatedSpline.push_back(qT);

		t += tInc;
	}
}

void Spline2d::DeBoorInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {

	double tInc = 0.05;

	int i = 3;
	while (i < controlPoints_.size()) {
		
		DeBoorInterpolateHelper(interpolatedSpline, i, tInc);

		//Check to make sure this is incrementing before the while loop is checked
		i++;
	}
}

void Spline2d::DeBoorInterpolateHelper(std::vector<Eigen::Vector2d>& interpolatedSpline, int i, double tInc) {

	//Couldn't get above to work, so going to calculate Bezier points and then use Bezier alg as accepted on Piazza

	Eigen::Vector2d b0(controlPoints_.at(i - 3)(0), controlPoints_.at(i - 3)(1));
	Eigen::Vector2d b1(controlPoints_.at(i - 2)(0), controlPoints_.at(i - 2)(1));
	Eigen::Vector2d b2(controlPoints_.at(i - 1)(0), controlPoints_.at(i - 1)(1));
	Eigen::Vector2d b3(controlPoints_.at(i)(0), controlPoints_.at(i)(1));

	Eigen::Vector2d v0 = ((double)1 / 2)*(b0 + ( ((double)2 / 3)*(b1 - b0) ) + b1 + ( ((double)1 / 3)*(b2 - b1) ) );
	Eigen::Vector2d v1 = b1 + (((double)1 / 3) * (b2 - b1));
	Eigen::Vector2d v2 = b1 + (((double)2 / 3) * (b2 - b1));
	Eigen::Vector2d v3 = ((double)1 / 2) * (b1 + (((double)2 / 3) *(b2 - b1)) + b2 + (((double)1 / 3) * (b3 - b2)));

	Eigen::MatrixXd points(4, 2); // (controlPoints_.at(i - 3), controlPoints_.at(i - 2), controlPoints_.at(i - 1), controlPoints_.at(i));
	points << v0(0), v0(1),
		v1(0), v1(1),
		v2(0), v2(1),
		v3(0), v3(1);

	Eigen::MatrixXd mB(4, 4);
	mB << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 3, 0, 0,
		1, 0, 0, 0;

	double t = 0;
	while (t < 1) {
		Eigen::RowVector4d tVec(std::pow(t, 3), std::pow(t, 2), t, 1);

		Eigen::RowVector4d bB = tVec * mB;

		Eigen::Vector2d qT = bB * points;

		interpolatedSpline.push_back(qT);

		t += tInc;
	}
}

void Spline2d::CatmullInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for catmull-rom interpolation

	double tInc = 0.05;

	int i = 3;
	while (i < controlPoints_.size()) {

		CatmullInterpolateHelper(interpolatedSpline, i, tInc);

		//Check to make sure this is incrementing before the while loop is checked
		i++;
	}
}

void Spline2d::CatmullInterpolateHelper(std::vector<Eigen::Vector2d>& interpolatedSpline, int i, double tInc) {

	Eigen::MatrixXd points(4, 2); // (controlPoints_.at(i - 3), controlPoints_.at(i - 2), controlPoints_.at(i - 1), controlPoints_.at(i));
	points << controlPoints_.at(i - 3)(0), controlPoints_.at(i - 3)(1),
		controlPoints_.at(i - 2)(0), controlPoints_.at(i - 2)(1),
		controlPoints_.at(i - 1)(0), controlPoints_.at(i - 1)(1),
		controlPoints_.at(i)(0), controlPoints_.at(i)(1);

	Eigen::MatrixXd mB(4, 4);
	mB << -1, 3, -3, 1,
		2, -5, 4, -1,
		-1, 0, 1, 0,
		0, 2, 0, 0;

	mB *= ((double) 1 / 2);

	double t = 0;
	while (t < 1) {

		Eigen::RowVector4d tVec(std::pow(t, 3), std::pow(t, 2), t, 1);

		Eigen::MatrixXd interim = mB * points;

		Eigen::Vector2d qT = tVec * interim;

		interpolatedSpline.push_back(qT);

		t += tInc;
	}
}






