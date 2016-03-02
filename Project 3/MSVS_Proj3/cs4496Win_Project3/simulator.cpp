#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(2);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    mTimeStep = 0.0003;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
    mParticles[0].mPosition[1] = 0.0;
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    for (int i = 0; i < 2; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    
}

double Simulator::getLambda(int particleNum) {
	double mass = mParticles[particleNum].mMass;
	Eigen::Vector3d velocity = mParticles[particleNum].mVelocity;
	Eigen::Vector3d position = mParticles[particleNum].mPosition;
	Eigen::Vector3d force = mParticles[particleNum].mAccumulatedForce;

	double firstNum = - position.dot(force) - (mass * velocity).dot(velocity);

	return firstNum / (position.dot(position));
}

Eigen::Vector3d Simulator::getConstraintForce(int particleNum, double lambda) {
	Eigen::Vector3d position = mParticles[particleNum].mPosition;

	return lambda * position;
}

Eigen::Vector3d Simulator::getLegalAcceleration(int particleNum, Eigen::Vector3d constraintForce) {
	Eigen::Vector3d force = mParticles[particleNum].mAccumulatedForce;
	double mass = mParticles[particleNum].mMass;

	return (constraintForce + force) / mass;
}

void Simulator::simulate() {
    // TODO:
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }

	//Going to implement constraint force on first particle
	double lambda = getLambda(0);
	//std::cout << "Testing Lambda: " << lambda << endl;

	Eigen::Vector3d constraintForce = getConstraintForce(0, lambda);
	//std::cout << "Testing constraintForce: " << constraintForce << endl;

	Eigen::Vector3d legalAcceleration = getLegalAcceleration(0, constraintForce);
	//std::cout << "Testing legalAcceleration: " << legalAcceleration << endl;

	//mParticles[0].mVelocity += legalAcceleration;
	mParticles[0].mAccumulatedForce += constraintForce;
	mParticles[1].mAccumulatedForce += constraintForce;

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep;
        mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;
    }
    
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}







