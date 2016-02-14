#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(3);
	startingHeight = 20.0;
	startingVelocity = 0.0;
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = startingHeight;
	mParticles[0].mVelocity(1) = startingVelocity;

    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = startingHeight;
	mParticles[1].mVelocity(1) = startingVelocity;

    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = startingHeight;
	mParticles[2].mVelocity(1) = startingVelocity;
    
    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    
    mTimeStep = 0.03;
    mElapsedTime = 0;
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
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = startingHeight;
	mParticles[0].mVelocity(1) = startingVelocity;

    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = startingHeight;
	mParticles[1].mVelocity(1) = startingVelocity;

    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = startingHeight;
	mParticles[2].mVelocity(1) = startingVelocity;
    
    for (int i = 0; i < 3; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }

	mParticles[1].mAccumulatedForce[1] = mParticles[1].mMass * -9.8;
	mParticles[2].mAccumulatedForce[1] = mParticles[1].mMass * -9.8;
    
    mElapsedTime = 0;

}

double Simulator::analyticalStep() {
	return -0.5 * 9.8 * (mElapsedTime * mElapsedTime) + startingHeight;
}

double Simulator::explicitEulerStep() {
	double initialX = mParticles[1].mPosition[0];
	double initialY = mParticles[1].mPosition[1];

	mParticles[1].mVelocity[1] += (-9.8 * mTimeStep);

	return (mParticles[1].mVelocity[1] * mTimeStep);

}

void Simulator::simulate() {
    // TODO: Replace the following code
    //for (int i = 0; i < mParticles.size(); i++) {
    //    mParticles[i].mPosition[1] -= 0.005;
    //}

	mParticles[0].mPosition[1] = analyticalStep();
	mParticles[1].mPosition[1] += explicitEulerStep();
    
    mElapsedTime += mTimeStep;
}







