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

void Simulator::particleSim() {

	Eigen::MatrixXd qDot(3 * mParticles.size(), 1);
	Eigen::MatrixXd Q(3 * mParticles.size(), 1);
	Eigen::MatrixXd W(3 * mParticles.size(), 3 * mParticles.size());

	qDot.setZero();
	Q.setZero();
	W.setZero();

	for (int p = 0; p < mParticles.size(); p++) {
		//Velocity Setting
		qDot(p * 3, 0) = mParticles[p].mVelocity[0]; //Insert xVel
		qDot(p * 3 + 1, 0) = mParticles[p].mVelocity[1]; //Insert yVel
		qDot(p * 3 + 2, 0) = mParticles[p].mVelocity[2]; //Insert zVel

		//Force Setting
		Q(p * 3, 0) = mParticles[p].mAccumulatedForce[0];
		Q(p * 3 + 1, 0) = mParticles[p].mAccumulatedForce[1];
		Q(p * 3 + 2, 0) = mParticles[p].mAccumulatedForce[2];


		//Mass Settings
		W(p * 3, p * 3) = 1.0 / mParticles[p].mMass;
		W(p * 3 + 1, p * 3 + 1) = 1.0 / mParticles[p].mMass;
		W(p * 3 + 2, p * 3 + 2) = 1.0 / mParticles[p].mMass;
	}

	//Matrices are (Rows, Columnms)
	//Eigen::MatrixXd Lambda(2, 1);
	Eigen::MatrixXd jacobi(2, 3 * mParticles.size());
	Eigen::MatrixXd jacobiPrime(2, 3 * mParticles.size());

	//Lambda.setZero();
	jacobi.setZero();
	jacobiPrime.setZero();

	//Set first constraint
	for (int i = 0; i < 3; i++) {
		jacobi(0, i) = mParticles[0].mPosition[i]; //Believe Jacobian is derivative wrt particlePos - C1 = x
		jacobiPrime(0, i) = mParticles[0].mVelocity[i];
	}

	//C2 Constraint

	//P1 - P2
	jacobi(1, 0) = mParticles[0].mPosition[0] - mParticles[1].mPosition[0]; 
	jacobi(1, 1) = mParticles[0].mPosition[1] - mParticles[1].mPosition[1];
	jacobi(1, 2) = mParticles[0].mPosition[2] - mParticles[1].mPosition[2];

	//jacobiPrime(1, 0) = jacobi(1, 0) * mParticles[0].mVelocity[0];
	//jacobiPrime(1, 1) = jacobi(1, 1) * mParticles[0].mVelocity[1];
	//jacobiPrime(1, 2) = jacobi(1, 2) * mParticles[0].mVelocity[2];

	jacobiPrime(1, 0) = mParticles[0].mVelocity[0] - mParticles[1].mVelocity[0];
	jacobiPrime(1, 1) = mParticles[0].mVelocity[1] - mParticles[1].mVelocity[1];
	jacobiPrime(1, 2) = mParticles[0].mVelocity[2] - mParticles[1].mVelocity[2];

	//P2 - P1
	jacobi(1, 3) = mParticles[1].mPosition[0] - mParticles[0].mPosition[0];
	jacobi(1, 4) = mParticles[1].mPosition[1] - mParticles[0].mPosition[1];
	jacobi(1, 5) = mParticles[1].mPosition[2] - mParticles[0].mPosition[2];
	
	//jacobiPrime(1, 3) = jacobi(1, 3) * mParticles[1].mVelocity[0];
	//jacobiPrime(1, 4) = jacobi(1, 4) * mParticles[1].mVelocity[1];
	//jacobiPrime(1, 5) = jacobi(1, 5) * mParticles[1].mVelocity[2];

	jacobiPrime(1, 3) = mParticles[1].mVelocity[0] - mParticles[0].mVelocity[0];
	jacobiPrime(1, 4) = mParticles[1].mVelocity[1] - mParticles[0].mVelocity[1];
	jacobiPrime(1, 5) = mParticles[1].mVelocity[2] - mParticles[0].mVelocity[2];

	Eigen::MatrixXd jacobiT = jacobi.transpose();

	Eigen::MatrixXd firstNum = (jacobi * W) * jacobiT;

	//Don't know if I messed up or not, but these have to be 2x1 matrices to be used in the below equation. That's why I'm doing weird norms.
	double ks = 0.000000001;
	Eigen::MatrixXd dampingC(2, 1);
	dampingC(0, 0) = (mParticles[0].mPosition - mParticles[1].mPosition).norm();
	dampingC(1, 0) = (mParticles[1].mPosition - mParticles[0].mPosition).norm();
	dampingC *= ks;

	double kd = 0.000000001;
	Eigen::MatrixXd dampingCPrime(2, 1);
	dampingCPrime(0, 0) = (mParticles[0].mVelocity - mParticles[1].mVelocity).norm();
	dampingCPrime(1, 0) = (mParticles[1].mVelocity - mParticles[0].mVelocity).norm();
	dampingCPrime *= kd;

	Eigen::MatrixXd secNum = -1.0 * jacobiPrime * qDot - jacobi * W * Q;

	//std::cout << "DEBUG: secNum R, C" << secNum.rows() << ", " << secNum.cols() << endl;

	Eigen::MatrixXd Lambda = (firstNum).ldlt().solve(secNum);

	Eigen::MatrixXd constraintForce = jacobiT * Lambda;

	for (int part = 0; part < mParticles.size(); part++) {
		mParticles[part].mAccumulatedForce[0] += constraintForce(part * 3 + 0, 0);
		mParticles[part].mAccumulatedForce[1] += constraintForce(part * 3 + 1, 0);
		//mParticles[part].mAccumulatedForce[2] += constraintForce(part * 3 + 2, 0);
	}
}

void Simulator::simulate() {
    // TODO:
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }

	//Going to implement constraint force on first particle
	//double lambda = getLambda(0);
	//Eigen::Vector3d constraintForce = getConstraintForce(0, lambda);
	//mParticles[0].mAccumulatedForce += constraintForce;
	//std::cout << "DEBUG: Do logs print? " << endl;

	particleSim();

	//std::cout << "DEBUG: Back in simulate()" << endl;

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep;
        mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;
    }
    
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}







