    #include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>

using namespace Eigen;
using namespace std;

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;      
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();
    
    // Create and intialize two default rigid bodies
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -0.3;
    rb1->mPosition[1] = -0.5;
    
    rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
}

void MyWorld::initializePinata() {
    // Add pinata to the collison detector
    mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));
    
    // Add some damping in the Pinata joints
    int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
    for (int i = 0; i < nJoints; i++) {
        int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
        for (int j = 0; j < nDofs; j++)
        mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
    }
    
    // Weld two seems to make a box
    dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
    dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
    dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
    dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);
    dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);
    mPinataWorld->getConstraintSolver()->addConstraint(joint1);
    mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
    delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
    delete mCollisionDetector;
}

void MyWorld::simulate() {
    mFrame++;
    
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum.
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;
        
        //****Integration of Orientation****
        //Convert quaternion to rotation matrix
        mRigidBodies[i] -> mOrientation = mRigidBodies[i] -> mQuatOrient.toRotationMatrix();
        
        //HAMYChange - Debug
        //std::cout << "Rotation: " <<    mRigidBodies[i] -> mOrientation << std::endl; 
        //std::cout << "mShape: " << mRigidBodies[i]->mShape << endl;
        //std::cout << "iBody: " << i << " -> " << mRigidBodies[i]->iBody << endl;

        //    I(t) = R(t) * Ibody * transpose(R(t))
        Eigen::Matrix3d mOrientTrans = mRigidBodies[i]->mOrientation.transpose();

        //std::cout << "Debug: " << "rigidBody: " << i << ", OrientationTranspose = " << mOrientTrans << std::endl;

        /*
        //Check to make sure vectors are same size
        std::cout << "Size Debug: " << "mOrientation - " << "Rows = " << mRigidBodies[i]->mOrientation.rows()
                    << ", Cols = " << mRigidBodies[i]->mOrientation.cols() << std::endl;
        std::cout << "Size Debug: " << "mOrientTrans - " << "Rows = " << mOrientTrans.rows()
                    << ", Cols = " << mOrientTrans.cols() << std::endl;
        std::cout << "Size Debug: " << "iBody - " << "Rows = " << mRigidBodies[i]->iBody.rows() 
                    << ", Cols = " << mRigidBodies[i]->iBody.cols() << std::endl;
        */

        Eigen::Matrix3d myI = mRigidBodies[i]->mOrientation * mRigidBodies[i]->iBody * mOrientTrans;

        std::cout << "Debug: " << "I(t) = " << myI << std::endl;

        //Omega = w(t) = I(t)^-1 * L(t)
        Eigen::Vector3d omega = myI.inverse() * mRigidBodies[i]->mAngMomentum;

        //Think I can add the half multiplication here
        Eigen::Quaterniond omega2 = Eigen::Quaterniond(0, 0.5 * omega(0), 0.5 * omega(1), 0.5 * omega(2));
        /*omega2(0) = 0;
        omega2(1) = omega(0);
        omega2(2) = omega(1);
        omega2(3) = omega(2);*/
        // << 0, omega(0), omega(1), omega(2);

        std::cout << "Debug: " << "omega = " << omega << std::endl;

        /*
        //Check dQuat sizes
        std::cout << "Size Debug: " << "mQuatOrient - " << "Size = " << mRigidBodies[i]->mQuatOrient.size()
                  << std::endl;
        std::cout << "Size Debug: " << "omega - " << "Rows = " << omega.rows()
                    << ", Cols = " << omega.cols() << std::endl;
        */

        //dQuat
        Eigen::Quaterniond dQuat = omega2 * mRigidBodies[i]->mQuatOrient;
        //std::cout << "Debug: " << "dQuat = " << dQuat << std::endl;

        //****Integration of angular momentum****
        //Eigen::Vector3d andMomentum = myI * omega;
        Eigen::Vector3d dAngMoment = mRigidBodies[i]->mAccumulatedTorque;

        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;

        //HAMYChange - Looks fishy
        //Update orientation

        Eigen::Vector3d qNew = mRigidBodies[i]->mQuatOrient + mTimeStep * dQuat;

        //dQuat.w() *= mTimeStep;
        //mRigidBodies[i]->mQuatOrient += dQuat;

        //Update angular momentum
        mRigidBodies[i]->mAngMomentum += dAngMoment * mTimeStep;
    }
    
    // Reset accumulated force and torque to be zero after a complete integration
    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mAccumulatedForce.setZero();
        mRigidBodies[i]->mAccumulatedTorque.setZero();
    }
    
    // Apply external force to the pinata
    mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
    mForce.setZero();
    
    // Simulate Pinata using DART
    mPinataWorld->step();
    
    // Run collision detector
    mCollisionDetector->checkCollision();
    
    // TODO: implement a collision handler
    collisionHandling();
    
    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;
    
    // TODO: handle the collision events
}









