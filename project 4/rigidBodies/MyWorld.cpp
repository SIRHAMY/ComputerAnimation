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

//Add more candy to the world
bool MyWorld::makeCandy() {
    RigidBody *newCandy = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(newCandy, "box");
    newCandy->mAngMomentum = Vector3d(0.01, 0, 0);
    
    double color = (double) rand() / (double) RAND_MAX;
    newCandy->mColor = Vector4d(color, color, color, color);

    bool collisionFound = true;
    while(collisionFound) {
        double x = (0 - -0.5) * ( (double) rand() / (double) RAND_MAX) + -1;
        double y = (0.5 - 0) * ( (double) rand() / (double) RAND_MAX) + -1;

        newCandy->mPosition[0] = x;
        newCandy->mPosition[1] = y;

        collisionFound = false;

        //Check for collisions
        mCollisionDetector->checkCollision();

         // Iterates through collisions
        int nContacts = mCollisionDetector->getNumContacts();
        for(int collision = 0; collision<nContacts; collision++) {
            //HAMYChange - Really not sure if these pointers are right
            if( newCandy == mCollisionDetector->getContact(collision).rb1 ||
                newCandy == mCollisionDetector->getContact(collision).rb2) {
                collisionFound = true;
                break;
            }
        }
    }

     mRigidBodies.push_back(newCandy);
     std::cout << "Created candy successfully" << std::endl;
     return true;
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

Eigen::Quaterniond MyWorld::quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
    Eigen::Quaterniond resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

void MyWorld::simulate() {
    mFrame++;
    
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum.
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;
        
        //****Integration of angular momentum****
        Eigen::Vector3d dAngMoment = mRigidBodies[i]->mAccumulatedTorque;

        //****Integration of Orientation****
        //Convert quaternion to rotation matrix
        mRigidBodies[i]->mQuatOrient.normalize();
        mRigidBodies[i] -> mOrientation = mRigidBodies[i] -> mQuatOrient.toRotationMatrix();
        
        //    I(t) = R(t) * Ibody * transpose(R(t))
        Eigen::Matrix3d mOrientTrans = mRigidBodies[i]->mOrientation.transpose();
        Eigen::Matrix3d myI = mRigidBodies[i]->mOrientation * mRigidBodies[i]->iBody * mOrientTrans;
        //std::cout << "Debug: " << "I(t) = " << myI << std::endl;

        //Omega = w(t) = I(t)^-1 * L(t)
        Eigen::Vector3d omega = myI.inverse() * mRigidBodies[i]->mAngMomentum;
        //std::cout << "Debug: " << "omega = " << omega << std::endl;

        // dQuat = 1/2 * quat(omega) * mQuatOrientation
        Eigen::Quaterniond omega2 = Eigen::Quaterniond(0, omega(0), omega(1), omega(2));
        omega2.w() *= 0.5;
        omega2.vec() *= 0.5;

        //dQuat
        Eigen::Quaterniond dQuat = quatMult(omega2, mRigidBodies[i]->mQuatOrient);
        //std::cout << "Debug: " << "dQuat.w() = " << dQuat.w() << std::endl;
        //std::cout << "Debug: " << "dQuat.vec() = " << dQuat.vec() << std::endl;

        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;

        //****Update orientation****
        dQuat.w() *= mTimeStep;
        dQuat.vec() *= mTimeStep;

        Eigen::Quaterniond qNew;
        qNew.w() = dQuat.w() + mRigidBodies[i]->mQuatOrient.w();
        qNew.vec() = dQuat.vec() + mRigidBodies[i]->mQuatOrient.vec();

        mRigidBodies[i]->mQuatOrient = qNew;
        mRigidBodies[i]->mQuatOrient.normalize();

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
    
    // Iterates through collisions
    int nContacts = mCollisionDetector->getNumContacts();
    for(int collision = 0; collision<nContacts; collision++) {
        //HAMYChange - Really not sure if these pointers are right
        RigidBody* rb1 = mCollisionDetector->getContact(collision).rb1;
        RigidBody* rb2 = mCollisionDetector->getContact(collision).rb2;

        Eigen::Vector3d normal = mCollisionDetector->getContact(collision).normal;
        Eigen::Vector3d collisionPt = mCollisionDetector->getContact(collision).point;

        Eigen::Vector3d vPinata = mCollisionDetector->getContact(collision).pinataVelocity;

        std::cout << "****Debug: " << "Collision=" << collision << "****" << std::endl;
        //std::cout << "Debug: " << "CollisionPt=" << collisionPt << std::endl;


        double littleJ = 0;
        //TODO: How to tell if obj pointing at is null?
        if(rb1 && rb2) {
            //For cases when rigid on rigid, not pinata
            littleJ = getLittleJ(*rb1, *rb2, normal, collisionPt, epsilon);

            //****Update mAccumulatedForce and mAccumulatedTorque****
            
            //**Update particle A**
            Eigen::Vector3d rA = collisionPt - rb1->mPosition;
            Eigen::Vector3d bigJA = littleJ * normal;

            rb1->mAccumulatedForce += bigJA;
            rb1->mAccumulatedTorque += rA.cross(bigJA);

            //**Update particle B**
            Eigen::Vector3d rB = collisionPt - rb2->mPosition;
            Eigen::Vector3d bigJB = -littleJ * normal;

            rb2->mAccumulatedForce += bigJB;
            rb2->mAccumulatedTorque += rB.cross(bigJB);

            std::cout << "Debug: In Rigid on Rigid" << std::endl;
            std::cout << " " << std::endl;

        } else if (rb2) {
            //rb1 is Pinata
            littleJ = getLittleJPinata(*rb2, vPinata, normal, collisionPt, epsilon); //Negate because it's the "B" particle

            //**Update particle B**
            Eigen::Vector3d rB = collisionPt - rb2->mPosition;
            Eigen::Vector3d bigJB = -littleJ * normal;

            rb2->mAccumulatedForce += bigJB;
            rb2->mAccumulatedTorque += rB.cross(bigJB);

            std::cout << "Debug rb2: " << "littleJ=" << littleJ << std::endl;
            std::cout << " " << std::endl;
        } else if (rb1) {
            //rb2 is Pinata
            littleJ = getLittleJPinata(*rb1, vPinata, normal, collisionPt, epsilon);

            //**Update particle A**
            Eigen::Vector3d rA = collisionPt - rb1->mPosition;
            Eigen::Vector3d bigJA = littleJ * normal;

            rb1->mAccumulatedForce += bigJA;
            rb1->mAccumulatedTorque += rA.cross(bigJA);

            std::cout << "Debug rb1: " << "littleJ=" << littleJ << std::endl;
            std::cout << " " << std::endl;
        } else {
            //Someone done goofed
            std::cout << "Error: Neither rigid body is defined in collision handling" <<std::endl;
        }
    }

    //Update each rigid body's vals
    for(int body = 0; body<mRigidBodies.size(); body++) {
        mRigidBodies[body]->mLinMomentum += mRigidBodies[body]->mAccumulatedForce;
        mRigidBodies[body]->mAngMomentum += mRigidBodies[body]->mAccumulatedTorque;

        //Set their accumulators back to zero
        mRigidBodies[body]->mAccumulatedForce.setZero();
        mRigidBodies[body]->mAccumulatedTorque.setZero();
    }
}

double MyWorld::getLittleJPinata(RigidBody rigidA, Eigen::Vector3d pinataVelocity, 
    Eigen::Vector3d normal, Eigen::Vector3d collisionPt, double epsilon){

    std::cout << "--Debug: getLittleJPinata" << std::endl;
    std::cout << "--Debug: CollisonPt=" << collisionPt <<std::endl;

    double littleJ = 0;

    Eigen::Vector3d rA = collisionPt - rigidA.mPosition;

    std::cout << "--DEbug: rA=" << rA << std::endl;

    //****Calculate vR****
    //**Calculate dotPA
    Eigen::Matrix3d iA = rigidA.mOrientation * rigidA.iBody * rigidA.mOrientation.transpose();
    std::cout << "--Debug: iA=" << iA << std::endl;

    std::cout << "--Debug: rigidAMomentum=" << rigidA.mLinMomentum << std::endl;

    Eigen::Vector3d omegaA = iA.inverse() * rigidA.mAngMomentum;
    Eigen::Vector3d dotPA = rigidA.mLinMomentum / (double) rigidA.mMass + omegaA.cross(rA)
        + mGravity * mTimeStep;

    std::cout << "--Debug: dotPA=" << dotPA << std::endl;

    //**Calculate dotPP
    //Goes to zero so collisions don't affect it

    double vR = normal.dot(dotPA - pinataVelocity);

    if(vR >= 0) return 0;

    std::cout << "--Debug: pinataVelocity=" << pinataVelocity << std::endl;
    std::cout << "--Debug: vR=" << vR << std::endl;

    //****littleJDenominator****

    double littleJDenom = 1.0/rigidA.mMass;
    littleJDenom += normal.dot( ( iA.inverse() * ( rA.cross(normal) ) ).cross(rA) ); 

    std::cout << "--Debug: littleJDenom=" << littleJDenom << std::endl;

    littleJ = -(1.0 + epsilon) * vR / littleJDenom;

    std::cout << "--Debug: littleJReturn=" << littleJ << std::endl;

    return littleJ;
}

double MyWorld::getLittleJ(RigidBody rigidA, RigidBody rigidB, Eigen::Vector3d normal, 
    Eigen::Vector3d collisionPt, double epsilon) {

    double littleJ = 0;

    //****Calculate vR-pre****
    Eigen::Vector3d rA = collisionPt - rigidA.mPosition;
    Eigen::Vector3d rB = collisionPt - rigidB.mPosition;

    //**Calculate dotPA
    Eigen::Matrix3d iA = rigidA.mOrientation * rigidA.iBody * rigidA.mOrientation.transpose();
    Eigen::Vector3d omegaA = iA.inverse() * rigidA.mAngMomentum;
    Eigen::Vector3d dotPA = rigidA.mLinMomentum / rigidA.mMass + omegaA.cross(rA)
        + mGravity * mTimeStep;

    //**Calculate dotPB
    Eigen::Matrix3d iB = rigidB.mOrientation * rigidB.iBody * rigidB.mOrientation.transpose();
    Eigen::Vector3d omegaB = iB.inverse() * rigidB.mAngMomentum;
    Eigen::Vector3d dotPB = rigidB.mLinMomentum / rigidB.mMass + omegaB.cross(rB)
        + mGravity * mTimeStep;

    double vR = normal.dot( (dotPA - dotPB) );

    if(vR >= 0 ) return 0;

    //****littleJDenominator****

    double littleJDenom = 1/rigidA.mMass + 1/rigidB.mMass;
    littleJDenom += normal.dot( ( iA.transpose() * ( rA.cross(normal) ) ).cross(rA) );
    littleJDenom += normal.dot( ( iB.transpose() * ( rB.cross(normal) ) ).cross(rB) );

    littleJ = -(1.0 + epsilon) * vR / littleJDenom;

    return littleJ;
}









