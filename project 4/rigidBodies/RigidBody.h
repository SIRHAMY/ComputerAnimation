#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include <Eigen/Dense>
#include "dart/dart.h"

class RigidBody {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RigidBody(dart::dynamics::Shape::ShapeType _type, Eigen::Vector3d _dim) {
        // Create a default rigid body
        mMass = 1.0;
        mPosition.setZero(); // x = (0, 0, 0)
        mOrientation.setIdentity(); // R = identity
		mQuatOrient.setIdentity();	//initialize quaternion
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
        
        if (_type == dart::dynamics::Shape::BOX) {
            mShape = Eigen::make_aligned_shared<dart::dynamics::BoxShape>(_dim);
            iBody << mMass*(std::pow(_dim(0), 2.0) + std::pow(_dim(2), 2.0)) / 12.0, 0, 0,
                     0, mMass*(std::pow(_dim(2), 2.0) + std::pow(_dim(1), 2.0)) / 12.0, 0,
                     0, 0, mMass*(std::pow(_dim(0), 2.0) + std::pow(_dim(1), 2.0)) / 12.0;
        } else if (_type == dart::dynamics::Shape::ELLIPSOID) {
            mShape = Eigen::make_aligned_shared<dart::dynamics::EllipsoidShape>(_dim);
            
            double sphereIBody = (2.0 / 5.0) * mMass * std::pow(_dim(0), 2.0);
            iBody << sphereIBody, 0, 0,
                     0, sphereIBody, 0,
                     0, 0, sphereIBody;
        }
        
        mLinMomentum.setZero();
        mAngMomentum.setZero();
        
        mAccumulatedForce.setZero();
        mAccumulatedTorque.setZero();

        //Going to initialize iBody here using the _dim param
        //std::cout << "_dim: " << _dim << std::endl;
    }
    virtual ~RigidBody() {}

    void draw(dart::renderer::RenderInterface* _ri);

    int getConfigSize() {
		return mPosition.size() + mOrientation.size();
    }
    
    double mMass;

    Eigen::Matrix3d iBody; //Want to store the Ibody here

	Eigen::Vector3d mPosition;
    Eigen::Quaterniond mQuatOrient; // quaternion
	Eigen::Matrix3d mOrientation;   // rotation matrix
    Eigen::Vector3d mLinMomentum;
    Eigen::Vector3d mAngMomentum;
    dart::dynamics::ShapePtr mShape;
    
	Eigen::Vector3d mAccumulatedForce;
    Eigen::Vector3d mAccumulatedTorque;

    Eigen::Vector4d mColor;
};

#endif
