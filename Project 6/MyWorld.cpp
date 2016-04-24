#include "MyWorld.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
  // Load a skeleton from file
  mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");

  // Create markers
  createMarkers();
  
  // Initialize Jacobian assuming that there is only one constraint
  mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
  mConstrainedMarker = -1;
}

MyWorld::~MyWorld() {
}

void MyWorld::solve() {
  if (mConstrainedMarker == -1)
    return; 
  int numIter = 300;
  double alpha = 0.01;
  int nDof = mSkel->getNumDofs();
  VectorXd gradients(nDof);
  VectorXd newPose(nDof);
  for (int i = 0; i < numIter; i++) {
    gradients = updateGradients();
    newPose = mSkel->getPositions() - alpha * gradients;
    mSkel->setPositions(newPose); 
    mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
  }
  
}

// Current code only works for the left leg with only one constraint
VectorXd MyWorld::updateGradients() {
  // compute c(q)
  mC = getMarker(mConstrainedMarker)->getWorldPosition() - mTarget;

  // compute J(q)
  Vector4d offset;
  offset << getMarker(mConstrainedMarker)->getLocalPosition(), 1; // Create a vector in homogeneous coordinates
  // w.r.t ankle dofs
  BodyNode *node = getMarker(mConstrainedMarker)->getBodyNode();
  Joint *joint = node->getParentJoint();
  Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
  Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  Matrix4d dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  Matrix4d R = joint->getTransform(1).matrix();
  Matrix4d jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  Vector4d jCol = worldToParent * parentToJoint * dR * R * jointToChild * offset;
  int colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of jCol
  dR = joint->getTransformDerivative(1);
  R = joint->getTransform(0).matrix();
  jCol = worldToParent * parentToJoint * R * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(1);
  mJ.col(colIndex) = jCol.head(3);
  offset = parentToJoint * joint->getTransform(0).matrix() * joint->getTransform(1).matrix() * jointToChild * offset; // Update offset so it stores the chain below the parent joint

  // w.r.t knee dof
  node = node->getParentBodyNode(); // return NULL if node is the root node
  joint = node->getParentJoint();
  worldToParent = node->getParentBodyNode()->getTransform().matrix();
  parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  jCol = worldToParent * parentToJoint * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of jCol
  offset = parentToJoint * joint->getTransform(0).matrix() * jointToChild * offset;

  // w.r.t hip dofs
  node = node->getParentBodyNode();
  joint = node->getParentJoint();
  worldToParent = node->getParentBodyNode()->getTransform().matrix();
  parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  Matrix4d R1 = joint->getTransform(1).matrix();
  Matrix4d R2 = joint->getTransform(2).matrix();
  jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  jCol = worldToParent * parentToJoint * dR * R1 * R2 * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of J

  R1 = joint->getTransform(0).matrix();
  dR = joint->getTransformDerivative(1);
  R2 = joint->getTransform(2).matrix();
  jCol = worldToParent * parentToJoint * R1 * dR * R2 * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(1);
  mJ.col(colIndex) = jCol.head(3);

  R1 = joint->getTransform(0).matrix();
  R2 = joint->getTransform(1).matrix();
  dR = joint->getTransformDerivative(2);
  jCol = worldToParent * parentToJoint * R1 * R2 * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(2);
  mJ.col(colIndex) = jCol.head(3);

  // compute gradients
  VectorXd gradients = 2 * mJ.transpose() * mC;
  return gradients;
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index) {
  if (_index == 0) {
    mTarget = getMarker(_index)->getWorldPosition();
    mConstrainedMarker = _index;
  } else {
    mConstrainedMarker = -1;
  }
}

void MyWorld::modifyConstraint(Vector3d _deltaP) {
  if (mConstrainedMarker == 0)
    mTarget += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
  mConstrainedMarker = -1;
}

Marker* MyWorld::getMarker(int _index) {
  return mMarkers[_index];
}

void MyWorld::createMarkers() {
  Vector3d offset(0.2, 0.0, 0.0);
  BodyNode* bNode = mSkel->getBodyNode("h_heel_right");
  Marker* m = new Marker("right_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.2, 0.0, 0.0);
  bNode = mSkel->getBodyNode("h_heel_left");
  m = new Marker("left_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_right");
  m = new Marker("right_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_left");
  m = new Marker("left_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, 0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, -0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.075, 0.1, 0.0);
  bNode = mSkel->getBodyNode("h_abdomen");
  m = new Marker("abdomen", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, 0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, -0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_right");
  m = new Marker("right_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_left");
  m = new Marker("left_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, 0.05);
  bNode = mSkel->getBodyNode("h_bicep_right");
  m = new Marker("right_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, -0.05);
  bNode = mSkel->getBodyNode("h_bicep_left");
  m = new Marker("left_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, 0.025);
  bNode = mSkel->getBodyNode("h_hand_right");
  m = new Marker("right_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, -0.025);
  bNode = mSkel->getBodyNode("h_hand_left");
  m = new Marker("left_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);
}
