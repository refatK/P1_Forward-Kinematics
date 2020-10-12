#ifndef A1SOLUTION_H
#define A1SOLUTION_H

#include <vector>
#include <dependencies/Eigen/Dense>
#include <dependencies/Eigen/Geometry>

#include "OpenGL/elements/joint2D.h"
#include "OpenGL/elements/obstacle2d.h"
#include "OpenGL/elements/link2d.h"

class A1Solution
{
public:
    A1Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links);

    std::vector<Joint2D*>& m_joints;
    std::vector<Link2D*>& m_links;

    std::vector<Eigen::Vector2f> m_joint_positions;
    std::vector<Eigen::Transform<float,2,Eigen::Affine>>* m_joint_transforms;

    void update(Joint2D* selected, QVector2D mouse_pos);

    static void test_eigen_library();

private:
    bool isRoot(Joint2D& joint);
    int getJointIndex(Joint2D& joint);
    void doFkPass(Joint2D& joint, QVector2D mouse_pos);
    void updateJointPositions(Joint2D& joint);
    void initialize_joint_position(int i);
    void initialize_all_joint_positions();
    void initialize_joint_transform(int i);
    void initialize_all_joint_transforms();
    void moveJointBy(Joint2D& joint, QVector2D translation);
    void rotateJointBy(Joint2D& joint, QVector2D currMathVecFromParent, float theta);
    QVector2D qtToMathCoords(QVector2D qtVec);
    QVector2D mathToQtCoords(QVector2D mathVec);
    float getMathAngle(QVector2D mathVec);
    float angleToRotate(QVector2D mathVecToJoint, QVector2D mathVecToNewPos);
    float radsToDegrees(float radians);
};

#endif // A2SOLUTION_H
