#ifndef A1SOLUTION_H
#define A1SOLUTION_H

#include <vector>
#include <dependencies/Eigen/Dense>

#include "OpenGL/elements/joint2D.h"
#include "OpenGL/elements/obstacle2d.h"
#include "OpenGL/elements/link2d.h"

class A1Solution
{
public:
    A1Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links);

    std::vector<Joint2D*>& m_joints;
    std::vector<Link2D*>& m_links;

    void update(Joint2D* selected, QVector2D mouse_pos);

    static void test_eigen_library();

private:
    void doFkPass(Joint2D& joint, QVector2D mouse_pos);
    void moveJointBy(Joint2D& joint, QVector2D translation);
    void rotateJointBy(Joint2D& joint, QVector2D currMathVecFromParent, float theta);
    float angleToRotate(QVector2D mathVecToJoint, QVector2D mathVecToNewPos);
    float getMathAngle(QVector2D mathVec);
    float radsToDegrees(float radians);
    QVector2D qtToMathCoords(QVector2D qtVec);
    QVector2D mathToQtCoords(QVector2D mathVec);
    bool isRoot(Joint2D& joint);
    int getJointIndex(Joint2D& joint);
};

#endif // A2SOLUTION_H
