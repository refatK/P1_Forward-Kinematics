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

    std::vector<Eigen::Vector2f> m_joint_transforms;


    void update(Joint2D* selected, QVector2D mouse_pos);

    static void test_eigen_library();

private:
    void initialize_joint_transform(int index);
    void initialize_joint_transforms();

};

#endif // A2SOLUTION_H
