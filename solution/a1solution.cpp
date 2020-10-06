#include "a1solution.h"

#include "dependencies/Eigen/Dense"


using Eigen::MatrixXd;

A1Solution::A1Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links)
    :m_joints(joints),
    m_links(links){

    this->initialize_joint_transforms();
}



void A1Solution::update(Joint2D* selected, QVector2D mouse_pos){

    // Students implement solution here
}

void A1Solution::initialize_joint_transform(int i) {
    Joint2D* joint = m_joints[i];
    QVector2D joint_pos = joint->get_position();

    if(joint->get_parents().empty()) {
        // Use relative postion if child
        QVector2D parent_pos = joint->get_parents()[0]->get_position();
        QVector2D relative_pos = joint_pos - parent_pos;
        this->m_joint_transforms[i] = Eigen::Vector2f(relative_pos.x(), relative_pos.y());

    } else {
        // Use global postion if root
        this->m_joint_transforms[i] = Eigen::Vector2f(joint_pos.x(), joint_pos.y());
    }

    return;
}

void A1Solution::initialize_joint_transforms() {
    for(unsigned int i=0; i < this->m_joints.size(); ++i){
        this->initialize_joint_transform(i);
    }
}


void A1Solution::test_eigen_library(){

    // create a simple matrix 5 by 6
    MatrixXd mat(5,6);

    // Fills in matrix
    for(unsigned int row=0;row<mat.rows();row++){
        for(unsigned int col=0;col<mat.cols();col++){
            mat(row,col) = row+col;
        }
    }

    // create the pseudoinverse
    MatrixXd pseudo_inv = mat.completeOrthogonalDecomposition().pseudoInverse();

    // print the pseudoinverse
    std::cout << "--------------------------" << std::endl;
    std::cout << pseudo_inv << std::endl;
    std::cout << "--------------------------" << std::endl;

}
