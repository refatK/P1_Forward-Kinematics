#include "a1solution.h"

#include <QDebug>
#include "dependencies/Eigen/Dense"

using Eigen::MatrixXd;

A1Solution::A1Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links)
    :m_joints(joints),
    m_links(links){
}

void A1Solution::update(Joint2D* selected, QVector2D mouse_pos){
    // load vector postions in array
//    this->initialize_all_joint_positions();
    this->initialize_all_joint_transforms();

    // Do Forward Kinematics
    this->doFkPass(*selected, mouse_pos);

    // if joint is a root (has no parent, move it to mouse position)
//    if (this->isRoot(*selected)) {
//        // TODO, move children
//        selected->set_position(mouse_pos);
//    }
}

void A1Solution::doFkPass(Joint2D& joint, QVector2D mouse_pos) {
    // TODO
    return;
}

void A1Solution::updateJointPositions(Joint2D& joint) {
    // TODO
    return;
}


bool A1Solution::isRoot(Joint2D& joint) {
    return joint.get_parents().empty();
}


void A1Solution::initialize_joint_transform(int i) {
    Joint2D* joint = m_joints[i];
    QVector2D joint_pos = joint->get_position();

    qDebug() << "INITITALIZE_A_TRANSFORM: this is node#" << i;

    if(this->isRoot(*joint)) {
//        qDebug() << "In root case " << i;
        // Use global postion if root
        this->m_joint_transforms[i].translate(Eigen::Vector2f(joint_pos.x(), -joint_pos.y()));
//        std::cout << "Global pos of root node#" << i << " is:" << std::endl << m_joint_transforms[i].matrix() << std::endl << std::endl;
    } else {
//        qDebug() << "In child case " << i;
        // Use relative postion if child
        QVector2D parent_pos = joint->get_parents()[0]->get_position();
        QVector2D relative_pos = joint_pos - parent_pos;
        this->m_joint_transforms[i].translate(Eigen::Vector2f(relative_pos.x(), -relative_pos.y()));
//        std::cout << "local pos of child node#" << i << " is:" << std::endl << m_joint_transforms[i].matrix() << std::endl << std::endl;
    }
}

void A1Solution::initialize_all_joint_transforms() {
    // TODO: Remove
    qDebug() << "Hello WORLD: A1Solution::initialize_joint_transforms()";

    // set transform list size to joints size
    std::vector<Eigen::Transform<float,2,Eigen::Affine>> transforms(this->m_joints.size());
    m_joint_transforms = transforms;

    qDebug() << "size is " << this->m_joint_transforms.size();

    for(int i=0; i < this->m_joint_transforms.size(); ++i){
        this->m_joint_transforms[i] = Eigen::Transform<float,2,Eigen::Affine>::Identity();
        this->initialize_joint_transform(i);
    }
}

void A1Solution::initialize_joint_position(int i) {
    Joint2D* joint = m_joints[i];
    QVector2D joint_pos = joint->get_position();

    qDebug() << "this is part " << i;

    if(joint->get_parents().empty()) {
        qDebug() << "In root case " << i;
        // Use global postion if root
        this->m_joint_positions[i] = Eigen::Vector2f(joint_pos.x(), joint_pos.y());
        qDebug() << "global pos of root node " << i << " is x=" << joint_pos.x() << ", y=" << joint_pos.y();

    } else {
        qDebug() << "In child case " << i;
        // Use relative postion if child
        QVector2D parent_pos = joint->get_parents()[0]->get_position();
        QVector2D relative_pos = joint_pos - parent_pos;
        this->m_joint_positions[i] = Eigen::Vector2f(relative_pos.x(), relative_pos.y());
        qDebug() << "relative pos of node " << i << " is x=" << relative_pos.x() << ", y=" << relative_pos.y();
    }
}

void A1Solution::initialize_all_joint_positions() {
    // TODO: Remove
    qDebug() << "Hello WORLD: A1Solution::initialize_joint_positions()";

    // set transform size to joints size
    std::vector<Eigen::Vector2f> transforms(this->m_joints.size());
    m_joint_positions = transforms;

    qDebug() << "size is " << this->m_joint_positions.size();

    for(int i=0; i < this->m_joint_positions.size(); ++i){
        this->initialize_joint_position(i);
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
