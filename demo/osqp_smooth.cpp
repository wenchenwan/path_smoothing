#include <fstream>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include<OsqpEigen/Data.hpp>
#include<OsqpEigen/Solver.hpp>
#include<OsqpEigen/OsqpEigen.h>
#include<iostream>
#include"csv_reader.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
// #include<Eigen/SparseMatrix.h>
// #include<Eigen/Core>
// #include<Eigen/Dense>
// #include<Eigen/Sparse>


/*
    正常具有不等式约束的二次规划问题
*/

Eigen::SparseMatrix<double> hessian;
Eigen::VectorXd gradient;
Eigen::SparseMatrix<double> linearMartix;
Eigen::VectorXd lowerBound;
Eigen::VectorXd upperBound;

Eigen::VectorXd QPSolution;

int numberOfVaribles = 100;
int numberOfConstraints = 100;
static double cost1_cof;
static double cost2_cof;
static double cost3_cof;





int main(int argc,char **argv){
    
    

    ros::init(argc,argv,"osqp_smooth");

    ros::NodeHandle nh;

    ros::Publisher pub_0 = nh.advertise<nav_msgs::Path>("rough_path",1000);
    ros::Publisher pub_1 = nh.advertise<nav_msgs::Path>("smooth_path",1000);

    cost1_cof = 1.0; //平滑代价
    cost2_cof = 0.3;  //相似代价

    cost3_cof = 0.8;  //距离代价

    std::string basic_dir = ros::package::getPath("path_smoothing");
    std::vector<geometry_msgs::Point> path;

    io::CSVReader<2> in(basic_dir + "/demo/data_sin.csv");;
    in.read_header(io::ignore_extra_column, "x", "y");

    geometry_msgs::Point p;
    std::vector<double> pt;
    while(in.read_row(p.x,p.y)){
        path.push_back(p);
        pt.push_back(p.x);
        pt.push_back(p.y);
    }
    numberOfVaribles = pt.size();
    numberOfConstraints = pt.size();
    nav_msgs::Path original_path,smooth_path;
    original_path.header.frame_id = "map";
    original_path.header.stamp = ros::Time::now();

    smooth_path.header = original_path.header;
    geometry_msgs::PoseStamped pose;
    pose.header = original_path.header;


    for (const auto &p:path) {
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        original_path.poses.push_back(pose);
    }


    double x,y;
    


    Eigen::VectorXd X(numberOfVaribles);
    for(int i=0;i<numberOfVaribles;++i){
        X[i] =  pt[i];
    }
    Eigen::VectorXd X_ref(X);

    
    
    const int n = numberOfVaribles/2;
    Eigen::MatrixXd A1(2*n,2*(n - 2));
    for(int i = 0;i<2*(n-2);++i){
        A1(i,i) = 1;
        A1(i+2,i) = -2;
        A1(i+4,i) = 1;
    }
    std::cout << "A1 row : " << A1.rows() << " A1 col : " <<A1.cols() << std::endl;
    A1 = A1*A1.transpose();
    std::cout << "A1T*A1 row : " << A1.rows() << " A1 col : " << A1.cols() << std::endl;

    
    Eigen::MatrixXd A2(2*n,2*(n - 1));
    for(int i=0;i<2*(n - 1);++i){
        A2(i,i) = 1;
        A2(i+2,i) = -1;
    }

    std::cout << "A2 row : " << A2.rows() << " A2 col : " << A2.cols() << std::endl;
    A2 =A2 * A2.transpose();
    std::cout << "A2T*A2 row : " << A2.rows() << " A2 col : " << A2.cols() << std::endl;


    Eigen::MatrixXd A3(2*n,2*n);
    for(int i=0;i<2*n;++i){
        A3(i,i) = 1;
    }

    Eigen::VectorXd f(2*n);
    for(int i=0;i<2*n;++i){
        f(i) = 0;
    }

    Eigen::MatrixXd A = cost1_cof*A1 + cost2_cof*A2 + cost3_cof*A3;
    

    hessian.resize(numberOfVaribles,numberOfVaribles);
    for(int i=0;i<A.rows();++i){
        for(int j=0;j<A.cols();++j){
            if(A(i,j) != 0)
                hessian.insert(i,j) = A(i,j);
        }
    }

    // std::cout << hessian << std::endl;


    OsqpEigen::Solver solver;

    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(numberOfVaribles);
    solver.data()->setNumberOfConstraints(numberOfConstraints);

    // hessian.resize(numberOfVaribles,numberOfVaribles);
    // hessian.insert(0,0) = 1;
    // hessian.insert(0,1) = -1;
    // hessian.insert(1,0) = -1;
    // hessian.insert(1,1) = 2;
    // 自己当前的版本没有重载流运算符
    // hessian <<  1,-1
    //             -1,2;
    
    // gradient.resize(numberOfVaribles);
    gradient = X_ref;
    
    linearMartix.resize(numberOfConstraints,numberOfVaribles);
    for(int i=0;i<numberOfConstraints;++i){
        linearMartix.insert(i,i) = 1;
    }


    // linearMartix.insert(0,0) = 1;
    // linearMartix.insert(0,1) = 1;
    // linearMartix.insert(1,0) = -1;
    // linearMartix.insert(1,1) = 2;
    // linearMartix.insert(2,0) = 2;
    // linearMartix.insert(2,1) = 1;
    // linearMartix << 1,1,-1,
    //                 2,2,1;

    lowerBound.resize(numberOfConstraints);
    upperBound.resize(numberOfConstraints);
    for(int i=0;i<numberOfConstraints;++i){
        lowerBound(i) = X_ref(i) - 2;
    }
    for(int i=0;i<numberOfConstraints;++i){
        upperBound(i) = X_ref(i) + 2;
    }
    

    if(!solver.data()->setHessianMatrix(hessian)) return 1;
    if(!solver.data()->setGradient(gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(linearMartix)) return 1;
    if(!solver.data()->setLowerBound(lowerBound)) return 1;
    if(!solver.data()->setUpperBound(upperBound)) return 1;

    if(!solver.initSolver()) return 1;
    if(!solver.solve()) return 1;

    QPSolution = solver.getSolution();


    std::cout << "The Program is Runing !!" << std::endl;
    std::cout << QPSolution << std::endl;
    for (int i=0;i<QPSolution.size();i =i + 2) {
        pose.pose.position.x = QPSolution[i];
        pose.pose.position.y = QPSolution[i+1];
        smooth_path.poses.push_back(pose);
    }

    while(ros::ok()){
        pub_0.publish(original_path);
        pub_1.publish(smooth_path);

        ros::spinOnce();
    }
    
    return 0;
}