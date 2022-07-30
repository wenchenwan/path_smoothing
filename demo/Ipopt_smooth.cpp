#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>
#include "csv_reader.hpp"
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <memory>
#include <string>
#include<Eigen/Core>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>


typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;


// using namespace std;

namespace {

using CppAD::AD;
class FG_eval 
{
	public:
	    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	    void operator()(ADvector& fg, const ADvector& x)
	    {
	        assert(fg.size() == 1);
	        // assert(x.size() == 4);
            std::vector<AD<double>> X_array;
            for(int i(0);i<x.size();++i){
                X_array.push_back(x[i]);
            }

            AD<double> dx_i = X_array[2] - X_array[0];
            AD<double> dy_i = X_array[3] - X_array[1];
            AD<double> dx_ii,dy_ii;
            int size = X_array.size()/2;
            for(int i=1; i < size - 1; ++i){
                if(i < size -2){
                    dx_ii = X_array[i*2] - X_array[(i-1)*2];
                    dy_ii = X_array[i*2 + 1] - X_array[(i-1)*2 + 1];
                }else{
                    dx_ii = X_array[2*(size - 1)] - X_array[(i-1)*2];
                    dy_ii = X_array[2*(size - 1) + 1] - X_array[(i-1)*2 + 1];
                }

                AD<double> heading_error = (dx_ii - dx_i) * (dx_ii - dx_i)
                + (dy_ii - dy_i) * (dy_ii - dy_i);

                AD<double> dot = dx_ii * dx_i + dy_ii * dy_i;
                AD<double> norm_i = sqrt(dx_i * dx_i + dy_i * dy_i) + 0.01;
                AD<double> norm_ii = sqrt(dx_ii * dx_ii + dy_ii * dy_ii) + 0.01;
                AD<double> theta = acos(dot / norm_i / norm_ii);

                double cof_1 = 1,cof_2 = 1;
                fg[0] = fg[0] + heading_error*cof_1 + cof_2 * theta * theta / norm_i / norm_i;
                
                dx_i = dx_ii;
                dy_i = dy_ii;
                
            }
	        // variables
	        // AD<double> x1 = x[0];
	        // AD<double> x2 = x[1];
	        // AD<double> x3 = x[2];
	        // AD<double> x4 = x[3];
	        // f(x) objective function
	        // fg[0] = x1 * x4 * (x1 + x2 + x3) + x3;
	        // constraints
	        // fg[1] = x1 * x2 * x3 * x4;
	        // fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
	        return;
	    }
};

}
struct data
{
    data():param_num(-1),
            degree(2),
            start(0),
            end(0){

    }
    Vector params;
    Vector start;
    Vector end;

    int param_num;
    int degree;
};


nav_msgs::Path get_started(std::vector<geometry_msgs::Point>& path)
{


    data my_data;
    my_data.param_num = (path.size() - 2)*my_data.degree;

    my_data.start.resize(my_data.degree);
    my_data.end.resize(my_data.degree);

    my_data.params.resize(my_data.param_num);

    
    for(int i(1);i<path.size() - 1;++i){
        const int j = i - 1;
        my_data.params(j*2) = path.at(i).x;
        my_data.params(j*2+1) = path.at(i).y;

    }

     

    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;


    size_t nx = path.size()*2; // number of varibles
    size_t ng = 0; // number of constraints
    Dvector x0(nx);

    
    // Dvector x0(nx); // initial condition of varibles
    for(int i(0);i<path.size();++i){
        x0[i*my_data.degree] = path.at(i).x;
        x0[i*my_data.degree+1] = path.at(i).y;
    }


    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for(i = 0; i < nx; i++)
    {
        xl[i] = x0[i]-2;
        xu[i] = x0[i]+2;
    }
    Dvector gl(ng), gu(ng);
    // gl[0] = 25.0;    gu[0] = 1.0e19;
    // gl[1] = 40.0;    gu[1] = 40.0;
    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  3\n";
    options += "String sb            no\n";
    // maximum iterations
    options += "Integer max_iter     30\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    //derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";


    CppAD::ipopt::solve_result<Dvector> solution; // solution
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, x0, xl, xu, gl, gu, fg_eval, solution); 

    std::cout<<"solution: "<<solution.x<<std::endl;

    // std::cout << solution.x << std::endl;

    nav_msgs::Path smooth_path;
    geometry_msgs::PoseStamped pose;
    smooth_path.header.frame_id = "map";
    smooth_path.header.stamp = ros::Time::now();
    pose.header = smooth_path.header;


    for(int i=0;i<solution.x.size();i = i+2){
        pose.pose.position.x = solution.x[i];
        pose.pose.position.y = solution.x[i+1];

        smooth_path.poses.push_back(pose);

    }

    std::cout << solution.obj_value << std::endl;
    //
    //check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    // //
    // double check_x[]  = {1.000000, 4.743000, 3.82115, 1.379408};
    // double check_zl[] = {1.087871, 0.,       0.,       0.      };
    // double check_zu[] = {0.,       0.,       0.,       0.      };
    // double rel_tol    = 1e-6; // relative tolerance
    // double abs_tol    = 1e-6; // absolute tolerance
    // for(i = 0; i < nx; i++)
    // {
    //     ok &= CppAD::NearEqual(check_x[i], solution.x[i], rel_tol, abs_tol);                    
    //     ok &= CppAD::NearEqual(check_zl[i], solution.zl[i], rel_tol, abs_tol);                    
    //     ok &= CppAD::NearEqual(check_zu[i], solution.zu[i], rel_tol, abs_tol);                    
    // }

    return smooth_path;
}

int main(int argc, char **argv)
{

    

    ros::init(argc, argv, "path_smooth_Ipopt");
    ros::NodeHandle nh;

    
    std::string basic_dir = ros::package::getPath("path_smoothing");

    ros::Publisher pub_0 = nh.advertise<nav_msgs::Path>("rough_path",1000);
    ros::Publisher pub_1 = nh.advertise<nav_msgs::Path>("smooth_path",1000);

    io::CSVReader<2> in(basic_dir + "/demo/data_sin.csv");
    in.read_header(io::ignore_extra_column, "x", "y");

    std::vector<geometry_msgs::Point> path;

    geometry_msgs::Point pt;
    while(in.read_row(pt.x,pt.y)){
        path.push_back(pt);
    }

    nav_msgs::Path original_path;
    original_path.header.frame_id = "map";
    original_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = original_path.header;


    for (const auto &pt:path) {
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        original_path.poses.push_back(pose);
    }

    
    std::cout << "CppAD : Hello World Demo!" << std::endl;
    nav_msgs::Path smooth_path = get_started(path);

    while(ros::ok()){
        pub_0.publish(original_path);
        pub_1.publish(smooth_path);

        ros::spinOnce();
    }



    return 0;
}
