//
// Created by yangt on 19-2-18.
//
#include "path_smoothing/path_smoothing.hpp"
#include "csv_reader.hpp"

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <fstream>

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "path_smooth_demo");
    ros::NodeHandle nh;
    ros::Publisher pub_0 = nh.advertise<nav_msgs::Path>("rough_path",1000);
    ros::Publisher pub_1 = nh.advertise<nav_msgs::Path>("smooth_path",1000);
    ros::Publisher pub_2 = nh.advertise<nav_msgs::Path>("path_pre_insert",1000);
    std::string basic_dir = ros::package::getPath("path_smoothing");
    io::CSVReader<2> in(basic_dir + "/demo/data_sin.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    std::vector<geometry_msgs::Point> path,control;

    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
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
    pub_0.publish(original_path);


    using namespace path_smoothing;

    PathSmoothing::Options options;
    options.cg_solver = CERES_SOLVER;
    options.smoother_type = CONJUGATE_GRADIENT_METHOD;

//    options.type = CASADI;
    std::unique_ptr<PathSmoothing>
            smoother(PathSmoothing::createSmoother(options, path));
    smoother->smoothPath(options);
    smoother->getSmoothPath(&path,&control);

    nav_msgs::Path smooth_path,path_pre_insert;
    

    smooth_path.header.frame_id = "map";
    smooth_path.header.stamp = ros::Time::now();

    path_pre_insert.header = smooth_path.header;

    for(const auto& state : control){
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;

        path_pre_insert.poses.push_back(pose);

    }

    for(const auto &state : path){
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;

        smooth_path.poses.push_back(pose);
    }

    pub_1.publish(smooth_path);





    // write to file:
    std::ofstream fout(basic_dir+"/demo/smooth_without_obstacle_result_1.csv");
    if(fout.is_open()) {
        fout << "\"x\"" << "," << "\"y\"" << "\n";
        for (int i(0); i < path.size(); ++i) {
            fout << path.at(i).x << "," << path.at(i).y << "\n";
        }
        fout.close();
    }

    while(ros::ok()){
        pub_0.publish(original_path);
        pub_1.publish(smooth_path);
        pub_2.publish(path_pre_insert);

        ros::spinOnce();
    }
    

}

