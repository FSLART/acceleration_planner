#include "../include/utils.hpp"
#include <fstream>
#include <limits>
#include <sstream>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include "topics.h"

double distance(double x, double y, double x1, double y1){
    return (double)std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
}

geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y,
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;

        pose.header.frame_id = "world";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        return pose;
    }

void localize_map(double ref_x, double ref_y, double heading, std::vector<PathStruct> *map)
{
    double cos_h = std::cos(heading);
    double sin_h = std::sin(heading);

    for (PathStruct& p : *map) {
        double ox = p.x; // coordenadas no referencial do CSV (linha de partida na origem)
        double oy = p.y;
        p.x = ox * cos_h - oy * sin_h + ref_x;
        p.y = ox * sin_h + oy * cos_h + ref_y;
    }
}

std::vector<PathStruct> file_loader(std::string fileName){
    std::ifstream file(fileName);
    std::vector<PathStruct> path_points;

    if(!file.is_open())
    {
        std::cerr << "Cannot open acceleration path file: " << fileName << std::endl;
        return path_points;
    }

    std::string line;
    std::getline(file, line); // descarta o cabeçalho ("x,y")

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string x_str, y_str;

        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str)){
            PathStruct tmp;
            tmp.x = std::stod(x_str);
            tmp.y = std::stod(y_str);
            tmp.cur = 0.0; // reta: curvatura sempre nula
            path_points.push_back(tmp);
        }
    }
    return path_points;
}

//Return values are passed through pointers
void nearest_cone(const lart_msgs::msg::ConeArray::SharedPtr msg, int *blue_index_o, int *yellow_index_o){
    *blue_index_o = -1;
    *yellow_index_o = -1;

    auto cones_s = msg->cones;
    if(cones_s.empty()) {
        return;
    }

    double dist_b = std::numeric_limits<double>::max();
    double dist_y = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cones_s.size(); i++){

        if (cones_s[i].class_type.data == lart_msgs::msg::Cone::BLUE){
            double tmp_dist = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
            if (tmp_dist < dist_b){
                dist_b = tmp_dist;
                *blue_index_o = i;
            }
        }

        if (cones_s[i].class_type.data == lart_msgs::msg::Cone::YELLOW){
            double tmp_dist = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
            if (tmp_dist < dist_y){
                dist_y = tmp_dist;
                *yellow_index_o = i;
            }
        }
    }
}
