#pragma once
#include <vector>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/cone.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "topics.h"
#include "utils.hpp"
#include "types.hpp"


class acceleration_node : public rclcpp::Node
{
    public:
     acceleration_node();
    private:
        std::size_t last_idx_ = 0;
        std::vector<PathStruct> map;
        bool map_Localized = false;
        CarData carData;
        lart_msgs::msg::ConeArray::SharedPtr coneArray;

        //LOCALIZACAO A PARTIR DA LINHA DE PARTIDA (cone azul + cone amarelo mais proximos)
        static constexpr size_t LOCALIZATION_SAMPLES = 10; // numero de deteçoes a promediar antes de trancar o mapa
        static constexpr double MIN_LANE_WIDTH = 1.0;      // metros, filtro de deteções espúrias
        static constexpr double MAX_LANE_WIDTH = 6.0;      // metros, filtro de deteções espúrias

        size_t localization_samples_ = 0;
        double sum_ref_x_ = 0.0;
        double sum_ref_y_ = 0.0;
        double sum_cos_ = 0.0;
        double sum_sin_ = 0.0;

        double prev_corr_x_ = 0.0;
        double prev_corr_y_ = 0.0;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;
        rclcpp::Publisher<lart_msgs::msg::PathSpline>::SharedPtr path_control_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
        rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr cone_array_subscriber;

        void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg);
        void publishPath();
        void track_correction(lart_msgs::msg::PathSpline *path,nav_msgs::msg::Path *path_vis);

};
