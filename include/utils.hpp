#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "lart_msgs/msg/cone_array.hpp"
#include "types.hpp"

double distance(double x, double y, double x1, double y1);

geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y,
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp);

std::vector<PathStruct> file_loader(std::string fileName);

// Aplica rotação (heading) + translação (ref_x, ref_y) a todos os pontos do mapa,
// alinhando a origem do CSV (linha de partida) com a linha de partida real.
void localize_map(double ref_x, double ref_y, double heading, std::vector<PathStruct> *map);

// Devolve, por apontador, o indice do cone azul e amarelo mais proximos do carro (origem).
void nearest_cone(const lart_msgs::msg::ConeArray::SharedPtr msg, int *blue_index_o, int *yellow_index_o);
