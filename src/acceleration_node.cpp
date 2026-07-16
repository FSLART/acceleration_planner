#include "../include/acceleration_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <limits>
#include <utility>

using std::placeholders::_1;

acceleration_node::acceleration_node() : Node("acceleration_node")
{
    RCLCPP_INFO(this->get_logger(), "Acceleration node has been started");

    this->path_vis_pub = this->create_publisher<nav_msgs::msg::Path>(TOPIC_PATH_MARKER, 10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>(TOPIC_PATH, 10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>(TOPIC_CONES, 10, std::bind(&acceleration_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(TOPIC_SLAM_POSE, 10, std::bind(&acceleration_node::positionCallback, this, _1));

    std::string csv_path = ament_index_cpp::get_package_share_directory("acceleration") + "/data/acceleration_path.csv";
    map = file_loader(csv_path);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s", map.size(), csv_path.c_str());
};

void acceleration_node::publishPath()
{
    auto stamp = this->now();

    lart_msgs::msg::PathSpline pathSpline_msg;
    pathSpline_msg.header.stamp = stamp;
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = stamp;
    path_rviz_msg.header.frame_id = "world";

    if (map.empty())
        return; // Prevenção de segurança

    // Encontrar o ponto do mapa mais proximo do carro (a reta e aberta, sem ambiguidade de wraparound)
    double best_dist = std::numeric_limits<double>::max();
    std::size_t closest_idx = 0;

    for (std::size_t i = 0; i < map.size(); i++)
    {
        double d = distance(map[i].x, map[i].y, carData.car_x, carData.car_y);
        if (d < best_dist)
        {
            best_dist = d;
            closest_idx = i;
        }
    }
    last_idx_ = closest_idx;

    double last_added_x = map[closest_idx].x;
    double last_added_y = map[closest_idx].y;
    double cumulative_dist = 0.0;

    geometry_msgs::msg::PoseStamped pose = createPoseMsg(
        map[closest_idx].x, map[closest_idx].y,
        carData.roll, carData.pitch, carData.yaw, stamp);
    pathSpline_msg.poses.push_back(pose);
    pathSpline_msg.curvature.push_back(map[closest_idx].cur);
    pathSpline_msg.distance.push_back(cumulative_dist);
    path_rviz_msg.poses.push_back(pose);

    // Percorre sequencialmente até ao fim da reta (sem modulo/wraparound)
    for (std::size_t i = closest_idx + 1; i < map.size(); i++)
    {
        double d = distance(last_added_x, last_added_y, map[i].x, map[i].y);

        if (d >= 0.1)
        {
            pose = createPoseMsg(
                map[i].x, map[i].y,
                carData.roll, carData.pitch, carData.yaw, stamp);
            pathSpline_msg.poses.push_back(pose);
            pathSpline_msg.curvature.push_back(map[i].cur);

            cumulative_dist += d;
            pathSpline_msg.distance.push_back(cumulative_dist);
            path_rviz_msg.poses.push_back(pose);

            last_added_x = map[i].x;
            last_added_y = map[i].y;
        }
    }

    //track_correction(&pathSpline_msg, &path_rviz_msg);

    if (!pathSpline_msg.poses.empty())
    {
        path_control_pub->publish(pathSpline_msg);
        path_vis_pub->publish(path_rviz_msg);
    }
}

void acceleration_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    carData.car_x = msg->pose.position.x;
    carData.car_y = msg->pose.position.y;

    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(carData.roll, carData.pitch, carData.yaw);
    
    publishPath();

}

void acceleration_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    this->coneArray = msg;

    if (map_Localized)
        return;

    int blue_index = -1, yellow_index = -1;
    nearest_cone(msg, &blue_index, &yellow_index);

    if (blue_index == -1 || yellow_index == -1)
        return;

    const auto &cones_s = msg->cones;
    double bx = cones_s[blue_index].position.x;
    double by = cones_s[blue_index].position.y;
    double yx = cones_s[yellow_index].position.x;
    double yy = cones_s[yellow_index].position.y;

    double lane_width = distance(bx, by, yx, yy);
    if (lane_width < MIN_LANE_WIDTH || lane_width > MAX_LANE_WIDTH)
        return; // deteção espúria, ignora esta amostra

    double ref_x = (bx + yx) / 2.0;
    double ref_y = (by + yy) / 2.0;
    // Azul e a esquerda, amarelo a direita: a frente e perpendicular ao vetor azul->amarelo.
    double heading = std::atan2(yy - by, yx - bx) - M_PI / 2.0;

    sum_ref_x_ += ref_x;
    sum_ref_y_ += ref_y;
    sum_cos_ += std::cos(heading);
    sum_sin_ += std::sin(heading);
    localization_samples_++;

    RCLCPP_INFO(this->get_logger(), "Amostra de localização %zu/%zu", localization_samples_, LOCALIZATION_SAMPLES);

    if (localization_samples_ < LOCALIZATION_SAMPLES)
        return;

    double avg_ref_x = sum_ref_x_ / localization_samples_;
    double avg_ref_y = sum_ref_y_ / localization_samples_;
    double avg_heading = std::atan2(sum_sin_ / localization_samples_, sum_cos_ / localization_samples_);


    RCLCPP_INFO(this->get_logger(), "SUCESSO: Mapa ancorado (ref: %.2f, %.2f | heading: %.2f rad)",
                avg_ref_x, avg_ref_y, avg_heading);
}

void acceleration_node::track_correction(lart_msgs::msg::PathSpline *path, nav_msgs::msg::Path *path_vis)
{
    //VARIAVEIS DE CONTROLO
    const size_t NUMERO_DE_PONTOS = 45;
    const double PAIR_DISTANCE_CONTROL = 5; //Limite da distancia para ligar os cones
    const double ALPHA = 0.15;          //EMA FILTER
    const double MAX_CORRECTION = 0.40;    //Proteção contra guinadas
    const double DISTANCE_TO_LOOK_AHEAD = 2.5; // distancia a olhar para a frente para calcular a media



    if (!path || !coneArray)
        return;

    auto local_coneArray = coneArray;
    const auto &cones_s = local_coneArray->cones;

    if (cones_s.size() > 5000)
        return;

    if (cones_s.empty() || path->poses.empty())
        return;

    double soma_erro_x = 0.0;
    double soma_erro_y = 0.0;
    int pontos_validos = 0;

    double start_x = path->poses[0].pose.position.x;
    double start_y = path->poses[0].pose.position.y;

    size_t num_pontos = std::min((size_t)NUMERO_DE_PONTOS, path->poses.size());

    for (size_t i = 0; i < num_pontos; i++)
    {
        double pt_x = path->poses[i].pose.position.x;
        double pt_y = path->poses[i].pose.position.y;

        if (distance(start_x, start_y, pt_x, pt_y) > DISTANCE_TO_LOOK_AHEAD)
            break;

        std::pair<double, double> pose_pos = {pt_x, pt_y};

        int nearstCone_blue = -1;
        int nearstCone_yellow = -1;
        double blue_distnace = std::numeric_limits<double>::max();
        double yellow_distnace = std::numeric_limits<double>::max();

        for (size_t j = 0; j < cones_s.size(); j++)
        {
            double tmp_distance;
            if (cones_s[j].class_type.data == lart_msgs::msg::Cone::BLUE)
            {
                tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
                if (blue_distnace > tmp_distance)
                {
                    blue_distnace = tmp_distance;
                    nearstCone_blue = j;
                }
            }

            if (cones_s[j].class_type.data == lart_msgs::msg::Cone::YELLOW){
                tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
                if (yellow_distnace > tmp_distance){
                    yellow_distnace = tmp_distance;
                    nearstCone_yellow = j;
                }
            }
        }

        if (nearstCone_blue == -1 || nearstCone_yellow == -1){
            continue;
        }

        double pair_distance = distance(
            cones_s[nearstCone_blue].position.x, cones_s[nearstCone_blue].position.y,
            cones_s[nearstCone_yellow].position.x, cones_s[nearstCone_yellow].position.y
        );

        if (pair_distance > PAIR_DISTANCE_CONTROL)
            continue;

        double midPoint_x = (cones_s[nearstCone_blue].position.x + cones_s[nearstCone_yellow].position.x) / 2.0;
        double midPoint_y = (cones_s[nearstCone_blue].position.y + cones_s[nearstCone_yellow].position.y) / 2.0;

        soma_erro_x += (midPoint_x - pt_x);
        soma_erro_y += (midPoint_y - pt_y);
        pontos_validos++;
    }

    if (pontos_validos == 0)
    {
        RCLCPP_WARN(this->get_logger(), "[ACCELERATION CORRECTION] 0 pontos validos. Path inalterado.");
        return;
    }

    double erro_medio_x = soma_erro_x / pontos_validos;
    double erro_medio_y = soma_erro_y / pontos_validos;

    RCLCPP_INFO(this->get_logger(), "[ACCELERATION CORRECTION] Pts: %d | Erro Real: (X: %.2f, Y: %.2f)",
                pontos_validos, erro_medio_x, erro_medio_y);

    double filtered_corr_x = ALPHA * erro_medio_x + (1.0 - ALPHA) * this->prev_corr_x_;
    double filtered_corr_y = ALPHA * erro_medio_y + (1.0 - ALPHA) * this->prev_corr_y_;

    double corr_magnitude = std::sqrt(filtered_corr_x * filtered_corr_x + filtered_corr_y * filtered_corr_y);

    if (corr_magnitude > MAX_CORRECTION)
    {
        double scale = MAX_CORRECTION / corr_magnitude;
        filtered_corr_x *= scale;
        filtered_corr_y *= scale;
        RCLCPP_WARN(this->get_logger(), "[ACCELERATION CORRECTION] Limite MAX (%f) atingido!",MAX_CORRECTION);
    }

    RCLCPP_INFO(this->get_logger(), "[ACCELERATION CORRECTION] Aplicado: (X: %.2f, Y: %.2f) | Filtro_Mag: %.2fm",
                filtered_corr_x, filtered_corr_y, corr_magnitude);

    this->prev_corr_x_ = filtered_corr_x;
    this->prev_corr_y_ = filtered_corr_y;

    size_t total_poses = path->poses.size();
    for (size_t k = 0; k < total_poses; k++)
    {
        double decaimento = 1.0 - ((double)k / (double)total_poses);

        path->poses[k].pose.position.x += (filtered_corr_x * decaimento);
        path->poses[k].pose.position.y += (filtered_corr_y * decaimento);

        path_vis->poses[k].pose.position.x += (filtered_corr_x * decaimento);
        path_vis->poses[k].pose.position.y += (filtered_corr_y * decaimento);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<acceleration_node>());
    rclcpp::shutdown();
    return 0;
}
