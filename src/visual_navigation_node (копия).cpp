#include <ros/ros.h>
#include <mavros/mavros.h>
//#include <mavros/setpoint_position/global.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>
#include <vector>

class DroneController {
public:
    DroneController() {
        // Инициализация подписчиков и публикаций
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &DroneController::state_cb, this);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        ros::Duration(1.0).sleep(); // Ждем немного для установки соединения

        // Инициализация камеры
        cap.open(0); // Открываем камеру (индекс 0)
        if (!cap.isOpened()) {
            ROS_ERROR("Cannot open camera");
            exit(1);
        }
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }
    
    // Функция для создания PoseStamped
    geometry_msgs::PoseStamped createPoseStamped(double x, double y, double z) {
    	geometry_msgs::PoseStamped pose;
    	pose.header.frame_id = "map"; // Укажите нужный фрейм
    	pose.header.stamp = ros::Time::now();
    	pose.pose.position.x = x;
    	pose.pose.position.y = y;
    	pose.pose.position.z = z;
    	pose.pose.orientation.w = 1.0; // Кватернион (без вращения)
    	pose.pose.orientation.x = 0.0;
    	pose.pose.orientation.y = 0.0;
    	pose.pose.orientation.z = 0.0;
    
    	return pose;
    }

    void run() {
        // Устанавливаем начальные параметры
        ros::Rate rate(20.0);
        while (ros::ok() && !current_state.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        // Взлет
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_footprint";
        pose.header.stamp = ros::Time::now();

        for (int i = 0; i < 100; ++i) {
            pose.pose.position.z = 2.0; // Высота взлета 2 метра
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        // Полет по заданным координатам с ориентацией по маркерам
        std::vector<geometry_msgs::PoseStamped> waypoints = {
        createPoseStamped(0.0, 0.0, 2.0),
        createPoseStamped(2.0, 0.0, 2.0),
        createPoseStamped(2.0, 2.0, 2.0),
        createPoseStamped(0.0, 2.0, 2.0)
    };

        for (const auto& waypoint : waypoints) {
            pose.pose.position = waypoint.pose.position;

            // Ориентация по маркерам
            cv::Mat frame;
            while (true) {
                cap >> frame; // Считываем кадр с камеры
                if (frame.empty()) break;

                std::vector<int> marker_ids;
                std::vector<std::vector<cv::Point2f>> marker_corners;
                cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
                cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids);

                if (!marker_ids.empty()) {
                    // Предположим, что мы используем первый найденный маркер для ориентации
                    int marker_id = marker_ids[0];
                    cv::Mat rvec, tvec;
                    cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.1, cameraMatrix, distCoeffs, rvec, tvec);

                    // Преобразуем в углы Эйлера или кватернионы для управления ориентацией БПЛА
                    double roll = rvec.at<double>(0);
                    double pitch = rvec.at<double>(1);
                    double yaw = rvec.at<double>(2);

                    // Устанавливаем ориентацию в pose
                    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                }

                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();

                // Условие выхода из цикла при достижении точки
                if (reachedWaypoint(pose.pose.position, waypoint.pose.position)) {
                    break;
                }
            }
        }

        // Посадка
        for (int i = 0; i < 100; ++i) {
            pose.pose.position.z = 0.1; // Высота для посадки
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        // Остановка БПЛА
        pose.pose.position.z = 0.0; // На земле
        local_pos_pub.publish(pose);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    mavros_msgs::State current_state;
    cv::VideoCapture cap;

    cv::Mat cameraMatrix; //  Задайте матрицу камеры
    cv::Mat distCoeffs;   // Задайте коэффициенты искажений

    bool reachedWaypoint(const geometry_msgs::Point& current_pos, const geometry_msgs::Point& waypoint) {
        double distance = sqrt(pow(current_pos.x - waypoint.x, 2) + pow(current_pos.y - waypoint.y, 2) + pow(current_pos.z - waypoint.z, 2));
        return distance < 0.5; // Порог достижения точки
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_controller");
    DroneController drone_controller;
    drone_controller.run();
    return 0;
}

