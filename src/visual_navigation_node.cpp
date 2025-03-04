#include <ros/ros.h>
#include <mavros/mavros.h>
//#include <mavros/setpoint_position/local.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class DroneController {
public:
    DroneController() {
        // Инициализация подписчиков и публикаций
        image_sub = nh.subscribe("/iris/camera1/image_raw", 10, &DroneController::imageCallback, this);
        
        
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Обработка изображения для поиска маркеров
        processImage(cv_ptr->image);
    }

    void processImage(cv::Mat& image) {
    	std::vector<int> markerIds;
    	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;


    	// Определение маркеров
    	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    	float distanceBetweenMarkers = 0.7f; // Расстояние между маркерами в метрах
    	cv::Ptr<cv::aruco::Board> board = cv::aruco::GridBoard::create(10, 10, markerSize, distanceBetweenMarkers, dictionary);
    	
    	
    	cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, cv::aruco::DetectorParameters::create(), rejectedCandidates);

    	if (markerIds.size() > 0) {
        	// Оцениваем позу маркеров
        	cv::Vec3d rvecs, tvecs; // Векторы поворота и трансляции
        	//std::cout << "????????????????" << std::endl;
        	int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvecs, tvecs);
        	// Рисуем маркеры и их оси
        	if (valid > 0) {
            		cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
            		cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 10); // Рисуем оси
            	}

            	// Отправляем информацию на автопилот
            	sendToAutopilot(rvecs, tvecs);
        	//}
    	}

    	// Отображение результата
    	cv::imshow("Image", image);
    	cv::waitKey(1);
    }

    void sendToAutopilot(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    	// Логика преобразования координат и отправка данных на автопилот
  	geometry_msgs::PoseStamped local_pos_msg;

  	// Преобразование в систему координат БПЛА
  	local_pos_msg.pose.position.x = tvec[0]; // X координата
  	local_pos_msg.pose.position.y = tvec[1]; // Y координата
  	local_pos_msg.pose.position.z = tvec[2]; // Z координата
  	std::cout << "X=" << tvec[0] << " Y=" << tvec[1] << " Z=" << tvec[2] << std::endl;

  	// double roll = rvec[0];
    	// double pitch = rvec[1];
    	// double yaw = rvec[2];
	
   	//local_pos_pub.publish(local_pos_msg);
    }




 
    
private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher local_pos_pub;
    
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 480, 0, 0, 1); // Примерные параметры камеры
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // Предполагаем, что искажений нет


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_controller");
    DroneController drone_controller;
    ros::spin();
    return 0;
}

