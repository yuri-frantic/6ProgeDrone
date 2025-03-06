#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


#include <thread>
#include <chrono>

class FlightCommander {
public:
    FlightCommander(): spinner_(2) {
        // Подписка на целевые координаты
        //target_pose_sub_ = nh_.subscribe("/vehicle/desPose", 10, &FlightCommander::poseCallback, this);    
        // Подписка на изображение с камеры
        image_sub_ = nh_.subscribe("/iris/camera1/image_raw", 10, &FlightCommander::imageCallback, this);
        // Подписываемся на состояние ЛА
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &FlightCommander::state_cb, this);
        // Подписываемся текущее положение ЛА
        //pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &FlightCommander::pose_cb, this);
        // Инициализируем publisher для целевого состояния ЛА
        local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        // Инициализируем таймер для отправки целевого состояния
        //offboard_timer_ = nh_.createTimer(ros::Duration(0.05), &FlightCommander::offboard_timer_cb, this); // 20 Hz
        //offboard_timer_.stop(); // Останавливаем таймер до перехода в режим OFFBOARD
        // (AsyncSpinner - это класс, который используется для асинхронного выполнения коллбеков (callback) из очереди сообщений
        // аналогично ros::spin() или ros::spinOnce()
        // Запускаем асинхронный спинер
        spinner_.start();  
    }
//jghjg
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
    	float markerSize = 0.114;
    	float distanceBetweenMarkers = 0.362; // Расстояние между маркерами в метрах
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
            		//cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 10); // Рисуем оси
            	}

            	// Отправляем информацию на автопилот
            	calculate_current_pose(rvecs, tvecs);
        	//}
    	}

    	// Отображение результата
    	//cv::imshow("Image", image);
    	//cv::waitKey(1);
    }
    
    void calculate_current_pose(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    	// Логика преобразования координат и отправка данных на автопилот
  	geometry_msgs::PoseStamped local_pos_msg;

  	// Преобразование в систему координат БПЛА
  	current_position_x = tvec[1]*2 - 3.5; // X координата
  	current_position_y = tvec[0]*2 + 6; // Y координата
  	current_position_z = tvec[2]; // Z координата
  	

  	// Здесь можно добавить преобразование углов Эйлера из вектора вращения rvec в кватернионы или углы Эйлера
  	// Например:
  	// double roll = rvec[0];
    	// double pitch = rvec[1];
    	// double yaw = rvec[2];
	
   	//local_pos_pub.publish(local_pos_msg);
    }


    void run() {
        // Ожидаем подключения к автопилоту
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        // Запускаем режим offboard
        offboard_enable(true);

        // Запускаем режим АРМ
        if (!current_state_.armed)
            arm_vehicle(true);

          
        //auto start_time = ros::Time::now();
        
        
        //Взлет на 15 метров.
        takeoff();
              
        
        

        //Запускаем полет по кругу
        while (ros::ok()) {
              rate.sleep();
              calculate_velocitys();
              local_pos_pub_.publish(setpoint_); 
              
        }     
 
        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_pos_pub_;
    ros::Timer offboard_timer_;
    ros::Subscriber image_sub_;
   
    double desired_position_x = 0;
    double desired_position_y = 0;
    double desired_position_z = 0;
    double current_position_x = 0;
    double current_position_y = 0;
    double current_position_z = 0;

    // Коэффициенты П-регулятора для осей
    double kp_position_x = 0.7; 
    double kp_position_y = 0.7;
    double kp_position_z = 0.7;
    
    double counter = 0;
    double counterStep = 0.01;
    double circleRadius = 2;	
     

    
    ros::Rate rate = ros::Rate(20.0);
    mavros_msgs::State current_state_;
    mavros_msgs::PositionTarget setpoint_;
    geometry_msgs::PoseStamped current_pose_;
    ros::AsyncSpinner spinner_;
    
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 480, 0, 0, 1); // Примерные параметры камеры
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // Предполагаем, что искажений нет

    void takeoff() {
        
        setpoint_.position.x = 0;
        setpoint_.position.y = 0;
        setpoint_.position.z = 3;
        setpoint_.yaw = 0;
        // Устанавливаем систему координат в рамках которой будет отправляться команда
        setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // Установим битовую маску для полета по точкам
        setpoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
        | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
    	for (int i = 0; i < 200; i++) {
    	    if (ros::ok()) {
              //std::cout << "X=" << current_position_x << " Y=" << current_position_y << " Z=" << current_position_z << " | " << std::endl;
    
              //std::cout << "!!!!" << std::endl;
              local_pos_pub_.publish(setpoint_); 
              rate.sleep();
              if (current_position_z > 4 ) {
              	//ROS_INFO("TakeOff completed.");
              }
            }  
        }
    	//ROS_INFO("FOR completed.");
    
    
    
    
    }








    //  Установка целевоой позиции
    void calculate_desired_pose() {
        // Расчет круговых координат
	desired_position_z = 3;
	desired_position_x = circleRadius * std::cos(counter);
	desired_position_y = circleRadius * std::sin(counter);
	counter += counterStep;
    }
    //  Получение текущего состояния дрона
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    //  Получение текущих координат дрона
//    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//        current_pose_ = *msg;
//    }


    void calculate_velocitys() {
        // Устанавливаем систему координат в рамках которой будет отправляться команда
        setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // Установим битовую маску где покажем что должен выполнить автопилот, полет в точку или набор заданной скорости, ускорения, угла угловой скорости.
        setpoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        calculate_desired_pose();
        
        
        // РАСЧЕТ скоростей       
        double position_error_x = desired_position_x - current_position_x;
        double position_error_y = desired_position_y - current_position_y;
        double position_error_z = desired_position_z - current_position_z;
        

        // Применение П-регулятора для линейной скорости
        double target_velocity_x = kp_position_x * position_error_x;
        double target_velocity_y = kp_position_y * position_error_y;
        double target_velocity_z = kp_position_z * position_error_z;
 

        setpoint_.velocity.x = target_velocity_x;
        setpoint_.velocity.y = target_velocity_y;   //setpoint_.velocity.y = target_velocity_y;
        setpoint_.velocity.z = target_velocity_z;

        //std::cout << "Xd=" << desired_position_x << " Yd=" << desired_position_y << " Zd=" << desired_position_z << " | Xc=" << current_position_x << " Yc=" << current_position_y << " Zc=" << current_position_z << " | Vx=" << target_velocity_x << " Vy=" << target_velocity_y << " Vz=" << target_velocity_z << std::endl;     
       
    }
    
    // Арм аппарата
    bool arm_vehicle(bool arm = true) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        
        ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        return arming_client.call(arm_cmd) && arm_cmd.response.success;
    }

    // Смена режима полета
    bool change_mode(const std::string& mode) {
        mavros_msgs::SetMode sm;
        sm.request.custom_mode = mode;

        ros::ServiceClient client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        return client.call(sm) && sm.response.mode_sent;
    }
    
    // Активация автоматического режима полета
    void offboard_enable(bool enable) {
        //if (!enable) {
        //    offboard_timer_.stop();
       //     return;
       // }

        // Запускаем таймер отправки целевого положения
        //offboard_timer_.start();
        // Немного ждем пока сообщения начнут приходить на автопилот
        ros::Rate rate(1.0);
        rate.sleep();
        // Переключение режима на OFFBOARD
        change_mode("OFFBOARD");
    }

    // Таймер отправки целевого положения
    //void offboard_timer_cb(const ros::TimerEvent&) {
    //    setpoint_.header.stamp = ros::Time::now();
        //local_pos_pub_.publish(setpoint_);
    //}

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "flight_commander_node");
    FlightCommander commander;
    commander.run();
    return 0;
}




