#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // PointCloud2 için gerekli header
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class SensorCleanerNode : public rclcpp::Node {
public:
    SensorCleanerNode() : Node("sensor_cleaner") {
        RCLCPP_INFO(this->get_logger(), "Sensor Cleaner Node Started");

        // Subscribers
        cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/front_camera_prius", 1, std::bind(&SensorCleanerNode::image_callback, this, _1));
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/center_lidar", 10000, std::bind(&SensorCleanerNode::lidar_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10000, std::bind(&SensorCleanerNode::imu_callback, this, _1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_sensor", 10000, std::bind(&SensorCleanerNode::gps_callback, this, _1));

        // PointCloud2 abone işlemi
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud_topic", 10, std::bind(&SensorCleanerNode::pointcloud_callback, this, _1));

        // Publishers
        cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/front_camera_prius_clean", 0.1);
        right_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/right_camera_prius", 10, std::bind(&SensorCleanerNode::right_image_callback, this, _1));
        left_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left_camera_prius", 10, std::bind(&SensorCleanerNode::left_image_callback, this, _1));
        right_cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera_prius_clean", 10);
        left_cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera_prius_clean", 10);

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/center_lidar_clean", 0.1);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_clean", 0.1);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps_sensor_clean", 0.1);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_clean", 0.1);

        // Rate control timer (frekans ayarı)
        this->declare_parameter<double>("publish_frequency", 0.5);
        double freq = this->get_parameter("publish_frequency").as_double();
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(500.0 / freq)),
            std::bind(&SensorCleanerNode::publish_cleaned_data, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;  // PointCloud2 abonesi

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;  // PointCloud2 yayıcı

    rclcpp::TimerBase::SharedPtr publish_timer_;

    sensor_msgs::msg::Image::SharedPtr last_image_;
    sensor_msgs::msg::LaserScan::SharedPtr last_lidar_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    sensor_msgs::msg::NavSatFix::SharedPtr last_gps_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_pointcloud_;  // Son point cloud verisi
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_cam_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_cam_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_cam_pub_;
    sensor_msgs::msg::Image::SharedPtr last_right_image_;
    sensor_msgs::msg::Image::SharedPtr last_left_image_;


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
            auto cleaned = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            last_image_ = cleaned;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion failed: %s", e.what());
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto cleaned = *msg;
        for (auto &range : cleaned.ranges) {
            if (!std::isfinite(range) || range < 0.1 || range > 30.0) {
                range = std::numeric_limits<float>::quiet_NaN();
            }
        }
        last_lidar_ = std::make_shared<sensor_msgs::msg::LaserScan>(cleaned);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = msg;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status >= 0) {
            last_gps_ = msg;
        }
    }

    // PointCloud2 mesajını işleyen callback fonksiyonu
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        last_pointcloud_ = msg;  // Veriyi sakla
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with %zu points", msg->data.size());
    }

    
    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
            auto cleaned = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            last_right_image_ = cleaned;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Right camera image conversion failed: %s", e.what());
        }
    }

    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
            auto cleaned = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            last_left_image_ = cleaned;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Left camera image conversion failed: %s", e.what());
        }
    }

    void publish_cleaned_data() {
        if (last_image_) cam_pub_->publish(*last_image_);
        if (last_lidar_) lidar_pub_->publish(*last_lidar_);
        if (last_imu_) imu_pub_->publish(*last_imu_);
        if (last_gps_) gps_pub_->publish(*last_gps_);
        
        if (last_right_image_) right_cam_pub_->publish(*last_right_image_);
        if (last_left_image_)  left_cam_pub_->publish(*last_left_image_);

        if (last_pointcloud_) {
            // PointCloud2 mesajını yayınla
            pointcloud_pub_->publish(*last_pointcloud_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorCleanerNode>());
    rclcpp::shutdown();
    return 0;
}

