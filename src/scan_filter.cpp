#include <functional>
#include <memory>
#include<cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ScanFilter : public rclcpp::Node 
{
  public:
    ScanFilter()
    : Node("scan_filter"), count_(0)
    {
      scan_data = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 100, std::bind(&ScanFilter::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("closest_object_distance", 100);
    }

  private:
  void topic_callback(const sensor_msgs::msg::LaserScan & msg) const
  {

       if (msg.ranges.empty()) {
        return;
        }
       double min_distance = std::numeric_limits<double>::max();
      int min_index = -1;
       for (size_t i = 0; i < msg.ranges.size(); ++i) {
          double range = msg.ranges[i];
          if (std::isfinite(range) && 
              range > 0.0 && 
              range >= msg.range_min && 
              range <= msg.range_max) 
              {
                if (range < min_distance) {
                    min_distance = range;
                    min_index = static_cast<int>(i);}
              }
       }
  
        if (min_index != -1) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = min_distance;
            publisher_->publish(distance_msg);
          
            RCLCPP_INFO(this->get_logger(), 
                       "Object detected at distance: %.2f m", 
                       min_distance);}
  }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_data;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFilter>());
  rclcpp::shutdown();
  return 0;
}
