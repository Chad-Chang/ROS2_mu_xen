#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{

public:
    TestPublisher()
    : Node("test_pub_node"), count_(0)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        test_pub_node_ = this -> create_publisher<std_msgs::msg::String>(
            "test", qos_profile);
        timer_ = this -> create_wall_timer(
            1ms,std::bind(&TestPublisher::publish_msg, this)
        );
    }
private:
    void publish_msg()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "test msg : " + std::to_string(count_);
        if(count_ < 2000 ) 
        {
            count_++;
            RCLCPP_INFO(this -> get_logger(), "Published message: '%s'", msg.data.c_str() );        
            test_pub_node_ -> publish(msg);
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_node_;
    size_t count_;
} ;

int main(int argc, char * argv[])
 // c++ 표준 : 
 // argc = 명령줄에 전달되는 인수의 갯수
 // argv: argument vector 명령줄 이수 문자열 저장한 포인터 : argv[0]은 실행된 프로그램의 경로또는 이름을 포함하고 있음.
    // 뒤로는 명령줄에서 전달된인수들이 순서대로 담겨잇음.
// 왜 하는가 = ROS2에서 런타임이 명령줄 인수 분석 + 이를 기반으로 설정 변경 가능
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}