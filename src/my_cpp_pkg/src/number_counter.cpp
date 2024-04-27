#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), count(0)
    {
        subscriber = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "Number counter has started");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr numberMsg)
    {
        this->count += numberMsg->data;
        auto msg = example_interfaces::msg::Int64();
        msg.data = this->count;
        publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing count");
    }

    int count;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

