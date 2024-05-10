#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), count(0)
    {
        subscriber = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        server = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter",
                std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Number counter has started");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr numberMsg)
    {
        this->count += numberMsg->data;
        auto msg = example_interfaces::msg::Int64();
        msg.data = this->count;
        publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing count %ld", msg.data);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data) {
            this->count = 0;
            response->message = "Reset count to zero";
        } else {
            response->message = "No change";
        }
        RCLCPP_INFO(this->get_logger(), "Reset counter: %s", response->message.c_str());
    }

    int count;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

