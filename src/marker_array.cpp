#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MarkerArrayPublisher : public rclcpp::Node {
public:
    MarkerArrayPublisher() : Node( "marker_array_publisher" ), m_count( 0 )
    {
        this->declare_parameter( "frame", "base_link" );
        m_frame     = this->get_parameter( "frame" ).as_string();
        m_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>( "marker_array", 10 );
        m_timer     = this->create_wall_timer( 100ms, std::bind( &MarkerArrayPublisher::timer_callback, this ) );
    }

private:
    void timer_callback()
    {
        auto message = visualization_msgs::msg::MarkerArray();
        RCLCPP_INFO( this->get_logger(), "Publishing message '%zu' to frame %s", m_count, m_frame.c_str());
        message.markers.resize( 10 );
        for( int i = 0; i < 10; ++i ) {
            message.markers[i].header.stamp       = now();
            message.markers[i].header.frame_id    = m_frame;
            message.markers[i].id                 = i;
            message.markers[i].ns                 = "marker_array";
            message.markers[i].lifetime           = rclcpp::Duration( 100ms );
            message.markers[i].action             = visualization_msgs::msg::Marker::MODIFY;
            message.markers[i].color.a            = 1.0f;
            message.markers[i].color.b            = 1.0f;
            message.markers[i].pose.position.x    = 10 * i;
            message.markers[i].pose.position.y    = 10;
            message.markers[i].pose.position.z    = 0;
            message.markers[i].pose.orientation.x = 0;
            message.markers[i].pose.orientation.y = 0;
            message.markers[i].pose.orientation.z = 0;
            message.markers[i].pose.orientation.w = 1;
            message.markers[i].scale.x            = 5;
            message.markers[i].scale.y            = 5;
            message.markers[i].scale.z            = 5;
            message.markers[i].type               = visualization_msgs::msg::Marker::CUBE;
            message.markers[i].frame_locked       = false;
        }
        m_publisher->publish( message );
        m_count++;
    }

    rclcpp::TimerBase::SharedPtr                                       m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_publisher;
    size_t                                                             m_count;
    std::string                                                        m_frame;
};

int
main( int argc, char* argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<MarkerArrayPublisher>() );
    rclcpp::shutdown();
    return 0;
}