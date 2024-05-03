#include <chrono>
#include <functional>
#include <memory>
#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher() : Node( "marker_publisher" ), m_count( 0 )
    {
        this->declare_parameter( "frame", "base_link" );
        m_frame = this->get_parameter( "frame" ).as_string();
        for( int i = 0; i < 10; ++i ) {
            m_publisher.push_back( this->create_publisher<visualization_msgs::msg::Marker>( "marker_" + std::to_string( i ), 10 ) );
        }
        m_timer = this->create_wall_timer( 100ms, std::bind( &MarkerPublisher::timer_callback, this ) );
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO( this->get_logger(), "Publishing message '%zu' to frame %s", m_count, m_frame.c_str());
        for( int i = 0; i < 10; ++i ) {
            auto message               = visualization_msgs::msg::Marker();
            message.header.stamp       = now();
            message.header.frame_id    = m_frame;
            message.id                 = 0;
            message.ns                 = "marker";
            message.lifetime           = rclcpp::Duration( 100ms );
            message.action             = visualization_msgs::msg::Marker::MODIFY;
            message.color.a            = 1.0f;
            message.color.r            = 1.0f;
            message.pose.position.x    = 10 * i;
            message.pose.position.y    = -10;
            message.pose.position.z    = 0;
            message.pose.orientation.x = 0;
            message.pose.orientation.y = 0;
            message.pose.orientation.z = 0;
            message.pose.orientation.w = 1;
            message.scale.x            = 5;
            message.scale.y            = 5;
            message.scale.z            = 5;
            message.type               = visualization_msgs::msg::Marker::CUBE;
            message.frame_locked       = false;
            m_publisher[i]->publish( message );
        }
        m_count++;
    }

    rclcpp::TimerBase::SharedPtr                                               m_timer;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> m_publisher;
    size_t                                                                     m_count;
    std::string                                                                m_frame;
};

int
main( int argc, char* argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<MarkerPublisher>() );
    rclcpp::shutdown();
    return 0;
}