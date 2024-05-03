#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BaseLinkPublisher : public rclcpp::Node {
public:
    BaseLinkPublisher() : Node( "base_link_publisher" ), m_count( 0 )
    {
        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
        m_timer          = this->create_wall_timer( 100ms, std::bind( &BaseLinkPublisher::timer_callback, this ) );
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        RCLCPP_INFO( this->get_logger(), "Publishing: '%zu'", m_count );
        t.header.stamp            = now();
        t.header.stamp.nanosec += 50000;
        t.header.frame_id         = "odom";
        t.child_frame_id          = "base_link";
        t.transform.translation.x = -10;

        m_tf_broadcaster->sendTransform( t );
        m_count++;
    }

    rclcpp::TimerBase::SharedPtr                   m_timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    size_t                                         m_count;
};

int
main( int argc, char* argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<BaseLinkPublisher>() );
    rclcpp::shutdown();
    return 0;
}