#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <fsai_laps/msg/lap.hpp>

#include <geodesy/utm.h>

#include <optional>

geodesy::UTMPoint convert_msg_to_utm( const sensor_msgs::msg::NavSatFix &msg )
{
    geographic_msgs::msg::GeoPoint geoPoint;
    geoPoint.latitude = msg.latitude;
    geoPoint.longitude = msg.longitude;
    geoPoint.altitude = msg.altitude;

    return geodesy::UTMPoint( geoPoint );
}

float distance( const geodesy::UTMPoint &a, const geodesy::UTMPoint &b )
{
    return std::sqrt( std::pow( a.easting - b.easting, 2 ) + std::pow( a.northing - b.northing, 2 ) );
}

class GpsLaps : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
    rclcpp::Publisher<fsai_laps::msg::Lap>::SharedPtr pub_;

    std::optional<geodesy::UTMPoint> start_;

    float threshold_ = 3.f;
    int lap_ = 0;

    bool near_ = false;

public:
    GpsLaps() : Node("gps_laps")
    {
        this->sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&GpsLaps::callback_, this, std::placeholders::_1) );

        this->pub_ = this->create_publisher<fsai_laps::msg::Lap>( "laps", 10 );
    }

private:
    void callback_( const sensor_msgs::msg::NavSatFix &msg )
    {
        auto current = convert_msg_to_utm( msg );

        RCLCPP_INFO_STREAM( this->get_logger(), "Callback " << current.easting << " " << current.northing );

        // initialise the start point on first message
        if( !this->start_ )
        {
            RCLCPP_INFO( this->get_logger(), "Set start" );

            this->start_ = current;
            this->lap_ = 0;
            this->near_ = true;
        }
        
        const float dist = distance( this->start_.value(), current );

        /* have moved away from the start point */
        if( this->near_ && dist > this->threshold_ * 1.2f )
        {
            this->near_ = false;
            this->lap_ += 1;
        }
        /* have moved back into proximity with the start point */
        else if( !this->near_ && dist < this->threshold_ )
        {
            this->near_ = true;
        }

        fsai_laps::msg::Lap lap;
        lap.lap = this->lap_;
        lap.on_line = this->near_;
        lap.distance = dist;

        this->pub_->publish( lap );
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsLaps>());
  rclcpp::shutdown();
  return 0;
}