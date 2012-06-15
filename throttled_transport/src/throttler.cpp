#include <boost/shared_array.hpp>
#include <ros/ros.h>


#include "throttled_transport/throttler.h"

namespace throttled_transport {

    Throttler::Throttler() {
        initialized = false;
        throttle_mode_ = THROTTLE_BY_FREQUENCY;
        max_freq_ = 0;
        max_bw_ = 0;
        used_bytes_ = 0;
        message_count_ = 0;
        max_bytes_reached_ = false;
    }

    bool Throttler::initialize() {
        // TODO: find what namespace to use
        ros::NodeHandle nh("~");

        std::string throttle_type;
        nh.param("throttled_transport/mode",throttle_type,throttle_type);
        if (throttle_type == "frequency") {
            throttle_mode_ = THROTTLE_BY_FREQUENCY;
        }
        if (throttle_type == "bandwidth") {
            throttle_mode_ = THROTTLE_BY_BANDWIDTH;
        }

        nh.param("throttled_transport/max_frequency",max_freq_,0.0);
        nh.param("throttled_transport/max_bandwidth",max_bw_,0.0);

        // TODO: create a service to configure this parameter ?
        throttling_server = nh.advertiseService("set_throttling_parameters",&Throttler::set_throttling_parameters, this);

        // TODO: use dynamic_reconfigure ?

        used_bytes_ = 0;
        message_count_ = 0;
        max_bytes_reached_ = false;
        last_publish_ = ros::Time::now();
        initialized = true;
        return true;
    }

    bool Throttler::set_throttling_parameters(SetThrottlingParameters::Request &req, 
            SetThrottlingParameters::Response &res) {
        if (req.throttling_mode == "frequency") {
            throttle_mode_ = THROTTLE_BY_FREQUENCY;
        }
        if (req.throttling_mode == "bandwidth") {
            throttle_mode_ = THROTTLE_BY_BANDWIDTH;
        }
        switch (throttle_mode_) {
            case THROTTLE_BY_FREQUENCY:
                max_freq_ = req.max_frequency;
                break;
            case THROTTLE_BY_BANDWIDTH:
                max_bw_ = req.max_bandwidth;
                break;
            default:
                break;
        }
        used_bytes_ = 0;
        message_count_ = 0;
        max_bytes_reached_ = false;
        last_publish_ = ros::Time::now();
        res.result = 0;
        return true;
    }

    bool Throttler::can_publish(size_t datasize) {
        if (!initialized) {
            initialize();
        }
        ros::Time now = ros::Time::now();
        double dt = (now - last_publish_).toSec();;
        message_count_ += 1;
        // ROS_INFO("Can publish?  dt %.3f mc %d mxf %.2f ub %d mxbw %.2f",
        //         dt,message_count_,max_freq_, used_bytes_, max_bw_);

        switch (throttle_mode_) {
            case THROTTLE_BY_FREQUENCY:
                if (max_freq_ <= 0) {
                    return false;
                }
                if ((1 / dt) < max_freq_) {
                    message_count_ = 0;
                    last_publish_ = now;
                    return true;
                }
                break;
            case THROTTLE_BY_BANDWIDTH:
                if (max_bw_ <= 0) {
                    return false;
                }
                if (max_bytes_reached_) {
                    if ((used_bytes_ / dt) < max_bw_) {
                        used_bytes_ = 0;
                        last_publish_ = now;
                        max_bytes_reached_ = false;
                    }
                    return false;
                } else {
                    if ((used_bytes_ / dt) < max_bw_) {
                        used_bytes_ += datasize;
                        return true;
                    } else {
                        max_bytes_reached_ = true;
                        return false;
                    }
                }
                break;
            default:
                break;
        }

        
        return false;
    }


};
