#ifndef THROTTLER_H
#define THROTTLER_H

#include <ros/ros.h>
#include "throttled_transport/SetThrottlingParameters.h"

namespace throttled_transport {

    class Throttler {
        public:
            Throttler(); 
            ~Throttler() {}

            /** Returns true if we can now publish a message of size datasize
             * The decision on when to publish will be based on
             * configuration parameters, exposed through rosparam
             * */
            bool can_publish(size_t datasize);
        protected:
            bool initialized;
            bool initialize();

            ros::ServiceServer throttling_server;
            bool set_throttling_parameters(SetThrottlingParameters::Request &req, 
                    SetThrottlingParameters::Response &res);

        protected:
            enum {
                THROTTLE_BY_FREQUENCY,
                THROTTLE_BY_BANDWIDTH
            } throttle_mode_;

            ros::Time last_publish_;

            unsigned int message_count_;
            double max_freq_;

            unsigned int used_bytes_;
            double max_bw_;
            bool max_bytes_reached_;
    };


}; // throttled_transport



#endif // THROTTLER_H
