#include "compressed_imagem_transport/compressed_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <boost/make_shared.hpp>

#include "compressed_image_transport/compression_common.h"

#include <vector>
#include <sstream>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;
namespace cit = compressed_image_transport;

namespace compressed_imagem_transport {


    void CompressedPublisher::configCb(Config& config, uint32_t level)
    {
        config_ = config;
    }



    void CompressedPublisher::publish(const sensor_msgs::Image& message,
            const message_transport::SimplePublisherPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>::PublishFn& publish_fn) const
    {
        // Compressed image message
        sensor_msgs::CompressedImage compressed;
        compressed.header = message.header;
        compressed.format = message.encoding;

        // Compression settings
        std::vector<int> params;
        params.resize(3, 0);

        // Get codec configuration
        cit::compressionFormat encodingFormat = cit::UNDEFINED;
        if (config_.format == "jpeg")
            encodingFormat = cit::JPEG;
        if (config_.format == "png")
            encodingFormat = cit::PNG;

        // Bit depth of image encoding
        int bitDepth = enc::bitDepth(message.encoding);
        int numChannels = enc::numChannels(message.encoding);

        switch (encodingFormat)
        {
            // JPEG Compression
            case cit::JPEG:
                {
                    params[0] = CV_IMWRITE_JPEG_QUALITY;
                    params[1] = config_.jpeg_quality;

                    // Update ros message format header
                    compressed.format += "; jpeg compressed";

                    // Check input format
                    if ((bitDepth == 8) && // JPEG only works on 8bit images
                            ((numChannels == 1) || (numChannels == 3)))
                    {

                        // Target image format
                        stringstream targetFormat;
                        if (enc::isColor(message.encoding))
                        {
                            // convert color images to RGB domain
                            targetFormat << "rgb" << bitDepth;
                        }

                        // OpenCV-ros bridge
                        cv_bridge::CvImagePtr cv_ptr;
                        try
                        {
                            cv_ptr = cv_bridge::toCvCopy(message, targetFormat.str());

                            // Compress image
                            if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
                            {

                                float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                                    / (float)compressed.data.size();
                                ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, (long unsigned int)compressed.data.size());
                            }
                            else
                            {
                                ROS_ERROR("cv::imencode (jpeg) failed on input image");
                            }
                        }
                        catch (cv_bridge::Exception& e)
                        {
                            ROS_ERROR("%s", e.what());
                        }
                        catch (cv::Exception& e)
                        {
                            ROS_ERROR("%s", e.what());
                        }

                        // Publish message
                        publish_fn(compressed);
                    }
                    else
                        ROS_ERROR("Compressed Image Transport - JPEG compression requires 8-bit, 1/3-channel images (input format is: %s)", message.encoding.c_str());

                    break;
                }
                // PNG Compression
            case cit::PNG:
                {
                    params[0] = CV_IMWRITE_PNG_COMPRESSION;
                    params[1] = config_.png_level;

                    // Update ros message format header
                    compressed.format += "; png compressed";

                    // Check input format
                    if (((bitDepth == 16) || (bitDepth == 8)) && ((numChannels == 1) || (numChannels == 3)))
                    {

                        // Target image format
                        stringstream targetFormat;
                        if (enc::isColor(message.encoding))
                        {
                            // convert color images to RGB domain
                            targetFormat << "rgb" << bitDepth;
                        }

                        // OpenCV-ros bridge
                        cv_bridge::CvImagePtr cv_ptr;
                        try
                        {
                            cv_ptr = cv_bridge::toCvCopy(message, targetFormat.str());

                            // Compress image
                            if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
                            {

                                float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                                    / (float)compressed.data.size();
                                ROS_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, (long unsigned int)compressed.data.size());
                            }
                            else
                            {
                                ROS_ERROR("cv::imencode (png) failed on input image");
                            }
                        }
                        catch (cv_bridge::Exception& e)
                        {
                            ROS_ERROR("%s", e.what());
                        }
                        catch (cv::Exception& e)
                        {
                            ROS_ERROR("%s", e.what());
                        }

                        // Publish message
                        publish_fn(compressed);
                    }
                    else
                        ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit, 1/3-channel images (input format is: %s)", message.encoding.c_str());
                    break;
                }

            default:
                ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
                break;
        }

    }

} //namespace compressed_imagem_transport
