#ifndef ALVAR_LOGGER_NODE_H_
#define ALVAR_LOGGER_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ar_track_alvar/MarkerDetector.h>

class AlvarLoggerNode
{
    public:
        AlvarLoggerNode(ros::NodeHandle &nh);

        virtual ~AlvarLoggerNode();

        void imageCallback(const sensor_msgs::ImageConstPtr &image);

    private:

        ros::NodeHandle nh_;

        image_transport::ImageTransport it_;

        image_transport::Subscriber image_subscriber_;

        image_transport::Publisher image_publisher_;

        std::ofstream file_out_;

        cv::VideoWriter video_writer_;

        int frame_number_;

        alvar::Camera *cam_;

        alvar::MarkerDetector<alvar::MarkerData> marker_detector_;                   
};

#endif
