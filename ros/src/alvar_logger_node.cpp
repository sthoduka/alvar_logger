#include <alvar_logger/alvar_logger_node.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

AlvarLoggerNode::AlvarLoggerNode(ros::NodeHandle &nh) : nh_(nh), it_(nh), frame_number_(0)
{
    std::string camera_info_topic;
    if (!nh_.getParam("camera_info_topic", camera_info_topic))
    {
        ROS_ERROR("Camera info topic needs to be specified");
        exit(0);
    }

    cam_ = new alvar::Camera(nh_, camera_info_topic);

    std::string log_file_name;
    if (!nh_.getParam("log_file", log_file_name))
    {
        ROS_ERROR("Log file needs to be specified");
        exit(0);
    }
    
    if (!nh_.hasParam("video_file"))
    {
        ROS_ERROR("Video file needs to be specified");
        exit(0);
    }

    file_out_.open(log_file_name.c_str());

    image_publisher_ = it_.advertise("detected_markers", 1);
    image_subscriber_ = it_.subscribe("input_image", 0, &AlvarLoggerNode::imageCallback, this);
}

AlvarLoggerNode::~AlvarLoggerNode()
{
    file_out_.close();
    delete cam_;
}

void AlvarLoggerNode::imageCallback(const sensor_msgs::ImageConstPtr &image)
{   
    if (cam_->getCamInfo_)
    {
        if (!video_writer_.isOpened())
        {
            cv::Size image_size(image->width, image->height);
            double frame_rate = 30.0;
            std::string video_file_name;
            nh_.getParam("video_file", video_file_name);
            video_writer_.open(video_file_name, CV_FOURCC('D','I','V','X'), frame_rate, image_size, true);
        }
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        IplImage ipl_image = cv_image->image;
        marker_detector_.Detect(&ipl_image, cam_);                

        video_writer_.write(cv_image->image);

        cv::Mat debug_image;
        cv_image->image.copyTo(debug_image);

        for (size_t i = 0; i < marker_detector_.markers->size(); i++)
        {
            int id = (*(marker_detector_.markers))[i].GetId();
            std::vector<alvar::PointDouble> marker_corners = (*(marker_detector_.markers))[i].marker_corners_img;
            for (int i = 0; i < marker_corners.size() - 1; i++)
            {
                cv::Point start_point(marker_corners.at(i).x, marker_corners.at(i).y);
                cv::Point end_point(marker_corners.at(i + 1).x, marker_corners.at(i + 1).y);
                cv::line(debug_image, start_point, end_point, CV_RGB(255, 0, 0), 3, CV_AA, 0);
                if (i == 0)
                {
                    file_out_ << id << ", " << frame_number_ << ", " << start_point.x << ", " << start_point.y;
                }
                else
                {
                    file_out_ << ", " << start_point.x << ", " << start_point.y;
                }
            }
            cv::Point start_point(marker_corners.at(marker_corners.size() - 1).x, marker_corners.at(marker_corners.size() - 1).y);
            cv::Point end_point(marker_corners.at(0).x, marker_corners.at(0).y);
            cv::line(debug_image, start_point, end_point, CV_RGB(255, 0, 0), 3, CV_AA, 0);
            file_out_ << ", " << start_point.x << ", " << start_point.y << std::endl;
        }
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        debug_image_msg.image = debug_image;
        image_publisher_.publish(debug_image_msg.toImageMsg());
    }
    
    frame_number_++;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "alvar_logger");

    ros::NodeHandle n("~");

    ROS_INFO("[alvar_logger] node started");

    AlvarLoggerNode alvar_logger_node(n); 

    ros::spin();

    return 0;
}
