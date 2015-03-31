#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar/AlvarMarkers.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1, 
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
        
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
const std::string windowName = "Original Image";
const std::string windowName1 = "HSV Image";
const std::string windowName2 = "Thresholded Image";
const std::string windowName3 = "After Morphological Operations";
const std::string trackbarWindowName = "Trackbars";

void tagCallback(ar_track_alvar::AlvarMarkersConstPtr markerList)
{
    for(int i = 0; i < markerList->markers.size(); i++)
    {
        ar_track_alvar::AlvarMarker marker = markerList->markers[i];
        if(marker.id == 0)
        {
            printf("Found marker 0\nx: %f\ty: %f\tz: %f\nqx: %f\tqy: %f\tqz: %f\tqw: %f\n\n",
                                                                marker.pose.pose.position.x,
                                                                marker.pose.pose.position.y,
                                                                marker.pose.pose.position.z,
                                                                marker.pose.pose.orientation.x,
                                                                marker.pose.pose.orientation.y,
                                                                marker.pose.pose.orientation.z,
                                                                marker.pose.pose.orientation.w
                    );
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    //ImageConverter ic;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imagePub;
    tf::TransformBroadcaster tfBroadcaster;
    tf::Transform transform;
    //ros::Subscriber tagSub = nh.subscribe("/ar_pose_marker", 1, tagCallback);
    ros::Publisher infoPub = nh.advertise<sensor_msgs::CameraInfo>("/hermes/camera_info", 1);
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);
    unsigned int infoSeq = 0;

    cv::Mat HSV;
    cv::Mat cameraFeed;
    cv::Mat threshold;
    cv::VideoCapture capture;
    capture.open(1);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    imagePub = it.advertise("/hermes/camera", 1);
    
    
    printf("init D\n");
    info->D = std::vector<double>();
    info->D.resize(5);
    info->D[0] = .195608;
    info->D[1] = -.661196;
    info->D[2] = .000777;
    info->D[3] = .000469;
    info->D[4] = 0.0;
    
    printf("init R\n");
    info->R[0] = 1.0;
    info->R[1] = 0.0;
    info->R[2] = 0.0;
    info->R[3] = 0.0;
    info->R[4] = 1.0;
    info->R[5] = 0.0;
    info->R[6] = 0.0;
    info->R[7] = 0.0;
    info->R[8] = 1.0;
    
    printf("init K\n");
    info->K[0] = 620.397948;
    info->K[1] = 0.0;
    info->K[2] = 321.047461;
    info->K[3] = 0.0;
    info->K[4] = 619.776573;
    info->K[5] = 234.233368;
    info->K[6] = 0.0;
    info->K[7] = 0.0;
    info->K[8] = 1.0;
    
    printf("init P\n");
    info->P[0] = 622.955261;
    info->P[1] = 0.0;
    info->P[2] = 320.885445;
    info->P[3] = 0.0;
    info->P[4] = 0.0;
    info->P[5] = 627.469971;
    info->P[6] = 233.947018;
    info->P[7] = 0.0;
    info->P[8] = 0.0;
    info->P[9] = 0.0;
    info->P[10] = 1.0;
    info->P[11] = 0.0;


    while(ros::ok())
    {
        capture.read(cameraFeed);
        cv::cvtColor(cameraFeed,HSV,cv::COLOR_BGR2HSV);
        cv::inRange(HSV,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),threshold);
        
        cv::imshow(windowName,cameraFeed);
        
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id     = "camera_link";
        out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
        out_msg.image        = cameraFeed; // Your cv::Mat

        info->header.seq = infoSeq;
        info->header.stamp = ros::Time::now();
        info->header.frame_id = "camera_link";
        info->width = FRAME_WIDTH;
        info->height = FRAME_HEIGHT;
        
        transform.setOrigin(tf::Vector3(0, 0, 0.2));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));

        transform.setOrigin(tf::Vector3(0.0, 0.1, 0.1));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/camera_link"));

        imagePub.publish(out_msg.toImageMsg());
        infoPub.publish(info);
        
        infoSeq++;
        ros::spinOnce();

        cv::waitKey(30);
    }


    ros::spin();
    return 0;
}

/*#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    //ImageConverter ic;
    ros::spin();
    return 0;
}*/
