#include "depth_online/depth_online_node.hpp"


clock_t start,end;
double duration;
std::ofstream outfile;
/**
 * @brief Setting up Tensorrt logger
*/
class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // Only output logs with severity greater than warning
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
}logger;

// modify the path to your model!!
DepthAnything depth_model("/media/ubun/DATA/Projects/reference_code/Depth-Anything/onnx/version14/vits/depth_anything_vits14.engine", logger);


void image_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    // 处理图像数据
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;
    start = clock();
    cv::Mat result_d = depth_model.predict(img);
    end=clock();
    duration = (double)(end-start)/CLOCKS_PER_SEC;
    outfile << duration << std::endl;
    cv::imshow("result_d",result_d);
    cv::waitKey(1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_online_node");
    ros::NodeHandle nh;

    nh.param<std::string>("common/ImageTopic",ImageTopic,"");

    outfile.open("/media/ubun/DATA/Projects/calibration/Code/CalibOnline/src/calib_online/src/time.txt");


    ros::Subscriber sub_image = nh.subscribe(ImageTopic, 10, image_callback);
    ros::spin();
    
    return 0;


}