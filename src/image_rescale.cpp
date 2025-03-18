#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class ImageRescaler
{
public:
    ImageRescaler(ros::NodeHandle &nh) : it_(nh)
    {
        // Criar a assinatura do tópico de entrada
        sub_ = it_.subscribe("image_in", 10, &ImageRescaler::imageCallback, this);

        // Criar o publisher para a imagem reescalada
        pub_ = it_.advertise("image_out", 10);
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        // Converter a mensagem ROS para OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Redimensionar a imagem para 800x600
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(800, 600), cv::INTER_LINEAR);

        // Converter a imagem OpenCV de volta para mensagem ROS
        sensor_msgs::ImagePtr rescaled_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, resized_image).toImageMsg();

        // Publicar a imagem reescalada
        pub_.publish(rescaled_msg);
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_rescaler");
    ros::NodeHandle nh;

    ImageRescaler rescaler(nh);
    ros::spin();
    
    return 0;
}
