// the following code is based on this tutorial: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages#The_Code
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>

bool tmp = false;
int argc;
char **argv;

std::string save_image = "/home/ricky/catkin_ws/src/RoboticProject/ur_vision/src/image.jpeg";

using json = nlohmann::json;

struct Prediction
{
  double x;
  double y;
  std::string classe;
};

static std::string toString(const Eigen::Vector3f mat)
{
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

cv::Mat ritagliaImmagine(const cv::Mat &immagine, int x, int y, int larghezza, int altezza)
{
  cv::Rect rettangoloRitaglio(x, y, larghezza, altezza);
  cv::Mat immagineRitagliata = immagine(rettangoloRitaglio);
  return immagineRitagliata;
}

std::vector<Prediction> extractIntegerWords(const std::string str)
{
  std::vector<Prediction> valoriXYClass;

  try
  {
    nlohmann::json json = nlohmann::json::parse(str);

    for (const auto &prediction : json["predictions"])
    {
      double x = prediction["x"];
      double y = prediction["y"];
      std::string classe = prediction["class"];

      Prediction valoreXYClass;
      valoreXYClass.x = x;
      valoreXYClass.y = y;
      valoreXYClass.classe = classe;

      valoriXYClass.push_back(valoreXYClass);
    }
  }
  catch (const nlohmann::json::exception &e)
  {
    std::cout << "Errore nella lettura del JSON: " << e.what() << std::endl;
  }

  return valoriXYClass;
}

std::string exec(const char *cmd)
{
  std::array<char, 1024> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
  {
    result += buffer.data();
  }
  return result;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  std::cout << "tmp : " << tmp << std::endl;
  if (tmp != 1)
  {
    tmp = true;
    try
    {
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

      cv::Mat dImg = cv_ptr->image;
      double min = 0;
      cv::Mat save_img;

      cv::Mat(cv_ptr->image - min).convertTo(save_img, 0.2, 1.5);

      if (save_img.empty())
      {
        std::cerr << "Something is wrong with the webcam, could not get frame." << std::endl;
      }

      int x = 300;          // Coordinata x del punto di inizio del ritaglio
      int y = 400;          // Coordinata y del punto di inizio del ritaglio
      int larghezza = 1600; // Larghezza del ritaglio
      int altezza = 600;    // Altezza del ritaglio
      cv::Mat img_ritagliata = ritagliaImmagine(save_img, x, y, larghezza, altezza);

      // Save the frame into a file
      cv::imwrite(save_image, img_ritagliata); // A JPG FILE IS BEING SAVED

      // Send to roboflow and receive the prediction inside stringa
      std::string to_exec = "base64 " + save_image + " | curl -d @- \ 'https://detect.roboflow.com/robot_vision-coy8u/1?api_key=yHS1WwgrBkoPZ5gHsqA3'";
      std::string stringa = exec(to_exec.c_str());
      std::cout << stringa << std::endl;
      std::vector<Prediction> valoriXYClass = extractIntegerWords(stringa);
      Eigen::Vector3f p;
      Eigen::Vector3f pointInBaseFrame;
      std::string totale;

      for (const auto &valore : valoriXYClass)
      {
        p << valore.x, valore.y, 0.92;

        Eigen::Vector3f t(-0.5, 0.35, 1.75);
        Eigen::AngleAxisf r(M_PI, Eigen::Vector3f::UnitX());

        // Create affine transformation from traslation and rotation
        Eigen::Affine3f transformation(r);
        transformation.pretranslate(t);

        // Calculate coordinates of new frame
        pointInBaseFrame = transformation * p;

        totale += std::to_string(pointInBaseFrame[0]) + " " + std::to_string(pointInBaseFrame(1)) + " " + std::to_string(pointInBaseFrame(2)) + " " + valore.classe + " || ";
      }

      // Publisher:
      ros::init(argc, argv, "vision");
      ros::NodeHandle n;
      ros::Publisher pub = n.advertise<std_msgs::String>("vision", 100);
      ros::Rate loop_rate(1);

      int count = 0;
      while (ros::ok())
      {
        if (pub.getNumSubscribers() > 0)
        {
          std_msgs::String msg;
          std::stringstream ss;

          ss << totale;
          msg.data = ss.str();

          pub.publish(msg);

          ros::spinOnce();

          loop_rate.sleep();
          ++count;
        }
        else
        {
          ROS_INFO("No one listening");
        }
      }
    }

    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe("/ur5/zed_node/left/image_rect_color", 1, imageCallback);

  ros::spin();

  return 0;
}
