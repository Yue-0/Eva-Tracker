/* @Author YueLin */

#include <string>
#include <vector>
#include <cstring>

#include "NvInfer.h"
#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"
#include "cuda_runtime_api.h"

namespace eva_tracker
{
    class Person
    {
        public:
            static const int SKELETON = 17;
            enum Skeleton{
                Nose = 0,
                LeftEye,      RightEye,
                LeftEar,      RightEar,
                LeftShoulder, RightShoulder,
                LeftElbow,    RightElbow,
                LeftWrist,    RightWrist,
                LeftHip,      RightHip,
                LeftKnee,     RightKnee,
                LeftAnkle,    RightAnkle
            };
        
        private:
            float confidence;
            float confidences[SKELETON];
        
        public:
            cv::Rect box;
            cv::Point keypoints[SKELETON];
            Eigen::Matrix<float, SKELETON, 3> skeleton;
        
        public:
            Person(cv::Rect2d rect, float score, 
                   std::vector<cv::Point3f> points);
        
        /* For visualization */
        private:
            static const int LINE = 19;
            cv::Point connect[LINE] = {
                cv::Point(LeftAnkle, LeftKnee), 
                cv::Point(LeftKnee, LeftHip), 
                cv::Point(RightAnkle, RightKnee),
                cv::Point(RightKnee, RightHip), 
                cv::Point(LeftHip, RightHip), 
                cv::Point(LeftShoulder, LeftHip), 
                cv::Point(RightShoulder, RightHip), 
                cv::Point(LeftShoulder, RightShoulder), 
                cv::Point(LeftShoulder, LeftElbow), 
                cv::Point(RightShoulder, RightElbow), 
                cv::Point(LeftElbow, LeftWrist), 
                cv::Point(RightElbow, RightWrist),
                cv::Point(LeftEye, RightEye), 
                cv::Point(Nose, LeftEye), 
                cv::Point(Nose, RightEye), 
                cv::Point(LeftEye, LeftEar), 
                cv::Point(RightEye, RightEar), 
                cv::Point(LeftEar, LeftShoulder), 
                cv::Point(RightEar, RightShoulder)
            };
            cv::Scalar colors[SKELETON + LINE] = {
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0x33, 0x99, 0xFF),
                cv::Scalar(0xFF, 0x33, 0x99),
                cv::Scalar(0xFF, 0x33, 0x99),
                cv::Scalar(0xFF, 0x33, 0x99),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0xFF, 0x80, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0),
                cv::Scalar(0, 0xFF, 0)
            };
            cv::Scalar color = cv::Scalar(0xF, 0xAF, 0x42);
        
        public:
            void visualize(cv::Mat& image, double threshold, 
                           bool rect, bool points, bool lines) const;
    };

    class YOLOv11Pose
    {
        private:
            float r;
            cv::Size offset;
            bool prev = false;
            cv::Point2d previous;
            Eigen::Vector3f position3d;
            const int SKELETON = Person::SKELETON;
        
        public:
            float height, width;

        /* Model parameters */
        private:
            const int DIM = 8400;
            const int SIZE[2] = {640, 640};
            const char *INPUT = (char*)"images", *OUTPUT = (char*)"output0";
        
        /* For TensorRT inference */
        private:
            void* buffers[2];
            int input, output;
            cudaStream_t stream;
            float *image, *detection;
            nvinfer1::IExecutionContext* context;
        
        public:
            ~YOLOv11Pose();
            YOLOv11Pose(std::string path);
            Eigen::Vector3f solve(const cv::Mat& depth, 
                                  const Person& person, 
                                  float fx, float fy, float cx, float cy);
            std::vector<Person> detect(const cv::Mat& img, double threshold, 
                                       bool single, double nms);
            void visualize(cv::Mat& image, std::vector<Person>& objects, 
                           bool boxes, bool keypoints, bool lines);
        
        private:
            inline void inference();
            inline void preprocess(const cv::Mat& mat);
            inline std::vector<Person> decode(cv::Size size, double threshold, 
                                              bool single, double nms);
            float value(int row, int col) const {
                return detection[col + row * DIM];
            }
    };
}
