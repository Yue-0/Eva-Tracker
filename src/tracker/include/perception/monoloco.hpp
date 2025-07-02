/* @Author YueLin */

#include <string>
#include <vector>
#include <cstring>

#include "NvInfer.h"
#include "Eigen/Eigen"
#include "cuda_runtime_api.h"

namespace eva_tracker
{
    class MonoLoco
    {
        private:
            static const int DIM = 9;
            static const int SKELETON = 17;
            const char *INPUT = (char*)"keypoints", *OUTPUT = (char*)"output";
        
        public:
            float yaw;

        private:
            bool init;
            Eigen::Matrix3f intrinsic;
        
        /* For TensorRT inference */
        private:
            void* buffers[2];
            int input, output;
            cudaStream_t stream;
            nvinfer1::IExecutionContext* context;
            float inputs[SKELETON << 1], outputs[DIM];
        
        public:
            ~MonoLoco();
            MonoLoco(std::string);
        
        public:
            void process(Eigen::Matrix<float, SKELETON, 3>,
                         float, float, float, float);
        
        private:
            inline void inference();
            inline void initialize(float, float, float, float);
            inline void preprocess(Eigen::Matrix<float, SKELETON, 3>&);
    };
}
