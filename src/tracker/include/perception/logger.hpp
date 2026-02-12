/* @Author YueLin */

#pragma once

#include <ros/ros.h>
#include <NvInfer.h>

class Logger: public nvinfer1::ILogger
{
    public:
        void log(Severity severity, const char* msg) noexcept override
        {
            if(severity <= Severity::kERROR)
                ROS_ERROR("[TRT] %s", msg);
            else if(severity == Severity::kWARNING)
                ROS_WARN("[TRT] %s", msg);
            else if(severity == Severity::kINFO)
                ROS_INFO("[TRT] %s", msg);
            else if(severity == Severity::kVERBOSE)
                ROS_DEBUG("[TRT] %s", msg);
        }
};
