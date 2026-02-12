#include <cmath>
#include <fstream>

#include "perception/logger.hpp"
#include "perception/monoloco.hpp"

const double PI = std::acos(-1);
const double RT = PI * 0.5;

namespace eva_tracker
{
    MonoLoco::~MonoLoco()
    {
        /* Release memory and stream */
        cudaFree(buffers[input]);
        cudaFree(buffers[output]);
        cudaStreamDestroy(stream);

        /* Release tensorrt context */
        delete context;
    }

    MonoLoco::MonoLoco(const std::string& path): init(false)
    {
        /* Initialize */
        size_t size{0};
        cudaSetDevice(0);
        char *binary{nullptr};
        static Logger _logger;

        /* Load tensorrt file */
        std::ifstream file(path, std::ios::binary);
        file.seekg(0, file.end); size = file.tellg();
        file.seekg(0, file.beg); binary = new char[size];
        file.read(binary, size); file.close();

        /* Make tensorrt context and engine */
        context = nvinfer1::createInferRuntime(_logger)
               -> deserializeCudaEngine(binary, size)
               -> createExecutionContext();
        const nvinfer1::ICudaEngine& engine = context->getEngine();

        /* Set bindding index */
        const int tensors = engine.getNbIOTensors();
        for(int n = 0; n < tensors; n++)
            if(!std::strcmp(engine.getIOTensorName(n), INPUT))
                input = n;
            else
                output = n;

        /* Apply for memory from cuda */
        cudaMalloc(&buffers[input], 2 * SKELETON * sizeof(float));
        cudaMalloc(&buffers[output], DIM * sizeof(float));
        context->setTensorAddress(OUTPUT, buffers[output]);
        context->setTensorAddress(INPUT, buffers[input]);

        /* Create cuda stream */
        cudaStreamCreate(&stream);
    }

    void MonoLoco::process(Eigen::Matrix<float, SKELETON, 3> skeleton,
                           float fx, float fy, float cx, float cy)
    {
        if(!init)
            initialize(fx, fy, cx, cy);
        preprocess(skeleton); inference();
        yaw = PI + RT - std::atan2(outputs[DIM - 2], outputs[DIM - 1]);
        if(yaw < 0 || yaw >= 2 * PI)
            yaw += 2 * PI * std::ceil(-yaw / (2 * PI));
    }

    inline void MonoLoco::inference()
    {
        /* Copy data to cuda */
        cudaMemcpyAsync(
            buffers[input], inputs,
            2 * SKELETON * sizeof(float),
            cudaMemcpyHostToDevice, stream
        );

        /* Inference */
        context->enqueueV3(stream);

        /* Copy results to cpu */
        cudaMemcpyAsync(
            outputs,
            buffers[output],
            DIM * sizeof(float),
            cudaMemcpyDeviceToHost, stream
        );

        /* Wait for inference to complete */
        cudaStreamSynchronize(stream);
    }

    inline void MonoLoco::initialize(float fx, float fy, float cx, float cy)
    {
        init = true;
        intrinsic << fx, 0, cx, 
                     0, fy, cy, 
                     0,  0,  1;
        intrinsic = intrinsic.inverse().eval();
    }

    inline void MonoLoco::preprocess(Eigen::Matrix<float, SKELETON, 3>& kps)
    {
        kps.col(2).setOnes(); kps *= intrinsic;
        for(int p = 0; p < SKELETON; p++)
        {
            inputs[p * 2 + 0] = kps(p, 0) * 10;
            inputs[p * 2 + 1] = kps(p, 1) * 10;
        }
    }
}
