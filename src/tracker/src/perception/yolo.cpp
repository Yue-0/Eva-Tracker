/* @Author YueLin */

#include <cmath>
#include <limits>
#include <fstream>

#include "perception/yolo.hpp"
#include "perception/logger.hpp"

namespace eva_tracker
{
    Person::Person(const cv::Rect2d& rect, float score, 
                   const std::vector<cv::Point3f>& points): confidence(score)
    {
        box.x = std::round(rect.x);
        box.y = std::round(rect.y);
        box.width = std::round(rect.width);
        box.height = std::round(rect.height);

        for(int p = 0; p < SKELETON; p++)
        {
            confidences[p] = skeleton(p, 2) = points[p].z;
            keypoints[p].x = std::round(skeleton(p, 0) = points[p].x);
            keypoints[p].y = std::round(skeleton(p, 1) = points[p].y);
        }
    }

    void Person::visualize(cv::Mat& image, double threshold,
                           bool rect, bool points, bool lines) const
    {
        /* Visualize box */
        if(rect) cv::rectangle(image, box, color, 4);

        /* Visualize keypoints */
        if(points)
            for(int p = 0; p < SKELETON; p++)
                if(confidences[p] > threshold)
                    cv::circle(image, keypoints[p], 8, colors[p], -1);
        
        /* Visualize lines */
        if(lines) for(int line = 0; line < LINE; line++)
        {
            cv::Point index = connect[line];
            if(std::max(confidences[index.x], confidences[index.y]) > threshold)
                cv::line(
                    image,
                    keypoints[index.x],
                    keypoints[index.y], 
                    colors[line + SKELETON], 4
                );
        }
    }

    YOLOv11Pose::~YOLOv11Pose()
    {
        /* Release memory and stream */
        cudaFree(buffers[input]);
        cudaFree(buffers[output]);
        cudaStreamDestroy(stream);
        delete image; delete detection;

        /* Release tensorrt context */
        delete context;
    }

    YOLOv11Pose::YOLOv11Pose(const std::string& path)
    {
        /* Initialize */
        size_t size{0};
        cudaSetDevice(0);
        char *binary{nullptr};
        static Logger _logger;

        /* Apply for memory from CPU */
        image = new float[SIZE[0] * SIZE[1] * 3];
        detection = new float[DIM * (SKELETON * 3 + 5)];

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
        cudaMalloc(&buffers[output], DIM * (SKELETON * 3 + 5) * sizeof(float));
        cudaMalloc(&buffers[input], 3 * SIZE[0] * SIZE[1] * sizeof(float));
        context->setTensorAddress(OUTPUT, buffers[output]);
        context->setTensorAddress(INPUT, buffers[input]);

        /* Create cuda stream */
        cudaStreamCreate(&stream);

        /* Warmup */
        position3d.setZero();
        cv::Mat warmup(SIZE[0], SIZE[1], CV_8UC3);
        cv::randu(warmup, cv::Scalar::all(0), cv::Scalar::all(0xFF));
        detect(warmup, 0.5, false, 0.5); detect(warmup, 0.5, false, 0.5);
    }

    Eigen::Vector3f YOLOv11Pose::solve(const cv::Mat& depth, 
                                       const Person& person,
                                       float fx, float fy, float cx, float cy)
    {
        /* Select ROI */
        cv::Point tl = (
            person.keypoints[person.LeftHip] +
            person.keypoints[person.RightHip] +
            person.keypoints[person.LeftShoulder] +
            person.keypoints[person.RightShoulder]
        ) / 4;
        cv::Point br(std::abs(
            - person.keypoints[person.LeftHip].x
            + person.keypoints[person.RightHip].x
        ), std::abs(std::min(
            person.keypoints[person.LeftHip].y,
            person.keypoints[person.RightHip].y
        ) - std::max(
            person.keypoints[person.LeftShoulder].y,
            person.keypoints[person.RightShoulder].y
        )));
        tl -= br / 2; br += tl;
        tl.x = std::min(std::max(tl.x, 0), depth.cols - 2);
        tl.y = std::min(std::max(tl.y, 0), depth.rows - 2);
        br.x = std::max(std::min(br.x, depth.cols - 1), tl.x + 1);
        br.y = std::max(std::min(br.y, depth.rows - 1), tl.y + 1);

        /* Solve 3D point */
        cv::Rect box(tl, br);
        // position3d[0] = cv::mean(depth(box), depth(box) > 0)[0];
        position3d[0] = cv::mean(depth(box), depth(box) > 0)[0] - 0.05;  // TODO
        position3d[1] = position3d[0] * (cx - box.x - (box.width >> 1)) / fx;
        position3d[2] = position3d[0] * (cy - box.y - (box.height >> 1)) / fy;

        /* Solve width and height */
        width = position3d[0] * person.box.width / fx;
        height = position3d[0] * person.box.height / fy;
        return position3d;
    }

    std::vector<Person> YOLOv11Pose::detect(const cv::Mat& img, 
                                            double threshold,
                                            bool single,
                                            double nms)
    {
        preprocess(img); inference();
        return decode(img.size(), threshold, single, nms);
    }

    inline void YOLOv11Pose::inference()
    {
        /* Copy data to cuda */
        cudaMemcpyAsync(
            buffers[input], image,
            3 * SIZE[0] * SIZE[1] * sizeof(float),
            cudaMemcpyHostToDevice, stream
        );

        /* Inference */
        context->enqueueV3(stream);

        /* Copy results to cpu */
        cudaMemcpyAsync(
            detection, buffers[output],
            DIM * (SKELETON * 3 + 5) * sizeof(float),
            cudaMemcpyDeviceToHost, stream
        );
        //// detection: 56 * DIM (56: x, y, w, h, confidence, 17x3) ////

        /* Wait for inference to complete */
        cudaStreamSynchronize(stream);
    }

    inline void YOLOv11Pose::preprocess(const cv::Mat& mat)
    {
        /* Convert from BGR mode into RGB mode */
        cv::Mat img;
        cv::cvtColor(mat, img, cv::COLOR_BGR2RGB);

        /* Resize image */
        cv::Size size = mat.size();
        r = std::min(
            (float)SIZE[0] / size.width,
            (float)SIZE[1] / size.height
        );
        cv::Size sz(std::round(r * size.width), std::round(r * size.height));
        offset = cv::Size((SIZE[0] - sz.width) / 2, (SIZE[1] - sz.height) / 2);
        if(sz.width != SIZE[0] || sz.height != SIZE[1])
            cv::resize(img, img, sz);
        r = 1.0 / r;
        
        /* Padding */
        cv::copyMakeBorder(
            img, img, offset.height, offset.height, offset.width, offset.width, 
            cv::BORDER_CONSTANT, cv::Scalar::all(0x72)
        );

        /* Check size */
        sz = img.size();
        if(sz.width != SIZE[0] || sz.height != SIZE[1])
            cv::resize(img, img, cv::Size(SIZE[0], SIZE[1]));
        
        /* Normalization */
        for(int c = 0; c < 3; c++)
            for(int y = 0; y < SIZE[1]; y++)
                for(int x = 0; x < SIZE[0]; x++)
                    image[c * SIZE[1] * SIZE[0] + y * SIZE[0] + x] 
                    = img.at<cv::Vec3b>(y, x)[c] / 255.f;
    }

    inline std::vector<Person> YOLOv11Pose::decode(cv::Size size,
                                                   double threshold,
                                                   bool single,
                                                   double nms)
    {
        std::vector<std::vector<cv::Point3f>> keypoints;
        std::vector<float> confidences;
        std::vector<cv::Rect2d> boxes;
        std::vector<Person> objects;
        
        /* Predictions filtering by threshold */
        int numbers = 0;
        for(int index = 0; index < DIM; index++)
        {
            float confidence = value(4, index);
            if(confidence >= threshold)
            {
                ++numbers;
                boxes.emplace_back(
                    value(0, index),
                    value(1, index),
                    value(2, index),
                    value(3, index)
                );
                std::vector<cv::Point3f> points;
                for(int p = 0; p < SKELETON; p++)
                    points.emplace_back(
                        value(5 + 3 * p, index),
                        value(6 + 3 * p, index),
                        value(7 + 3 * p, index)
                    );
                keypoints.push_back(points);
                confidences.push_back(confidence);
            }
        }
        if(!numbers) return objects;

        /* Keep the previous detected object */
        if(single)
        {
            int match = 0;
            if(prev)
            {
                double distance = std::numeric_limits<double>::infinity();
                for(int index = 0; index < numbers; index++)
                {
                    cv::Point2d delta = previous - boxes[index].tl();
                    double d = delta.dot(delta);
                    if(d < distance)
                    {
                        match = index;
                        distance = d;
                    }
                }
            }
            else
            {
                prev = true;
                float confidence = 0;
                for(int index = 0; index < numbers; index++)
                    if(confidences[index] > confidence)
                    {
                        match = index;
                        confidence = confidences[index];
                    }
            }
            numbers = 1;
            if(match)
            {
                boxes[0] = boxes[match];
                keypoints[0] = keypoints[match];
                confidences[0] = confidences[match];
            }
            previous = boxes[match].tl();
        }

        /* NMS filtering */
        if(numbers > 1)
        {
            numbers = 0;
            std::vector<int> indices;
            cv::dnn::NMSBoxes(boxes, confidences, threshold, nms, indices);
            for(int index: indices)
            {
                if(index != numbers)
                {
                    boxes[numbers] = boxes[index];
                    keypoints[numbers] = keypoints[index];
                    confidences[numbers] = confidences[index];
                }
                ++numbers;
            }
        }

        /* Post process */
        for(int index = 0; index < numbers; index++)
        {
            /* Process boxes: (x_center, y_center) -> (x_tl, y_tl) */
            boxes[index].y -= offset.height + boxes[index].height * 0.5;
            boxes[index].x -= offset.width + boxes[index].width * 0.5;
            boxes[index].y *= r; boxes[index].height *= r;
            boxes[index].x *= r; boxes[index].width *= r;
            
            /* Filter out edge boxes */
            if(boxes[index].x > size.width * 0.95)
                continue;
            if(boxes[index].x + boxes[index].width < size.width * 0.05)
                continue;

            /* Rescale keypoints to the shape of original image */
            for(int p = 0; p < SKELETON; p++)
            {
                keypoints[index][p].y -= offset.height;
                keypoints[index][p].x -= offset.width;
                keypoints[index][p].y *= r;
                keypoints[index][p].x *= r;
            }
    
            objects.emplace_back(
                boxes[index], confidences[index], keypoints[index]
            );
        }
        return objects;
    }

    void YOLOv11Pose::visualize(cv::Mat& image, 
                                const std::vector<Person>& objects,
                                bool boxes, bool keypoints, bool lines) const
    {
        for(const Person& person: objects)
            person.visualize(image, 0.25, boxes, keypoints, lines);
    }
}
