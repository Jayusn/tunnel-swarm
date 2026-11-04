#include <math.h>
#include <cmath>
#include "gpu.h"
#include <algorithm>
#include "yolo-fastestv2.h"

yoloFastestv2::yoloFastestv2()
{
    printf("Creat yoloFastestv2 Detector...\n");
}

yoloFastestv2::~yoloFastestv2()
{
    printf("Destroy yoloFastestv2 Detector...\n");
}

int yoloFastestv2::loadParams(bool gpu_flag, int size, float thresh1, float thresh2,
                            int category_count, const std::vector<float>& configured_anchors)
{
    if (category_count <= 0 || category_count > 256 ||
        (!configured_anchors.empty() && configured_anchors.size() != 12) ||
        !std::all_of(configured_anchors.begin(), configured_anchors.end(),
                     [](float value) { return std::isfinite(value) && value > 0.f; })) {
        fprintf(stderr, "Invalid YOLO category count or anchors\n");
        return -1;
    }
    useGpu = gpu_flag;

    numOutput = 2;
    numThreads = 4;
    numAnchor = 3;
    numCategory = category_count;
    nmsThresh = thresh1;
    confThresh = thresh2;

    inputWidth = size;
    inputHeight = size;

    inputName = "input";
    outputName1 = "output0";
    outputName2 = "output1";

    printf("useGpu:%d\n", useGpu ? 1 : 0);
    printf("numThreads:%d\n", numThreads);
    printf("inputWidth:%d inputHeight:%d\n", inputWidth, inputHeight);

    std::vector<float> bias {27.87,75.03, 53.56,31.74, 68.35,109.40,
                            125.20,47.37, 140.63,110.75, 225.25,225.60};
    anchor = configured_anchors.empty() ? bias : configured_anchors;

    return 0;
}

int yoloFastestv2::loadModel(const char* paramPath, const char* binPath)
{
    printf("Ncnn model init:\n%s\n%s\n", paramPath, binPath);
    net.opt.num_threads = numThreads;
#if NCNN_VULKAN
    if(useGpu)
    {
        net.opt.use_vulkan_compute = true;
    }
#endif
    int result = net.load_param(paramPath);
    if (result != 0) {
        fprintf(stderr, "NCNN param load failed: %d (%s)\n", result, paramPath);
        return result;
    }
    result = net.load_model(binPath);
    if (result != 0) {
        fprintf(stderr, "NCNN weights load failed: %d (%s)\n", result, binPath);
        return result;
    }

    printf("Ncnn model init success...\n");

    return 0;
}

float intersection_area(const TargetBox &a, const TargetBox &b)
{
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        return 0.f;
    }

    float inter_width = std::max(0, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    float inter_height = std::max(0, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));

    return inter_width * inter_height;
}

bool scoreSort(TargetBox a, TargetBox b)
{
    return (a.score > b.score);
}

int yoloFastestv2::nmsHandle(std::vector<TargetBox> &tmpBoxes,
                             std::vector<TargetBox> &dstBoxes)
{
    std::vector<int> picked;

    sort(tmpBoxes.begin(), tmpBoxes.end(), scoreSort);

    for (int i = 0; i < tmpBoxes.size(); i++) {
        int keep = 1;
        for (int j = 0; j < picked.size(); j++) {
            float inter_area = intersection_area(tmpBoxes[i], tmpBoxes[picked[j]]);
            float union_area = tmpBoxes[i].area() + tmpBoxes[picked[j]].area() - inter_area;
            float IoU = union_area > 0.f ? inter_area / union_area : 0.f;

            if(IoU > nmsThresh && tmpBoxes[i].cate == tmpBoxes[picked[j]].cate) {
                keep = 0;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }

    for (int i = 0; i < picked.size(); i++) {
        dstBoxes.push_back(tmpBoxes[picked[i]]);
    }

    return 0;
}

int yoloFastestv2::getCategory(const float *values, int index, int &category, float &score)
{
    float tmp = 0;
    float objScore  = values[4 * numAnchor + index];

    for (int i = 0; i < numCategory; i++) {
        float clsScore = values[4 * numAnchor + numAnchor + i];
        clsScore *= objScore;

        if(clsScore > tmp) {
            score = clsScore;
            category = i;

            tmp = clsScore;
        }
    }

    return 0;
}

int yoloFastestv2::predHandle(const ncnn::Mat *out, std::vector<TargetBox> &dstBoxes,
                              const float scaleW, const float scaleH, const float thresh)
{
    for (int i = 0; i < numOutput; i++) {
        int stride;
        int outW, outH, outC;

        outH = out[i].c;
        outW = out[i].h;
        outC = out[i].w;

        assert(inputHeight / outH == inputWidth / outW);
        stride = inputHeight / outH;

        for (int h = 0; h < outH; h++) {
            const float* values = out[i].channel(h);

            for (int w = 0; w < outW; w++) {
                for (int b = 0; b < numAnchor; b++) {
                    TargetBox tmpBox;
                    int category = -1;
                    float score = -1;

                    getCategory(values, b, category, score);

                    if (score > thresh) {
                        float bcx, bcy, bw, bh;

                        bcx = ((values[b * 4 + 0] * 2. - 0.5) + w) * stride;
                        bcy = ((values[b * 4 + 1] * 2. - 0.5) + h) * stride;
                        bw = pow((values[b * 4 + 2] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 0];
                        bh = pow((values[b * 4 + 3] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 1];

                        tmpBox.x1 = (bcx - 0.5 * bw) * scaleW;
                        tmpBox.y1 = (bcy - 0.5 * bh) * scaleH;
                        tmpBox.x2 = (bcx + 0.5 * bw) * scaleW;
                        tmpBox.y2 = (bcy + 0.5 * bh) * scaleH;
                        tmpBox.score = score;
                        tmpBox.cate = category;

                        dstBoxes.push_back(tmpBox);
                    }
                }
                values += outC;
            }
        }
    }
    return 0;
}

int yoloFastestv2::detection(const cv::Mat srcImg, std::vector<TargetBox> &dstBoxes, const float thresh)
{
    dstBoxes.clear();

    if (srcImg.empty() || srcImg.type() != CV_8UC3) {
        return -1;
    }

    float scaleW = (float)srcImg.cols / (float)inputWidth;
    float scaleH = (float)srcImg.rows / (float)inputHeight;

    ncnn::Mat inputImg = ncnn::Mat::from_pixels_resize(srcImg.data, ncnn::Mat::PIXEL_BGR,
                                                       srcImg.cols, srcImg.rows, inputWidth, inputHeight);

    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    inputImg.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = net.create_extractor();

#if NCNN_VULKAN
    if(useGpu)
    {
        ex.set_vulkan_compute(true);
    }
#endif

    int result = ex.input(inputName, inputImg);
    if (result != 0) {
        return result;
    }

    ncnn::Mat out[2];
    result = ex.extract(outputName1, out[0]);
    if (result != 0) {
        return result;
    }
    result = ex.extract(outputName2, out[1]);
    if (result != 0) {
        return result;
    }
    for (int i = 0; i < numOutput; ++i) {
        if (out[i].empty() || out[i].dims != 3 ||
            out[i].w != numAnchor * 5 + numCategory ||
            out[i].h <= 0 || out[i].c <= 0 ||
            inputWidth % out[i].h != 0 || inputHeight % out[i].c != 0 ||
            inputWidth / out[i].h != inputHeight / out[i].c) {
            fprintf(stderr, "Unexpected output%d shape: %d x %d x %d\n",
                    i, out[i].w, out[i].h, out[i].c);
            return -1;
        }
    }

    std::vector<TargetBox> tmpBoxes;
    predHandle(out, tmpBoxes, scaleW, scaleH, thresh < 0.f ? confThresh : thresh);
    nmsHandle(tmpBoxes, dstBoxes);

    return 0;
}
