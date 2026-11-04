#include "sliced_inference.h"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
TargetBox box(int x1, int y1, int x2, int y2, int category = 0, float score = 0.9f) {
    TargetBox result;
    result.x1 = x1; result.y1 = y1; result.x2 = x2; result.y2 = y2;
    result.cate = category; result.score = score;
    return result;
}
void testGridAndPixels() {
    crack_inference::Options options;
    options.mode = "sliced";
    require(crack_inference::axisStarts(1536, 512, 384) == std::vector<int>({0, 384, 768, 1024}),
            "Incorrect horizontal tail coverage");
    require(crack_inference::axisStarts(1252, 512, 384) == std::vector<int>({0, 384, 740}),
            "Incorrect vertical tail coverage");
    cv::Mat image(1252, 1536, CV_8UC3);
    for (int y = 0; y < image.rows; ++y)
        for (int x = 0; x < image.cols; ++x)
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(x % 251, y % 251, (x / 251 + y / 251) % 251);
    const std::vector<int> xs{0, 384, 768, 1024}, ys{0, 384, 740};
    size_t calls = 0;
    std::vector<TargetBox> output;
    crack_inference::Stats stats;
    const int result = crack_inference::run(image, options,
        [&](const cv::Mat& tile, std::vector<TargetBox>& found) {
            const int x = xs.at(calls % 4), y = ys.at(calls / 4);
            require(tile.isContinuous() && tile.cols == 512 && tile.rows == 512,
                    "NCNN must receive a contiguous 512 square tile");
            require(cv::norm(tile, image(cv::Rect(x, y, 512, 512)), cv::NORM_INF) == 0.0,
                    "ROI pixels or row stride corrupted");
            found.push_back(box(10, 20, 30, 40));
            ++calls;
            return 0;
        }, output, stats);
    require(result == 0 && calls == 12 && stats.tiles == 12 && output.size() == 12,
            "Expected 12 tiles and 12 independent boxes");
    for (size_t index = 0; index < output.size(); ++index)
        require(output[index].x1 == xs[index % 4] + 10 && output[index].y1 == ys[index / 4] + 20,
                "Tile boxes were not mapped to original coordinates");
}
void testPaddingAndClipping() {
    crack_inference::Options options;
    options.mode = "sliced";
    cv::Mat image(80, 100, CV_8UC3, cv::Scalar(5, 10, 15));
    std::vector<TargetBox> output;
    crack_inference::Stats stats;
    crack_inference::run(image, options,
        [&](const cv::Mat& tile, std::vector<TargetBox>& found) {
            require(tile.isContinuous() && tile.at<cv::Vec3b>(79, 99) == cv::Vec3b(5, 10, 15),
                    "Small image pixels changed");
            require(tile.at<cv::Vec3b>(80, 100) == cv::Vec3b(114, 114, 114), "Incorrect padding");
            found = {box(-10, 10, 150, 90), box(120, 120, 180, 180), box(1, 1, 2, 2, -1),
                     box(1, 1, 2, 2, 0, std::numeric_limits<float>::quiet_NaN())};
            return 0;
        }, output, stats);
    require(output.size() == 1 && output[0].x1 == 0 && output[0].x2 == 100 &&
            output[0].y1 == 10 && output[0].y2 == 80, "Padding boxes must be clipped or discarded");
}
void testDuplicateMerge() {
    crack_inference::Options options;
    options.mode = "sliced";
    cv::Mat image(512, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<TargetBox> output;
    crack_inference::Stats stats;
    int calls = 0;
    crack_inference::run(image, options,
        [&](const cv::Mat&, std::vector<TargetBox>& found) {
            const int offset = calls++ == 0 ? 0 : 88;
            found.push_back(box(400 - offset, 100, 480 - offset, 200, 0, offset == 0 ? 0.9f : 0.8f));
            if (offset != 0) found.push_back(box(400 - offset, 100, 480 - offset, 200, 1, 0.7f));
            return 0;
        }, output, stats);
    require(stats.candidates_before_merge == 3 && output.size() == 2,
            "Same-class duplicate must merge; a different class must remain");
    require(output[0].x1 == 400 && output[0].score == 0.9f && output[1].cate == 1,
            "Global NMS did not retain the highest-score detection");
}
void testWholeAndFailure() {
    crack_inference::Options options;
    cv::Mat image(512, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<TargetBox> output;
    crack_inference::Stats stats;
    int calls = 0;
    crack_inference::run(image, options,
        [&](const cv::Mat& received, std::vector<TargetBox>& found) {
            require(received.data == image.data, "Whole mode must forward the original frame");
            ++calls;
            found = {box(-5, -5, 50, 50), box(-5, -5, 50, 50)};
            return 0;
        }, output, stats);
    require(calls == 1 && stats.tiles == 1 && output.size() == 2 && output[0].x1 == -5,
            "Whole mode must not introduce a second filter");
    options.mode = "sliced";
    calls = 0;
    const int result = crack_inference::run(image, options,
        [&](const cv::Mat&, std::vector<TargetBox>& found) {
            found.push_back(box(0, 0, 20, 20));
            return ++calls == 2 ? -7 : 0;
        }, output, stats);
    require(result == -7 && output.empty(), "Failed inference must not expose partial or stale boxes");
    options.max_tiles = 1;
    bool rejected = false;
    try {
        crack_inference::run(image, options,
            [](const cv::Mat&, std::vector<TargetBox>&) { return 0; }, output, stats);
    } catch (const std::runtime_error&) { rejected = true; }
    require(rejected && output.empty(), "Tile limit was not enforced");
    options.tile_overlap = std::numeric_limits<double>::quiet_NaN();
    rejected = false;
    try { options.validate(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "NaN overlap must be rejected");
}
void testHybrid() {
    crack_inference::Options options;
    options.mode = "hybrid";
    cv::Mat image(512, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<TargetBox> output;
    crack_inference::Stats stats;
    int calls = 0;
    const auto predict = [&](const cv::Mat& input, std::vector<TargetBox>& found) {
        if (calls++ == 0) {
            require(input.cols == 600, "Hybrid whole pass lost original field of view");
            found = {box(100, 100, 300, 300, 0, .95f)};
        } else if (calls == 2) {
            found = {box(110, 110, 290, 290, 0, .8f), box(400, 20, 450, 70, 0, .85f)};
        } else {
            found = {box(312, 20, 362, 70, 0, .8f)};
        }
        return 0;
    };
    require(crack_inference::run(image, options, predict, output, stats) == 0,
            "Hybrid inference failed");
    require(calls == 3 && stats.tiles == 3 && stats.whole_passes == 1 && output.size() == 2,
            "Hybrid must run whole plus both tiles and suppress duplicates");
    require(output[0].x1 == 100 && output[0].x2 == 300 && output[1].x1 == 400,
            "Hybrid must retain real box bounds without unioning unrelated detections");
    calls = 0;
    const int failed = crack_inference::run(image, options,
        [&](const cv::Mat&, std::vector<TargetBox>& found) {
            found = {box(10, 10, 50, 50)};
            return ++calls == 3 ? -9 : 0;
        }, output, stats);
    require(failed == -9 && output.empty(), "Failed hybrid tiles leaked partial whole results");
    calls = 0;
    crack_inference::run(image, options,
        [&](const cv::Mat&, std::vector<TargetBox>& found) {
            if (++calls == 2) found = {box(20, 30, 40, 50)};
            return 0;
        }, output, stats);
    require(output.size() == 1 && output[0].x1 == 20, "Empty whole must not gate slice inference");
    calls = 0;
    crack_inference::run(image, options,
        [&](const cv::Mat&, std::vector<TargetBox>& found) {
            if (++calls == 1) found = {box(-10, -10, 620, 530)};
            return 0;
        }, output, stats);
    require(output.size() == 1 && output[0].x1 == 0 && output[0].x2 == 600 && output[0].y2 == 512,
            "Hybrid whole boxes must be clipped to original bounds");
    const std::vector<TargetBox> whole{box(100, 100, 400, 400, 0, .9f)};
    output = {whole[0], box(200, 200, 220, 220, 0, .8f),
              box(390, 200, 420, 220, 0, .8f), box(200, 200, 220, 220, 1, .8f),
              box(250, 250, 270, 270, 0, .95f)};
    crack_inference::removeContainedTiles(whole, output);
    require(output.size() == 4 && output[0].x2 == 400 && output[1].x2 == 420 &&
            output[2].cate == 1 && output[3].score == .95f,
            "Contained duplicate rule lost a branch tip, different class or higher-confidence tile");
    output = {box(200, 200, 220, 220, 0, .8f)};
    crack_inference::removeContainedTiles(whole, output);
    require(output.size() == 1, "A suppressed whole-frame box must not suppress tiles");
}
}
int main() {
    try {
        testGridAndPixels();
        testPaddingAndClipping();
        testDuplicateMerge();
        testWholeAndFailure();
        testHybrid();
        std::cout << "PASS: tile coverage, contiguous pixels, mapping, padding, class NMS, whole mode, failures\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
