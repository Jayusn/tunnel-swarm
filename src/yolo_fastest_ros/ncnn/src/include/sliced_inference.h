#ifndef CRACK_SLICED_INFERENCE_H_
#define CRACK_SLICED_INFERENCE_H_

#include "yolo-fastestv2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace crack_inference {

struct Options {
    std::string mode = "whole";
    int tile_size = 512;
    double tile_overlap = 0.25;
    double global_nms = 0.25;
    int max_tiles = 256;

    void validate() const {
        if (mode != "whole" && mode != "sliced" && mode != "hybrid")
            throw std::invalid_argument("detection mode must be whole, sliced or hybrid");
        if (tile_size < 32 || tile_size > 8192 || max_tiles < 1 ||
            !std::isfinite(tile_overlap) || tile_overlap < 0.0 || tile_overlap >= 1.0 ||
            std::lround(tile_size * (1.0 - tile_overlap)) < 1 ||
            !std::isfinite(global_nms) || global_nms < 0.0 || global_nms > 1.0)
            throw std::invalid_argument("Invalid tile size, overlap, global NMS or tile limit");
    }
};

struct Stats {
    size_t tiles = 0;
    size_t whole_passes = 0;
    size_t candidates_before_merge = 0;
};

inline std::vector<int> axisStarts(int length, int tile_size, int stride) {
    if (length <= tile_size) return {0};
    std::vector<int> starts;
    const int last = length - tile_size;
    for (int start = 0; start < last; start += stride) starts.push_back(start);
    if (starts.empty() || starts.back() != last) starts.push_back(last);
    return starts;
}

inline double intersectionOverUnion(const TargetBox& a, const TargetBox& b) {
    const double width = std::max(0, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    const double height = std::max(0, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const double intersection = width * height;
    const double area_a = static_cast<double>(a.x2 - a.x1) * (a.y2 - a.y1);
    const double area_b = static_cast<double>(b.x2 - b.x1) * (b.y2 - b.y1);
    const double denominator = area_a + area_b - intersection;
    return denominator > 0.0 ? intersection / denominator : 0.0;
}

inline void globalNms(std::vector<TargetBox>& candidates, double threshold,
                      std::vector<TargetBox>& output) {
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const TargetBox& a, const TargetBox& b) { return a.score > b.score; });
    output.clear();
    for (const auto& box : candidates) {
        bool suppressed = false;
        for (const auto& kept : output) {
            if (box.cate == kept.cate && intersectionOverUnion(box, kept) > threshold) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) output.push_back(box);
    }
}

// Remove a contained tile detection only when a surviving whole-frame box
// of the same class has at least its confidence. Never enlarge any box or
// discard a tile that extends beyond the whole-frame prediction.
inline void removeContainedTiles(const std::vector<TargetBox>& whole,
                                 std::vector<TargetBox>& output) {
    const auto same = [](const TargetBox& a, const TargetBox& b) {
        return a.cate == b.cate && a.score == b.score && a.x1 == b.x1 &&
               a.y1 == b.y1 && a.x2 == b.x2 && a.y2 == b.y2;
    };
    std::vector<TargetBox> kept_whole;
    for (const auto& box : output)
        if (std::any_of(whole.begin(), whole.end(), [&](const TargetBox& w) { return same(box, w); }))
            kept_whole.push_back(box);
    output.erase(std::remove_if(output.begin(), output.end(), [&](const TargetBox& box) {
        if (std::any_of(kept_whole.begin(), kept_whole.end(),
                        [&](const TargetBox& w) { return same(box, w); })) return false;
        return std::any_of(kept_whole.begin(), kept_whole.end(), [&](const TargetBox& w) {
            return w.cate == box.cate && w.score >= box.score &&
                   w.x1 <= box.x1 && w.y1 <= box.y1 && w.x2 >= box.x2 && w.y2 >= box.y2;
        });
    }), output.end());
}

// Predictor receives contiguous BGR pixels. The NCNN wrapper has no row-stride
// argument, so passing a direct ROI view would read incorrect rows.
template <typename Predictor>
int run(const cv::Mat& image, const Options& options, Predictor&& predict,
        std::vector<TargetBox>& output, Stats& stats) {
    options.validate();
    output.clear();
    stats = Stats();
    if (image.empty() || image.type() != CV_8UC3) return -1;
    if (options.mode == "whole") {
        stats.tiles = 1;
        stats.whole_passes = 1;
        const int result = predict(image, output);
        if (result != 0) output.clear();
        stats.candidates_before_merge = output.size();
        return result;
    }
    const int stride = static_cast<int>(std::lround(options.tile_size * (1.0 - options.tile_overlap)));
    const auto xs = axisStarts(image.cols, options.tile_size, stride);
    const auto ys = axisStarts(image.rows, options.tile_size, stride);
    if (xs.size() * ys.size() > static_cast<size_t>(options.max_tiles))
        throw std::runtime_error("Image exceeds max_tiles; increase tile_size or tile limit in YAML");
    std::vector<TargetBox> candidates;
    std::vector<TargetBox> whole_boxes;
    if (options.mode == "hybrid") {
        std::vector<TargetBox> whole;
        const int result = predict(image.isContinuous() ? image : image.clone(), whole);
        ++stats.tiles;  // Total network passes, including the whole frame.
        ++stats.whole_passes;
        if (result != 0) return result;
        for (auto box : whole) {
            if (box.cate < 0 || !std::isfinite(box.score) || box.score < 0.f || box.score > 1.f)
                continue;
            box.x1 = std::max(0, std::min(image.cols, box.x1));
            box.x2 = std::max(0, std::min(image.cols, box.x2));
            box.y1 = std::max(0, std::min(image.rows, box.y1));
            box.y2 = std::max(0, std::min(image.rows, box.y2));
            if (box.x2 > box.x1 && box.y2 > box.y1) {
                candidates.push_back(box);
                whole_boxes.push_back(box);
            }
        }
    }
    for (const int y : ys) {
        for (const int x : xs) {
            const cv::Rect region(x, y, std::min(options.tile_size, image.cols - x),
                                   std::min(options.tile_size, image.rows - y));
            cv::Mat tile;
            if (region.width == options.tile_size && region.height == options.tile_size) {
                tile = image(region).clone();
            } else {
                cv::copyMakeBorder(image(region), tile, 0, options.tile_size - region.height,
                                    0, options.tile_size - region.width,
                                    cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(114, 114, 114));
            }
            std::vector<TargetBox> local;
            const int result = predict(tile, local);
            ++stats.tiles;
            if (result != 0) return result;  // Never return a partial frame.
            for (auto box : local) {
                if (box.cate < 0 || !std::isfinite(box.score) || box.score < 0.f || box.score > 1.f)
                    continue;
                box.x1 = std::max(0, std::min(region.width, box.x1));
                box.x2 = std::max(0, std::min(region.width, box.x2));
                box.y1 = std::max(0, std::min(region.height, box.y1));
                box.y2 = std::max(0, std::min(region.height, box.y2));
                if (box.x2 <= box.x1 || box.y2 <= box.y1) continue;
                box.x1 += x;
                box.x2 += x;
                box.y1 += y;
                box.y2 += y;
                candidates.push_back(box);
            }
        }
    }
    stats.candidates_before_merge = candidates.size();
    // Keep actual model boxes. Do not union nearby boxes: panel-edge false
    // positives must not enlarge a crack box.
    globalNms(candidates, options.global_nms, output);
    if (options.mode == "hybrid") removeContainedTiles(whole_boxes, output);
    return 0;
}

}  // namespace crack_inference
#endif
