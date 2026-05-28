#include "armor_detector.h"
#include <algorithm>
#include <cmath>

ArmorDetector::ArmorDetector() {}

float ArmorDetector::normalizeAngle(float angle) {
    while (angle >= 90.0f)  angle -= 180.0f;
    while (angle < -90.0f)  angle += 180.0f;
    return angle;
}

float ArmorDetector::angleDiff(float a, float b) {
    float d = std::abs(normalizeAngle(a - b));
    return d > 90.0f ? 180.0f - d : d;
}

bool ArmorDetector::isLightBar(const std::vector<cv::Point>& contour,
                                LightBar& lb, const cv::Size& frameSize) {
    double area = cv::contourArea(contour);
    double maxArea = frameSize.area() * maxContourAreaRatio_;

    if (area < minContourArea_ || area > maxArea)
        return false;

    cv::RotatedRect rect = cv::minAreaRect(contour);

    float w = rect.size.width;
    float h = rect.size.height;

    if (w < 1.0f || h < 3.0f)
        return false;

    float length = std::max(w, h);
    float width  = std::min(w, h);

    if (width <= 0.1f)
        return false;

    float ratio = length / width;
    if (ratio < lightBarMinRatio_ || ratio > lightBarMaxRatio_)
        return false;

    float maxLength = frameSize.height * lightBarMaxLengthRatio_;
    if (length < lightBarMinLength_ || length > maxLength)
        return false;

    // 规范化: height=长边, angle=长边与水平线的夹角
    float angle = rect.angle;
    if (w > h) {
        angle += 90.0f;
    }
    angle = normalizeAngle(angle);

    if (std::abs(angle) > lightBarMaxAngle_)
        return false;

    // 忽略画面上25%的区域
    if (rect.center.y < frameSize.height * 0.25f)
        return false;

    lb.rect   = rect;
    lb.center = rect.center;
    lb.length = length;
    lb.width  = width;
    lb.angle  = angle;
    rect.points(lb.pts);

    return true;
}

bool ArmorDetector::tryPair(const LightBar& a, const LightBar& b,
                             ArmorCandidate& armor, const cv::Size& frameSize) {
    // 确定左右
    const LightBar* pL = &a;
    const LightBar* pR = &b;
    if (pR->center.x < pL->center.x)
        std::swap(pL, pR);

    float dx = pR->center.x - pL->center.x;
    float dy = std::abs(pR->center.y - pL->center.y);

    if (dx <= 0.0f)
        return false;

    float avgLength = (pL->length + pR->length) * 0.5f;
    float heightRatio = std::max(pL->length, pR->length) /
                        std::min(pL->length, pR->length);
    float angleDiff_ = angleDiff(pL->angle, pR->angle);
    float xRatio = dx / avgLength;
    float armorAngle = std::abs(std::atan2(dy, dx) * 180.0f / CV_PI);

    // 配对约束
    if (heightRatio   > pairMaxHeightRatio_)  return false;
    if (angleDiff_    > pairMaxAngleDiff_)    return false;
    if (dy            > avgLength * pairMaxYOffsetRatio_) return false;
    if (xRatio < pairDistRatioMin_ || xRatio > pairDistRatioMax_) return false;
    if (armorAngle    > pairMaxArmorAngle_)   return false;
    if (dx            > frameSize.width * pairMaxDxRatio_) return false;

    cv::Point2f center = (pL->center + pR->center) * 0.5f;

    if (center.y < frameSize.height * 0.25f)
        return false;

    // 综合评分 (越小越好)
    cv::Point2f expectedCenter(frameSize.width * 0.5f, frameSize.height * 0.58f);
    double centerDist = cv::norm(center - expectedCenter) / frameSize.width;

    armor.left  = *pL;
    armor.right = *pR;

    armor.score =
        angleDiff_ * 1.5 +
        std::abs(heightRatio - 1.0f) * 25.0 +
        dy / avgLength * 20.0 +
        std::abs(xRatio - 2.5f) * 8.0 +
        centerDist * 15.0;

    return true;
}

std::vector<ImageArmor> ArmorDetector::detect(const cv::Mat& binaryMask,
                                               const cv::Size& frameSize) {
    std::vector<ImageArmor> armors;

    // 1. 提取轮廓并筛选灯条
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    lastLightBars_.clear();
    for (const auto& contour : contours) {
        LightBar lb;
        if (isLightBar(contour, lb, frameSize))
            lastLightBars_.push_back(lb);
    }

    if (lastLightBars_.size() < 2)
        return armors;

    // 2. 尝试所有配对, 选评分最低的
    ArmorCandidate bestArmor;
    bestArmor.score = 1e9;
    bool found = false;

    for (size_t i = 0; i < lastLightBars_.size(); i++) {
        for (size_t j = i + 1; j < lastLightBars_.size(); j++) {
            ArmorCandidate candidate;
            if (tryPair(lastLightBars_[i], lastLightBars_[j], candidate, frameSize)) {
                if (candidate.score < bestArmor.score) {
                    bestArmor = candidate;
                    found = true;
                }
            }
        }
    }

    if (!found)
        return armors;

    // 3. 构造 ImageArmor 输出 (corners: TL, TR, BR, BL)
    ImageArmor armor;
    const auto& L = bestArmor.left;
    const auto& R = bestArmor.right;

    // 提取灯条端点: 找每个灯条的上下短边中点
    auto getTopBottom = [](const cv::Point2f pts[4]) -> std::pair<cv::Point2f, cv::Point2f> {
        // 6条边中找出最短的两条(灯条的上下短边)
        struct Edge { double d; int i, j; } edges[6];
        int idx = 0;
        for (int a = 0; a < 4; a++)
            for (int b = a + 1; b < 4; b++) {
                double dx = pts[a].x - pts[b].x;
                double dy = pts[a].y - pts[b].y;
                edges[idx++] = {dx*dx + dy*dy, a, b};
            }
        // 找最短的两条边
        if (edges[0].d > edges[1].d) std::swap(edges[0], edges[1]);
        for (int k = 2; k < 6; k++) {
            if (edges[k].d < edges[0].d) {
                edges[1] = edges[0];
                edges[0] = edges[k];
            } else if (edges[k].d < edges[1].d) {
                edges[1] = edges[k];
            }
        }
        cv::Point2f m0 = (pts[edges[0].i] + pts[edges[0].j]) * 0.5f;
        cv::Point2f m1 = (pts[edges[1].i] + pts[edges[1].j]) * 0.5f;
        if (m0.y < m1.y) return {m0, m1};
        else              return {m1, m0};
    };

    auto [lt, lb] = getTopBottom(L.pts);
    auto [rt, rb] = getTopBottom(R.pts);

    armor.corners = {lt, rt, rb, lb};  // TL, TR, BR, BL
    armor.center  = (lt + rb) * 0.5f;
    armor.valid   = true;
    armors.push_back(armor);

    return armors;
}
