#include "armor_detector.h"
#include <algorithm>
#include <cmath>
#include <list>

ArmorDetector::ArmorDetector() {}

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

    // 取4个角点，按 y 排序求 top / bottom 端点
    cv::Point2f corners[4];
    rect.points(corners);
    std::sort(corners, corners + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });

    cv::Point2f top    = (corners[0] + corners[1]) * 0.5f;
    cv::Point2f bottom = (corners[2] + corners[3]) * 0.5f;
    cv::Point2f top2bottom = bottom - top;

    float length = cv::norm(top2bottom);
    float width  = cv::norm(corners[0] - corners[1]);

    if (width <= 0.1f)
        return false;

    float ratio = length / width;
    if (ratio < lightBarMinRatio_ || ratio > lightBarMaxRatio_)
        return false;

    float maxLength = frameSize.height * lightBarMaxLengthRatio_;
    if (length < lightBarMinLength_ || length > maxLength)
        return false;

    float angle       = std::atan2(top2bottom.y, top2bottom.x);
    float angle_error = std::abs(angle - CV_PI / 2);

    if (angle_error > lightBarMaxAngleError_)
        return false;

    // 忽略画面上25%的区域
    if (rect.center.y < frameSize.height * 0.25f)
        return false;

    lb.rect        = rect;
    lb.center      = rect.center;
    lb.top         = top;
    lb.bottom      = bottom;
    lb.top2bottom  = top2bottom;
    lb.length      = length;
    lb.width       = width;
    lb.angle       = angle;
    lb.angle_error = angle_error;
    lb.ratio       = ratio;
    std::copy(corners, corners + 4, lb.pts);

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

    // 1. 左右灯条长度比
    float side_ratio = std::max(pL->length, pR->length) /
                       std::min(pL->length, pR->length);
    if (side_ratio > pairMaxSideRatio_)
        return false;

    // 2. 两灯条角度差
    float angle_diff = std::abs((pL->angle - pR->angle) * 180.0f / CV_PI);
    if (angle_diff > pairMaxAngleDiff_)
        return false;

    // 3. y方向偏移
    if (dy > avgLength * pairMaxYOffsetRatio_)
        return false;

    // 4. armor ratio: 连线宽度 / 长灯条长度
    cv::Point2f left2right = pR->center - pL->center;
    float armor_width = cv::norm(left2right);
    float max_lightbar_length = std::max(pL->length, pR->length);
    float armor_ratio = armor_width / max_lightbar_length;
    if (armor_ratio < pairMinArmorRatio_ || armor_ratio > pairMaxArmorRatio_)
        return false;

    // 5. 连线与水平的夹角
    float armor_angle = std::abs(std::atan2(dy, dx) * 180.0f / CV_PI);
    if (armor_angle > pairMaxArmorAngle_)
        return false;

    // 6. dx 绝对值约束
    if (dx > frameSize.width * pairMaxDxRatio_)
        return false;

    // 7. y-投影重叠
    float topL = pL->center.y - pL->length * 0.5f;
    float botL = pL->center.y + pL->length * 0.5f;
    float topR = pR->center.y - pR->length * 0.5f;
    float botR = pR->center.y + pR->length * 0.5f;
    float overlap = std::max(0.0f, std::min(botL, botR) - std::max(topL, topR));
    if (overlap < max_lightbar_length * pairMinOverlapRatio_)
        return false;

    // 8. 宽度一致性
    float width_ratio = std::max(pL->width, pR->width) /
                        std::min(pL->width, pR->width);
    if (width_ratio > pairMaxWidthRatio_)
        return false;

    // 9. 矩形度误差
    float roll = std::atan2(left2right.y, left2right.x);
    float left_rect_error  = std::abs(pL->angle - roll - CV_PI / 2);
    float right_rect_error = std::abs(pR->angle - roll - CV_PI / 2);
    float rectangular_error = std::max(left_rect_error, right_rect_error);
    if (rectangular_error > pairMaxRectError_)
        return false;

    // 中心在上25%忽略
    cv::Point2f center = (pL->center + pR->center) * 0.5f;
    if (center.y < frameSize.height * 0.25f)
        return false;

    armor.left              = *pL;
    armor.right             = *pR;
    armor.ratio             = armor_ratio;
    armor.side_ratio        = side_ratio;
    armor.rectangular_error = rectangular_error;

    return true;
}

std::vector<ImageArmor> ArmorDetector::detect(const cv::Mat& binaryMask,
                                               const cv::Size& frameSize) {
    std::vector<ImageArmor> armors;

    // 1. 提取轮廓并筛选灯条
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int lightbar_id = 0;
    lastLightBars_.clear();
    for (const auto& contour : contours) {
        LightBar lb;
        if (isLightBar(contour, lb, frameSize)) {
            lb.id = lightbar_id++;
            lastLightBars_.push_back(lb);
        }
    }

    if (lastLightBars_.size() < 2)
        return armors;

    // 2. 灯条按 x 坐标从左到右排序
    std::sort(lastLightBars_.begin(), lastLightBars_.end(),
        [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });

    // 3. 两两配对，保留所有通过几何检查的装甲板
    std::list<ArmorCandidate> candidates;
    for (size_t i = 0; i < lastLightBars_.size(); i++) {
        for (size_t j = i + 1; j < lastLightBars_.size(); j++) {
            ArmorCandidate candidate;
            if (tryPair(lastLightBars_[i], lastLightBars_[j], candidate, frameSize)) {
                candidates.push_back(candidate);
            }
        }
    }

    // 4. 去重处理
    // 计算四边形面积的辅助函数 (corners: TL, TR, BR, BL)
    auto quadArea = [](const cv::Point2f& tl, const cv::Point2f& tr,
                        const cv::Point2f& br, const cv::Point2f& bl) -> float {
        // 拆为两个三角形: TL-TR-BL 和 TR-BR-BL
        auto cross1 = std::abs((tr.x - tl.x) * (bl.y - tl.y) - (tr.y - tl.y) * (bl.x - tl.x));
        auto cross2 = std::abs((br.x - tr.x) * (bl.y - tr.y) - (br.y - tr.y) * (bl.x - tr.x));
        return (cross1 + cross2) * 0.5f;
    };

    for (auto armor1 = candidates.begin(); armor1 != candidates.end(); ++armor1) {
        for (auto armor2 = std::next(armor1); armor2 != candidates.end(); ++armor2) {
            if (armor1->left.id != armor2->left.id &&
                armor1->left.id != armor2->right.id &&
                armor1->right.id != armor2->left.id &&
                armor1->right.id != armor2->right.id) {
                continue;  // 没有共享灯条
            }

            // 重叠: 共享左灯条或共享右灯条 → 保留四边形面积小的
            if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
                auto area1 = quadArea(armor1->left.top, armor1->right.top,
                                       armor1->right.bottom, armor1->left.bottom);
                auto area2 = quadArea(armor2->left.top, armor2->right.top,
                                       armor2->right.bottom, armor2->left.bottom);
                if (area1 < area2)
                    armor2->duplicated = true;
                else
                    armor1->duplicated = true;
            }

            // 相连: 甲板1右 = 甲板2左，或甲板1左 = 甲板2右 → 保留矩形度好的
            if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
                if (armor1->rectangular_error > armor2->rectangular_error)
                    armor1->duplicated = true;
                else
                    armor2->duplicated = true;
            }
        }
    }

    candidates.remove_if([](const ArmorCandidate& a) { return a.duplicated; });

    // 5. 构造 ImageArmor 输出，按图像中心距离排序（越靠近中心优先级越高）
    for (const auto& candidate : candidates) {
        ImageArmor armor;
        armor.corners = {
            candidate.left.top,
            candidate.right.top,
            candidate.right.bottom,
            candidate.left.bottom
        };
        armor.center = (candidate.left.center + candidate.right.center) * 0.5f;
        armor.valid  = true;
        armors.push_back(armor);
    }

    // 按距离图像中心的距离升序排列（最近的排在最前面）
    cv::Point2f img_center(frameSize.width * 0.5f, frameSize.height * 0.58f);
    std::sort(armors.begin(), armors.end(),
        [&img_center](const ImageArmor& a, const ImageArmor& b) {
            return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
        });

    return armors;
}
