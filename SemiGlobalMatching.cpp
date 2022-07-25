//
// Created by ZZK on 2022/7/10.
//

#include "SemiGlobalMatching.h"

#define SAFE_DELETE(P) if(P!=nullptr){delete[](P); (P)=nullptr;}

SemiGlobalMatching::SemiGlobalMatching() {

}

SemiGlobalMatching::~SemiGlobalMatching() {
    // 释放内存
    Release();

    // 重置初始化标记
    this->isInitialized_ = false;
}

bool
SemiGlobalMatching::initialize(const uint32 &width, const uint32 &height, const SemiGlobalMatching::SGMOption &option) {
    // ··· 赋值

    // 图像尺寸
    this->width_ = width;
    this->height_ = height;
    // SGM参数
    this->option_ = option;

    if (width == 0 || height == 0) {
        return false;
    }

    //··· 开辟内存空间

    // census值（左右图像）
    this->censusLeft_ = new uint32[width * height];
    this->censusRight_ = new uint32[width * height];

    // 匹配代价（初始/聚合）
    const sint32 dispRange = option.maxDisparity - option.minDisparity;
    if (dispRange <= 0) {
        return false;
    }
    const uint32 size = width * height * dispRange;
    this->costInit_ = new float32[size];
    this->costAggr_ = new float32[size];
    this->costAggr1_ = new float32[size];
    this->costAggr2_ = new float32[size];
    this->costAggr3_ = new float32[size];
    this->costAggr4_ = new float32[size];
    this->costAggr5_ = new float32[size];
    this->costAggr6_ = new float32[size];
    this->costAggr7_ = new float32[size];
    this->costAggr8_ = new float32[size];

    // 视差图
    this->dispLeft_ = new float32[width * height];
    this->dispRight_ = new float32[width * height];

    this->isInitialized_ =
            this->censusLeft_ && this->censusRight_ && this->costInit_ && this->costAggr_ && this->dispLeft_;

    return this->isInitialized_;
}

//bool SemiGlobalMatching::match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft) {
//    if (!this->isInitialized_) {
//        return false;
//    }
//
//    if (imgLeft == nullptr || imgRight == nullptr) {
//        return false;
//    }
//
//    this->imgLeft_ = imgLeft;
//    this->imgRight_ = imgRight;
//
//    // census变换
//    censusTransform();
//
//    // 代价计算
//    computeCostCensus();
//
//    // 代价聚合
//    costAggregation();
//
//    // 视差计算
//    computeDisparity();
//
//    // 左右一致性检查
//    if (option_.isCheckLR) {
//        // 视差计算（右影像）
//        computeDisparityRight();
//        // 一致性检查
//        lrCheck();
//    }
//
//    // 移除小连通区
//    if (option_.isRemoveSpeckles) {
//        SGMUtil::removeSpeckles(dispLeft_, width_, height_, option_.diffRange, option_.minSpeckleArea, Invalid_Float);
//    }
//
//    // 中值滤波
//    SGMUtil::medianFilter(dispLeft_, dispLeft_, width_, height_, 3);
//
//    memcpy(dispLeft, this->dispLeft_, this->width_ * this->height_ * sizeof(float32));
//
//    return true;
//}

//bool SemiGlobalMatching::match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft) {
//    if (!this->isInitialized_) {
//        return false;
//    }
//
//    if (imgLeft == nullptr || imgRight == nullptr) {
//        return false;
//    }
//
//    this->imgLeft_ = imgLeft;
//    this->imgRight_ = imgRight;
//
//    computeCostSAD();
//
//    // 代价聚合
//    costAggregation();
//
//    // 视差计算
////    computeDisparityBase();
//
//    // 视差计算
//    computeDisparity();
//
//    // 左右一致性检查
//    if (option_.isCheckLR) {
//        // 视差计算（右影像）
//        computeDisparityRight();
//        // 一致性检查
//        lrCheck();
//    }
//
//    // 移除小连通区
//    if (option_.isRemoveSpeckles) {
//        SGMUtil::removeSpeckles(dispLeft_, width_, height_, option_.diffRange, option_.minSpeckleArea, Invalid_Float);
//    }
//
//    // 中值滤波
//    SGMUtil::medianFilter(dispLeft_, dispLeft_, width_, height_, 3);
//
//    memcpy(dispLeft, this->dispLeft_, this->width_ * this->height_ * sizeof(float32));
//
//    return true;
//}

bool SemiGlobalMatching::match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft) {
    if (!this->isInitialized_) {
        return false;
    }

    if (imgLeft == nullptr || imgRight == nullptr) {
        return false;
    }

    this->imgLeft_ = imgLeft;
    this->imgRight_ = imgRight;

    computeCostNCC();

    // 代价聚合
    costAggregation();

    // 视差计算
//    computeDisparityBase();

    // 视差计算
    computeDisparity();

    // 左右一致性检查
    if (option_.isCheckLR) {
        // 视差计算（右影像）
        computeDisparityRight();
        // 一致性检查
        lrCheck();
    }

    // 移除小连通区
    if (option_.isRemoveSpeckles) {
        SGMUtil::removeSpeckles(dispLeft_, width_, height_, option_.diffRange, option_.minSpeckleArea, Invalid_Float);
    }

    // 中值滤波
    SGMUtil::medianFilter(dispLeft_, dispLeft_, width_, height_, 3);

    memcpy(dispLeft, this->dispLeft_, this->width_ * this->height_ * sizeof(float32));

    return true;
}

bool SemiGlobalMatching::reset(const uint32 &width, const uint32 &height, const SemiGlobalMatching::SGMOption &option) {
    // 释放内存
    Release();

    // 重置初始化标记
    this->isInitialized_ = false;

    // 初始化
    return initialize(width, height, option);
}

void SemiGlobalMatching::censusTransform() const {
    SGMUtil::censusTransform5x5(imgLeft_, censusLeft_, width_, height_);
    SGMUtil::censusTransform5x5(imgRight_, censusRight_, width_, height_);
}

void SemiGlobalMatching::computeCostCensus() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

    // 计算代价（基于Hamming距离）
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            // 左图像census值
            const uint32 censusValLeft = censusLeft_[i * width_ + j];

            // 逐视差计算代价值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                auto &cost = costInit_[i * width_ * dispRange + j * dispRange + (d - minDisparity)];
                if (j - d < 0 || j - d >= width_) {
                    cost = UINT8_MAX / 2;
                    continue;
                }
                // 右图像对应像点的census值
                const uint32 censusValRight = censusRight_[i * width_ + j - d];
                // 计算匹配代价
                cost = SGMUtil::hamming32(censusValLeft, censusValRight);
            }
        }
    }
}

void SemiGlobalMatching::costAggregation() const {
    // 路径聚合
    // 1、左->右/右->左
    // 2、上->下/下->上
    // 3、左上->右下/右下->左上
    // 4、右上->左上/左下->右上
    //
    // ↘ ↓ ↙   5  3  7
    // →    ←	 1    2
    // ↗ ↑ ↖   8  4  6
    //
    const auto &minDisparity = option_.minDisparity;
    const auto &maxDisparity = option_.maxDisparity;
    assert(maxDisparity > minDisparity);

    const uint32 size = width_ * height_ * (maxDisparity - minDisparity);
    if (size <= 0) {
        return;
    }

    const auto &P1 = option_.p1;
    const auto &P2Int = option_.p2Init;

    // 左右聚合
    SGMUtil::costAggregateLeftRight(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                    costAggr1_, true);
    SGMUtil::costAggregateLeftRight(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                    costAggr2_, false);
    // 上下聚合
    SGMUtil::costAggregateUpDown(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                 costAggr3_, true);
    SGMUtil::costAggregateUpDown(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                 costAggr4_, false);

    // 左上到右下对角线聚合
    SGMUtil::costAggregateDiagonalLeftRight(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                            costAggr5_, true);
    SGMUtil::costAggregateDiagonalLeftRight(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                            costAggr6_, false);

    // 右上到左下对角线聚合
    SGMUtil::costAggregateDiagonalRightLeft(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                            costAggr7_, true);
    SGMUtil::costAggregateDiagonalRightLeft(imgLeft_, width_, height_, minDisparity, maxDisparity, P1, P2Int, costInit_,
                                            costAggr8_, false);


    // 把4/8个方向加起来
    for (sint32 i = 0; i < size; i++) {
        if (option_.numPaths == 4) {
            costAggr_[i] = costAggr1_[i] + costAggr2_[i] + costAggr3_[i] + costAggr4_[i];
        }
        if (option_.numPaths == 8) {
            costAggr_[i] =
                    costAggr1_[i] + costAggr2_[i] + costAggr3_[i] + costAggr4_[i] + costAggr5_[i] + costAggr6_[i] +
                    costAggr7_[i] + costAggr8_[i];
        }
    }
}

void SemiGlobalMatching::computeDisparityBase() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

//    auto *costPtr = costInit_;
    auto *costPtr = costAggr_;

    // 逐像素计算最优视差
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            float32 minCost = UINT16_MAX;
            float32 maxCost = 0;
            sint32 bestDisparity = 0;

            // 遍历视差范围内的所有代价值，输出最小代价值及对应的视差值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                auto &cost = costPtr[i * width_ * dispRange + j * dispRange + (d - minDisparity)];
                if (minCost > cost) {
                    minCost = cost;
                    bestDisparity = d;
                }
                maxCost = std::max(maxCost, static_cast<float32>(cost));
            }

            // 最小代价值对应的视差值即为像素的最优视差
            if (maxCost != minCost) {
                dispLeft_[i * width_ + j] = static_cast<float>(bestDisparity);
            } else {
                // 如果所有视差下的代价值都一样，则该像素无效
                dispLeft_[i * width_ + j] = Invalid_Float;
            }
        }
    }
}

void SemiGlobalMatching::computeDisparity() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;
    if (dispRange <= 0) {
        return;
    }

    // 左影像视差图
    const auto disparity = dispLeft_;
    // 左影像聚合代价数组
    const auto costPtr = costAggr_;

    const sint32 width = width_;
    const sint32 height = height_;

    const bool isCheckUnique = option_.isCheckUnique;
    const float32 uniquenessRatio = option_.uniquenessRatio;

    // 为了加快读取效率，把单个像素的所有代价值存储到局部数组里
    std::vector<uint16> costLocal(dispRange);

    // ---逐像素计算最优视差
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 minCost = UINT16_MAX;
            uint16 secMinCost = UINT16_MAX;
            sint32 bestDisparity = 0;

            // ---遍历视差范围内的所有代价值，输出最小代价值及对应的视差值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                const sint32 d_idx = d - minDisparity;
                const auto &cost = costLocal[d_idx] = costPtr[i * width * dispRange + j * dispRange + d_idx];
                if (minCost > cost) {
                    minCost = cost;
                    bestDisparity = d;
                }
            }

            if (isCheckUnique) {
                // 再遍历一次，输出次最小代价值
                for (sint32 d = minDisparity; d < maxDisparity; d++) {
                    if (d == bestDisparity) {
                        // 跳过最小代价值
                        continue;
                    }
                    const auto &cost = costLocal[d - minDisparity];
                    secMinCost = std::min(secMinCost, cost);
                }

                // 判断唯一性约束
                // 若(min-sec)/min < min*(1-uniquness)，则为无效估计
                if (secMinCost - minCost <= static_cast<uint16>(minCost * (1 - uniquenessRatio))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }

            // ---子像素拟合
            if (bestDisparity == minDisparity || bestDisparity == maxDisparity - 1) {
                disparity[i * width + j] = Invalid_Float;
                continue;
            }
            // 最优视差前一个视差的代价值cost_1，后一个视差的代价值cost_2
            const sint32 idx_1 = bestDisparity - 1 - minDisparity;
            const sint32 idx_2 = bestDisparity + 1 - minDisparity;
            const uint16 cost_1 = costLocal[idx_1];
            const uint16 cost_2 = costLocal[idx_2];
            // 解一元二次曲线极值
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * minCost);
            disparity[i * width + j] =
                    static_cast<float32>(bestDisparity) + static_cast<float32>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SemiGlobalMatching::computeDisparityRight() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;
    if (dispRange <= 0) {
        return;
    }

    // 左影像视差图
    const auto disparity = dispRight_;
    // 左影像聚合代价数组
    const auto costPtr = costAggr_;

    const sint32 width = width_;
    const sint32 height = height_;

    const bool isCheckUnique = option_.isCheckUnique;
    const float32 uniquenessRatio = option_.uniquenessRatio;

    // 为了加快读取效率，把单个像素的所有代价值存储到局部数组里
    std::vector<uint16> costLocal(dispRange);

    // ---逐像素计算最优视差
    // 通过左影像的代价，获取右影像的代价
    // 右cost(xr,yr,d) = 左cost(xr+d,yl,d)
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 minCost = UINT16_MAX;
            uint16 secMinCost = UINT16_MAX;
            sint32 bestDisparity = 0;

            // ---统计候选视差下的代价值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                const sint32 dIdx = d - minDisparity;
                const sint32 colLeft = j + d;
                if (colLeft >= 0 && colLeft < width) {
                    const auto &cost = costLocal[dIdx] = costPtr[i * width * dispRange + colLeft * dispRange +
                                                                 dIdx];
                    if (minCost > cost) {
                        minCost = cost;
                        bestDisparity = d;
                    }
                } else {
                    costLocal[dIdx] = UINT16_MAX;
                }
            }

            if (isCheckUnique) {
                // 再遍历一次，输出次最小代价值
                for (sint32 d = minDisparity; d < maxDisparity; d++) {
                    if (d == bestDisparity) {
                        // 跳过最小代价值
                        continue;
                    }
                    const auto &cost = costLocal[d - minDisparity];
                    secMinCost = std::min(secMinCost, cost);
                }

                // 判断唯一性约束
                // 若(min-sec)/min < min*(1-uniquness)，则为无效估计
                if (secMinCost - minCost <= static_cast<uint16>(minCost * (1 - uniquenessRatio))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }

            // ---子像素拟合
            if (bestDisparity == minDisparity || bestDisparity == maxDisparity - 1) {
                disparity[i * width + j] = Invalid_Float;
                continue;
            }
            // 最优视差前一个视差的代价值cost_1，后一个视差的代价值cost_2
            const sint32 idx_1 = bestDisparity - 1 - minDisparity;
            const sint32 idx_2 = bestDisparity + 1 - minDisparity;
            const uint16 cost_1 = costLocal[idx_1];
            const uint16 cost_2 = costLocal[idx_2];
            // 解一元二次曲线极值
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * minCost);
            disparity[i * width + j] =
                    static_cast<float32>(bestDisparity) + static_cast<float32>(cost_1 - cost_2 ) / (denom * 2.0f);

        }
    }
}

void SemiGlobalMatching::lrCheck() const {
    const int width = width_;
    const int height = height_;

    const float32 &threshold = option_.lrCheckThres;

    // ---左右一致性检查
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            // 左影像视差值
            auto &disp = dispLeft_[i * width + j];

            // 根据视差值找到右影像上对应的同名像素
            const auto colRight = static_cast<sint32>(j - disp + 0.5);

            if (colRight >= 0 && colRight < width) {
                // 右影像上同名像素的视差值
                const auto &disp_r = dispRight_[i * width + colRight];

                // 判断两个视差值是否一致（差值在阈值内）
                if (abs(disp - disp_r) > threshold) {
                    // 左右不一致
                    disp = Invalid_Float;
                }
            } else {
                // 通过视差值在右影像上找不到同名像素（超出影像范围）
                disp = Invalid_Float;
            }
        }
    }
}

void SemiGlobalMatching::Release() {
    // 释放内存
    SAFE_DELETE(censusLeft_);
    SAFE_DELETE(censusRight_);
    SAFE_DELETE(costInit_);
    SAFE_DELETE(costAggr_);
    SAFE_DELETE(costAggr1_);
    SAFE_DELETE(costAggr2_);
    SAFE_DELETE(costAggr3_);
    SAFE_DELETE(costAggr4_);
    SAFE_DELETE(costAggr5_);
    SAFE_DELETE(costAggr6_);
    SAFE_DELETE(costAggr7_);
    SAFE_DELETE(costAggr8_);
    SAFE_DELETE(dispLeft_);
    SAFE_DELETE(dispRight_);
}

void SemiGlobalMatching::computeCostSAD() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

    // 计算代价（基于SAD）
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            // 逐视差计算代价值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                auto &cost = costInit_[i * width_ * dispRange + j * dispRange + (d - minDisparity)];

                // 中心点超出图像范围，无法匹配
                if (j - d < 0 || j - d >= width_) {
                    cost = 2;
                    continue;
                }

                uint16 sum = 0;
                sint32 rL = i, cL = j, rR = i, cR = j - d;
                for (sint32 r = -1; r <= 1; r++) {
                    for (sint32 c = -1; c <= 1; c++) {
                        if (rL + r >= 0 && rL + r < height_ && cL + c >= 0 && cL + c < width_) {
                            if (rR + r >= 0 && rR + r < height_ && cR + c >= 0 && cR + c < width_) {
                                sum += (uint16) std::abs(imgLeft_[(rL + r) * width_ + (cL + c)] -
                                                         imgRight_[(rR + r) * width_ + (cR + c)]);
                            }
                        }
                    }
                }

                cost = sum;
            }
        }
    }
}

void SemiGlobalMatching::computeCostNCC() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

    // 计算代价（基于NCC）
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            // 逐视差计算代价值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                auto &cost = costInit_[i * width_ * dispRange + j * dispRange + (d - minDisparity)];

                // 中心点超出图像范围，无法匹配
                if (j - d < 0 || j - d >= width_) {
                    cost = UINT8_MAX / 2;
                    continue;
                }

                float32 avg1 = 0, avg2 = 0;
                const sint32 rL = i, cL = j, rR = i, cR = j - d;
                for (sint32 r = -1; r <= 1; r++) {
                    for (sint32 c = -1; c <= 1; c++) {
                        if (rL + r >= 0 && rL + r < height_ && cL + c >= 0 && cL + c < width_) {
                            if (rR + r >= 0 && rR + r < height_ && cR + c >= 0 && cR + c < width_) {
                                avg1 += imgLeft_[(rL + r) * width_ + (cL + c)];
                                avg2 += imgRight_[(rR + r) * width_ + (cR + c)];
                            }
                        }
                    }
                }
                avg1 /= 9;
                avg2 /= 9;

                float32 ncc1 = 0, ncc2 = 0, ncc3 = 0, ncc = 0;
                for (sint32 r = -1; r <= 1; r++) {
                    for (sint32 c = -1; c <= 1; c++) {
                        if (rL + r >= 0 && rL + r < height_ && cL + c >= 0 && cL + c < width_) {
                            if (rR + r >= 0 && rR + r < height_ && cR + c >= 0 && cR + c < width_) {
                                ncc1 += (imgLeft_[(rL + r) * width_ + (cL + c)] - avg1) *
                                        (imgRight_[(rR + r) * width_ + (cR + c)] - avg2);
                                ncc2 += (imgLeft_[(rL + r) * width_ + (cL + c)] - avg1) *
                                        (imgLeft_[(rL + r) * width_ + (cL + c)] - avg1);
                                ncc3 += (imgRight_[(rR + r) * width_ + (cR + c)] - avg2) *
                                        (imgRight_[(rR + r) * width_ + (cR + c)] - avg2);
                            }
                        }
                    }
                }
                ncc = ncc1 / std::sqrt(ncc2 * ncc3);

                if (std::isnan(ncc)) {
                    cost = UINT8_MAX / 2;
                } else {
                    cost = (-ncc + 1) * 64;
                }
            }
        }
    }
}
