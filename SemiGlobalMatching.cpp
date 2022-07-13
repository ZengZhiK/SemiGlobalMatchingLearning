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
    this->costInit_ = new uint8[size];
    this->costAggr_ = new uint16[size];
    this->costAggr1_ = new uint8[size];
    this->costAggr2_ = new uint8[size];
    this->costAggr3_ = new uint8[size];
    this->costAggr4_ = new uint8[size];
    this->costAggr5_ = new uint8[size];
    this->costAggr6_ = new uint8[size];
    this->costAggr7_ = new uint8[size];
    this->costAggr8_ = new uint8[size];

    // 视差图
    this->dispLeft_ = new float32[width * height];

    this->isInitialized_ =
            this->censusLeft_ && this->censusRight_ && this->costInit_ && this->costAggr_ && this->dispLeft_;

    return this->isInitialized_;
}

bool SemiGlobalMatching::match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft) {
    if (!this->isInitialized_) {
        return false;
    }

    if (imgLeft == nullptr || imgRight == nullptr) {
        return false;
    }

    this->imgLeft_ = imgLeft;
    this->imgRight_ = imgRight;

    // census变换
    censusTransform();

    // 代价计算
    computeCost();

    // 代价聚合
    costAggregation();

    // 视差计算
    computeDisparity();

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

void SemiGlobalMatching::computeCost() const {
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
                uint8 &cost = costInit_[i * width_ * dispRange + j * dispRange + (d - minDisparity)];
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
//    SGMUtil::costAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_,
//                                 cost_aggr_3_, true);
//    SGMUtil::costAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_,
//                                 cost_aggr_4_, false);


    // 把4/8个方向加起来
    for (sint32 i = 0; i < size; i++) {
        costAggr_[i] = costAggr1_[i] + costAggr2_[i];
//        costAggr_[i] = costAggr1_[i] + costAggr2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
//        if (option_.numPaths == 8) {
//            costAggr_[i] += cost_aggr_5_[i] + cost_aggr_6_[i] + cost_aggr_7_[i] + cost_aggr_8_[i];
//        }
    }
}

void SemiGlobalMatching::computeDisparity() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

    const auto *costPtr = costAggr_;

    // 逐像素计算最优视差
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            uint16 minCost = UINT16_MAX;
            uint16 maxCost = 0;
            sint32 bestDisparity = 0;

            // 遍历视差范围内的所有代价值，输出最小代价值及对应的视差值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                auto &cost = costPtr[i * width_ * dispRange + j * dispRange + (d - minDisparity)];
                if (minCost > cost) {
                    minCost = cost;
                    bestDisparity = d;
                }
                maxCost = std::max(maxCost, static_cast<uint16>(cost));
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

void SemiGlobalMatching::lrCheck() const {

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
}
