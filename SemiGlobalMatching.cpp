//
// Created by ZZK on 2022/7/10.
//

#include "SemiGlobalMatching.h"

SemiGlobalMatching::SemiGlobalMatching() {

}

SemiGlobalMatching::~SemiGlobalMatching() {
// 释放内存
    if (this->censusLeft_ != nullptr) {
        delete[] this->censusLeft_;
        this->censusLeft_ = nullptr;
    }
    if (this->censusRight_ != nullptr) {
        delete[] this->censusRight_;
        this->censusRight_ = nullptr;
    }
    if (this->costInit_ != nullptr) {
        delete[] this->costInit_;
        this->costInit_ = nullptr;
    }
    if (this->costAggr_ != nullptr) {
        delete[] this->costAggr_;
        this->costAggr_ = nullptr;
    }
    if (this->dispLeft_ != nullptr) {
        delete[] this->dispLeft_;
        this->dispLeft_ = nullptr;
    }

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
    this->costInit_ = new uint8[width * height * dispRange];
    this->costAggr_ = new uint16[width * height * dispRange];

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
    if (this->censusLeft_ != nullptr) {
        delete[] this->censusLeft_;
        this->censusLeft_ = nullptr;
    }
    if (this->censusRight_ != nullptr) {
        delete[] this->censusRight_;
        this->censusRight_ = nullptr;
    }
    if (this->costInit_ != nullptr) {
        delete[] this->costInit_;
        this->costInit_ = nullptr;
    }
    if (this->costAggr_ != nullptr) {
        delete[] this->costAggr_;
        this->costAggr_ = nullptr;
    }
    if (this->dispLeft_ != nullptr) {
        delete[] this->dispLeft_;
        this->dispLeft_ = nullptr;
    }

    // 重置初始化标记
    this->isInitialized_ = false;

    // 初始化
    return initialize(width, height, option);
}

void SemiGlobalMatching::censusTransform() const {
    SGMUtil::census_transform_5x5(imgLeft_, censusLeft_, width_, height_);
    SGMUtil::census_transform_5x5(imgRight_, censusRight_, width_, height_);
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
                cost = SGMUtil::Hamming32(censusValLeft, censusValRight);
            }
        }
    }
}

void SemiGlobalMatching::costAggregation() const {

}

void SemiGlobalMatching::computeDisparity() const {
    const sint32 &minDisparity = option_.minDisparity;
    const sint32 &maxDisparity = option_.maxDisparity;
    const sint32 dispRange = maxDisparity - minDisparity;

    uint8 *costPtr = costInit_;

    // 逐像素计算最优视差
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            uint16 minCost = UINT16_MAX;
            uint16 maxCost = 0;
            sint32 bestDisparity = 0;

            // 遍历视差范围内的所有代价值，输出最小代价值及对应的视差值
            for (sint32 d = minDisparity; d < maxDisparity; d++) {
                uint8 &cost = costPtr[i * width_ * dispRange + j * dispRange + (d - minDisparity)];
                if (minCost > cost) {
                    minCost = cost;
                    bestDisparity = d;
                }
                maxCost = std::max(maxCost, static_cast<uint16>(cost));
            }

            // 最小代价值对应的视差值即为像素的最优视差
            if (maxCost != minCost) {
                dispLeft_[i * width_ + j] = static_cast<float>(bestDisparity);
            }
            else {
                // 如果所有视差下的代价值都一样，则该像素无效
                dispLeft_[i * width_ + j] = Invalid_Float;
            }
        }
    }
}

void SemiGlobalMatching::lrCheck() const {

}
