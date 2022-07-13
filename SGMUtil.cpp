//
// Created by tianhe on 2022/7/12.
//

#include "SGMUtil.h"

void SGMUtil::censusTransform5x5(const uint8 *source, uint32 *census, const uint32 &width, const uint32 &height) {
    if (source == nullptr || census == nullptr || width <= 5u || height <= 5u) {
        return;
    }

    // 逐像素计算census值
    for (sint32 i = 2; i < height - 2; i++) {
        for (sint32 j = 2; j < width - 2; j++) {
            // 中心像素值
            const uint8 grayCenter = source[i * width + j];

            // 遍历大小为5x5的窗口内邻域像素，逐一比较像素值与中心像素值的的大小，计算census值
            uint32 censusVal = 0u;
            for (sint32 r = -2; r <= 2; r++) {
                for (sint32 c = -2; c <= 2; c++) {
                    censusVal <<= 1;
                    const uint8 gray = source[(i + r) * width + (j + c)];
                    if (gray < grayCenter) {
                        censusVal += 1;
                    }
                }
            }

            // 中心像素的census值
            census[i * width + j] = censusVal;
        }
    }

}

uint8 SGMUtil::hamming32(const uint32 &x, const uint32 &y) {
    uint32 dist = 0, val = x ^ y;

    while (val != 0) {
        ++dist;
        val &= val - 1;
    }

    return dist;
}

void SGMUtil::costAggregateLeftRight(const uint8 *imgData, const uint32 &width, const uint32 &height,
                                     const sint32 &minDisparity, const sint32 &maxDisparity, const sint32 &p1,
                                     const sint32 &p2Init, const uint8 *costInit, uint8 *costAggr, bool isForward) {
    assert(width > 0 && height > 0 && maxDisparity > minDisparity);
    const sint32 dispRange = maxDisparity - minDisparity;
    // P1,P2
    const auto &P1 = p1;
    const auto &P2Init = p2Init;

    // 正向(左->右) ：is_forward = true ; direction = 1
    // 反向(右->左) ：is_forward = false; direction = -1;
    const sint32 direction = isForward ? 1 : -1;

    // 聚合
    for (sint32 i = 0; i < height; i++) {
        // 路径头为每一行的首(尾,dir=-1)列像素
        auto costInitRow = (isForward) ? (costInit + i * width * dispRange) : (costInit + i * width * dispRange +
                                                                               (width - 1) * dispRange);
        auto costAggrRow = (isForward) ? (costAggr + i * width * dispRange) : (costAggr + i * width * dispRange +
                                                                               (width - 1) * dispRange);
        auto imgRow = (isForward) ? (imgData + i * width) : (imgData + i * width + (width - 1));

        // 路径上当前灰度值和上一个灰度值
        uint8 gray = *imgRow;
        uint8 grayLast = *imgRow;

        // 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
        std::vector<uint8> costLastPath(dispRange + 2, UINT8_MAX);
        // 初始化：第一个像素的聚合代价值等于初始代价值
        memcpy(costAggrRow, costInitRow, dispRange * sizeof(uint8));
        memcpy(&costLastPath[1], costAggrRow, dispRange * sizeof(uint8));
        costInitRow += direction * dispRange;
        costAggrRow += direction * dispRange;
        imgRow += direction;

        // 路径上上个像素的最小代价值
        uint8 minCostLastPath = UINT8_MAX;
        for (auto cost: costLastPath) {
            minCostLastPath = std::min(minCostLastPath, cost);
        }

        // 自方向上第2个像素开始按顺序聚合
        for (sint32 j = 0; j < width - 1; j++) {
            gray = *imgRow;
            uint8 minCost = UINT8_MAX;
            for (sint32 d = 0; d < dispRange; d++) {
                // Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
                const uint8 cost = costInitRow[d];

                const uint16 l1 = costLastPath[d + 1];
                const uint16 l2 = costLastPath[d] + P1;
                const uint16 l3 = costLastPath[d + 2] + P1;
                const uint16 l4 = minCostLastPath + std::max(P1, P2Init / (abs(gray - grayLast) + 1));
                uint8 costNew = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - minCostLastPath);
                costAggrRow[d] = costNew;
                minCost = std::min(minCost, costNew);
            }
            // 重置上个像素的最小代价值和代价数组
            minCostLastPath = minCost;
            memcpy(&costLastPath[1], costAggrRow, dispRange * sizeof(uint8));

            // 下一个像素
            costInitRow += direction * dispRange;
            costAggrRow += direction * dispRange;
            imgRow += direction;

            // 像素值重新赋值
            grayLast = gray;
        }
    }
}
