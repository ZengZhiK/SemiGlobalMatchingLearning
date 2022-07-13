//
// Created by tianhe on 2022/7/12.
//

#include "SGMUtil.h"

void SGMUtil::census_transform_5x5(const uint8 *source, uint32 *census, const uint32 &width, const uint32 &height) {
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

uint8 SGMUtil::Hamming32(const uint32 &x, const uint32 &y) {
    uint32 dist = 0, val = x ^ y;

    while (val != 0) {
        ++dist;
        val &= val - 1;
    }

    return dist;
}
