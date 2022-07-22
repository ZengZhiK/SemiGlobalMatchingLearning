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

void SGMUtil::costAggregateLeftRight(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                     const sint32 &minDisparity, const sint32 &maxDisparity, const sint32 &p1,
                                     const sint32 &p2Init, const uint8 *costInit, uint8 *costAggr, bool isForward) {
    assert(width > 0 && height > 0 && maxDisparity > minDisparity);
    const sint32 dispRange = maxDisparity - minDisparity;
    // P1,P2
    const auto &P1 = p1;
    const auto &P2Init = p2Init;

    // 正向(左->右) ：isForward = true ; direction = 1
    // 反向(右->左) ：isForward = false; direction = -1;
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
                uint8 costNew =
                        cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - minCostLastPath);
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

void SGMUtil::costAggregateUpDown(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                  const sint32 &minDisparity, const sint32 &maxDisparity, const sint32 &p1,
                                  const sint32 &p2Init, const uint8 *costInit, uint8 *costAggr, bool isForward) {
    assert(width > 0 && height > 0 && maxDisparity > minDisparity);
    // 视差范围
    const sint32 dispRange = maxDisparity - minDisparity;
    // P1,P2
    const auto &P1 = p1;
    const auto &P2Init = p2Init;

    // 正向(上->下) ：isForward = true ; direction = 1
    // 反向(下->上) ：isForward = false; direction = -1;
    const sint32 direction = isForward ? 1 : -1;

    // 聚合
    for (sint32 j = 0; j < width; j++) {
        // 路径头为每一列的首(尾,dir=-1)行像素
        auto costInitCol = (isForward) ? (costInit + j * dispRange) : (costInit + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto costAggrCol = (isForward) ? (costAggr + j * dispRange) : (costAggr + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto imgCol = (isForward) ? (imgData + j) : (imgData + (height - 1) * width + j);

        // 路径上当前灰度值和上一个灰度值
        uint8 gray = *imgCol;
        uint8 grayLast = *imgCol;

        // 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
        std::vector<uint8> costLastPath(dispRange + 2, UINT8_MAX);

        // 初始化：第一个像素的聚合代价值等于初始代价值
        memcpy(costAggrCol, costInitCol, dispRange * sizeof(uint8));
        memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));
        costInitCol += direction * width * dispRange;
        costAggrCol += direction * width * dispRange;
        imgCol += direction * width;

        // 路径上上个像素的最小代价值
        uint8 minCostLastPath = UINT8_MAX;
        for (auto cost: costLastPath) {
            minCostLastPath = std::min(minCostLastPath, cost);
        }

        // 自方向上第2个像素开始按顺序聚合
        for (sint32 i = 0; i < height - 1; i++) {
            gray = *imgCol;
            uint8 minCost = UINT8_MAX;
            for (sint32 d = 0; d < dispRange; d++) {
                // Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
                const uint8 cost = costInitCol[d];

                const uint16 l1 = costLastPath[d + 1];
                const uint16 l2 = costLastPath[d] + P1;
                const uint16 l3 = costLastPath[d + 2] + P1;
                const uint16 l4 = minCostLastPath + std::max(P1, P2Init / (abs(gray - grayLast) + 1));

                const uint8 costNew =
                        cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - minCostLastPath);
                costAggrCol[d] = costNew;
                minCost = std::min(minCost, costNew);
            }

            // 重置上个像素的最小代价值和代价数组
            minCostLastPath = minCost;
            memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));

            // 下一个像素
            costInitCol += direction * width * dispRange;
            costAggrCol += direction * width * dispRange;
            imgCol += direction * width;

            // 像素值重新赋值
            grayLast = gray;
        }
    }

}

void SGMUtil::costAggregateDiagonalLeftRight(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                             const sint32 &minDisparity, const sint32 &maxDisparity, const sint32 &p1,
                                             const sint32 &p2Init, const uint8 *costInit, uint8 *costAggr,
                                             bool isForward) {
    assert(width > 1 && height > 1 && maxDisparity > minDisparity);

    // 视差范围
    const sint32 dispRange = maxDisparity - minDisparity;

    // P1,P2
    const auto &P1 = p1;
    const auto &P2Init = p2Init;

    // 正向(左上->右下) ：isForward = true ; direction = 1
    // 反向(右下->左上) ：isForward = false; direction = -1;
    const sint32 direction = isForward ? 1 : -1;

    // 聚合

    // 存储当前的行列号，判断是否到达影像边界
    sint32 currentRow = 0;
    sint32 currentCol = 0;

    for (sint32 j = 0; j < width; j++) {
        // 路径头为每一列的首(尾,dir=-1)行像素
        auto costInitCol = (isForward) ? (costInit + j * dispRange) : (costInit + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto costAggrCol = (isForward) ? (costAggr + j * dispRange) : (costAggr + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto imgCol = (isForward) ? (imgData + j) : (imgData + (height - 1) * width + j);

        // 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
        std::vector<uint8> costLastPath(dispRange + 2, UINT8_MAX);

        // 初始化：第一个像素的聚合代价值等于初始代价值
        memcpy(costAggrCol, costInitCol, dispRange * sizeof(uint8));
        memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));

        // 路径上当前灰度值和上一个灰度值
        uint8 gray = *imgCol;
        uint8 grayLast = *imgCol;

        // 对角线路径上的下一个像素，中间间隔width+1个像素
        // 这里要多一个边界处理
        // 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
        currentRow = isForward ? 0 : height - 1;
        currentCol = j;
        if (isForward && currentCol == width - 1 && currentRow < height - 1) {
            // 左上->右下，碰右边界
            costInitCol = costInit + (currentRow + direction) * width * dispRange;
            costAggrCol = costAggr + (currentRow + direction) * width * dispRange;
            imgCol = imgData + (currentRow + direction) * width;
            currentCol = 0;
        } else if (!isForward && currentCol == 0 && currentRow > 0) {
            // 右下->左上，碰左边界
            costInitCol = costInit + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
            costAggrCol = costAggr + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
            imgCol = imgData + (currentRow + direction) * width + (width - 1);
            currentCol = width - 1;
        } else {
            costInitCol += direction * (width + 1) * dispRange;
            costAggrCol += direction * (width + 1) * dispRange;
            imgCol += direction * (width + 1);
        }

        // 路径上上个像素的最小代价值
        uint8 minCostLastPath = UINT8_MAX;
        for (auto cost: costLastPath) {
            minCostLastPath = std::min(minCostLastPath, cost);
        }

        // 自方向上第2个像素开始按顺序聚合
        for (sint32 i = 0; i < height - 1; i++) {
            gray = *imgCol;
            uint8 minCost = UINT8_MAX;
            for (sint32 d = 0; d < dispRange; d++) {
                // Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
                const uint8 cost = costInitCol[d];
                const uint16 l1 = costLastPath[d + 1];
                const uint16 l2 = costLastPath[d] + P1;
                const uint16 l3 = costLastPath[d + 2] + P1;
                const uint16 l4 = minCostLastPath + std::max(P1, P2Init / (abs(gray - grayLast) + 1));

                const uint8 costNew =
                        cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - minCostLastPath);

                costAggrCol[d] = costNew;
                minCost = std::min(minCost, costNew);
            }

            // 重置上个像素的最小代价值和代价数组
            minCostLastPath = minCost;
            memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));

            // 当前像素的行列号
            currentRow += direction;
            currentCol += direction;

            // 下一个像素,这里要多一个边界处理
            // 这里要多一个边界处理
            // 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
            if (isForward && currentCol == width - 1 && currentRow < height - 1) {
                // 左上->右下，碰右边界
                costInitCol = costInit + (currentRow + direction) * width * dispRange;
                costAggrCol = costAggr + (currentRow + direction) * width * dispRange;
                imgCol = imgData + (currentRow + direction) * width;
                currentCol = 0;
            } else if (!isForward && currentCol == 0 && currentRow > 0) {
                // 右下->左上，碰左边界
                costInitCol = costInit + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
                costAggrCol = costAggr + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
                imgCol = imgData + (currentRow + direction) * width + (width - 1);
                currentCol = width - 1;
            } else {
                costInitCol += direction * (width + 1) * dispRange;
                costAggrCol += direction * (width + 1) * dispRange;
                imgCol += direction * (width + 1);
            }

            // 像素值重新赋值
            grayLast = gray;
        }
    }
}

void SGMUtil::costAggregateDiagonalRightLeft(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                             const sint32 &minDisparity, const sint32 &maxDisparity, const sint32 &p1,
                                             const sint32 &p2Init, const uint8 *costInit, uint8 *costAggr,
                                             bool isForward) {
    assert(width > 1 && height > 1 && maxDisparity > minDisparity);

    // 视差范围
    const sint32 dispRange = maxDisparity - minDisparity;

    // P1,P2
    const auto &P1 = p1;
    const auto &P2Init = p2Init;

    // 正向(右上->左下) ：isForward = true ; direction = 1
    // 反向(左下->右上) ：isForward = false; direction = -1;
    const sint32 direction = isForward ? 1 : -1;

    // 聚合

    // 存储当前的行列号，判断是否到达影像边界
    sint32 currentRow = 0;
    sint32 currentCol = 0;

    for (sint32 j = 0; j < width; j++) {
        // 路径头为每一列的首(尾,dir=-1)行像素
        auto costInitCol = (isForward) ? (costInit + j * dispRange) : (costInit + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto costAggrCol = (isForward) ? (costAggr + j * dispRange) : (costAggr + (height - 1) * width * dispRange +
                                                                       j * dispRange);
        auto imgCol = (isForward) ? (imgData + j) : (imgData + (height - 1) * width + j);

        // 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
        std::vector<uint8> costLastPath(dispRange + 2, UINT8_MAX);

        // 初始化：第一个像素的聚合代价值等于初始代价值
        memcpy(costAggrCol, costInitCol, dispRange * sizeof(uint8));
        memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));

        // 路径上当前灰度值和上一个灰度值
        uint8 gray = *imgCol;
        uint8 grayLast = *imgCol;

        // 对角线路径上的下一个像素，中间间隔width-1个像素
        // 这里要多一个边界处理
        // 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
        currentRow = isForward ? 0 : height - 1;
        currentCol = j;
        if (isForward && currentCol == 0 && currentRow < height - 1) {
            // 右上->左下，碰左边界
            costInitCol = costInit + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
            costAggrCol = costAggr + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
            imgCol = imgData + (currentRow + direction) * width + (width - 1);
            currentCol = width - 1;
        } else if (!isForward && currentCol == width - 1 && currentRow > 0) {
            // 左下->右上，碰右边界
            costInitCol = costInit + (currentRow + direction) * width * dispRange;
            costAggrCol = costAggr + (currentRow + direction) * width * dispRange;
            imgCol = imgData + (currentRow + direction) * width;
            currentCol = 0;
        } else {
            costInitCol += direction * (width - 1) * dispRange;
            costAggrCol += direction * (width - 1) * dispRange;
            imgCol += direction * (width - 1);
        }

        // 路径上上个像素的最小代价值
        uint8 minCostLastPath = UINT8_MAX;
        for (auto cost: costLastPath) {
            minCostLastPath = std::min(minCostLastPath, cost);
        }

        // 自路径上第2个像素开始按顺序聚合
        for (sint32 i = 0; i < height - 1; i++) {
            gray = *imgCol;
            uint8 minCost = UINT8_MAX;
            for (sint32 d = 0; d < dispRange; d++) {
                // Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
                const uint8 cost = costInitCol[d];
                const uint16 l1 = costLastPath[d + 1];
                const uint16 l2 = costLastPath[d] + P1;
                const uint16 l3 = costLastPath[d + 2] + P1;
                const uint16 l4 = minCostLastPath + std::max(P1, P2Init / (abs(gray - grayLast) + 1));

                const uint8 costNew =
                        cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - minCostLastPath);

                costAggrCol[d] = costNew;
                minCost = std::min(minCost, costNew);
            }

            // 重置上个像素的最小代价值和代价数组
            minCostLastPath = minCost;
            memcpy(&costLastPath[1], costAggrCol, dispRange * sizeof(uint8));

            // 当前像素的行列号
            currentRow += direction;
            currentCol -= direction;

            // 下一个像素,这里要多一个边界处理
            // 这里要多一个边界处理
            // 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
            if (isForward && currentCol == 0 && currentRow < height - 1) {
                // 右上->左下，碰左边界
                costInitCol = costInit + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
                costAggrCol = costAggr + (currentRow + direction) * width * dispRange + (width - 1) * dispRange;
                imgCol = imgData + (currentRow + direction) * width + (width - 1);
                currentCol = width - 1;
            } else if (!isForward && currentCol == width - 1 && currentRow > 0) {
                // 左下->右上，碰右边界
                costInitCol = costInit + (currentRow + direction) * width * dispRange;
                costAggrCol = costAggr + (currentRow + direction) * width * dispRange;
                imgCol = imgData + (currentRow + direction) * width;
                currentCol = 0;
            } else {
                costInitCol += direction * (width - 1) * dispRange;
                costAggrCol += direction * (width - 1) * dispRange;
                imgCol += direction * (width - 1);
            }

            // 像素值重新赋值
            grayLast = gray;
        }
    }
}

void SGMUtil::removeSpeckles(float32 *dispMap, const sint32 &width, const sint32 &height, const sint32 &diffRange,
                             const uint32 &minSpeckleArea, const float32 &invalidVal) {
    assert(width > 0 && height > 0);
    if (width < 0 || height < 0) {
        return;
    }

    // 定义标记像素是否访问的数组
    std::vector<bool> visited(uint32(width * height), false);
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            if (visited[i * width + j] || dispMap[i * width + j] == invalidVal) {
                // 跳过已访问的像素及无效像素
                continue;
            }
            // 广度优先遍历，区域跟踪
            // 把连通域面积小于阈值的区域视差全设为无效值
            std::vector<std::pair<sint32, sint32>> vec;
            vec.emplace_back(i, j);
            visited[i * width + j] = true;
            uint32 cur = 0;
            uint32 next = 0;
            do {
                // 广度优先遍历区域跟踪
                next = vec.size();
                for (uint32 k = cur; k < next; k++) {
                    const auto &pixel = vec[k];
                    const sint32 row = pixel.first;
                    const sint32 col = pixel.second;
                    const auto &dispBase = dispMap[row * width + col];
                    // 8邻域遍历
                    for (int r = -1; r <= 1; r++) {
                        for (int c = -1; c <= 1; c++) {
                            if (r == 0 && c == 0) {
                                continue;
                            }
                            int rowr = row + r;
                            int colc = col + c;
                            if (rowr >= 0 && rowr < height && colc >= 0 && colc < width) {
                                if (!visited[rowr * width + colc] &&
                                    (dispMap[rowr * width + colc] != invalidVal) &&
                                    abs(dispMap[rowr * width + colc] - dispBase) <= diffRange) {
                                    vec.emplace_back(rowr, colc);
                                    visited[rowr * width + colc] = true;
                                }
                            }
                        }
                    }
                }
                cur = next;
            } while (next < vec.size());

            // 把连通域面积小于阈值的区域视差全设为无效值
            if (vec.size() < minSpeckleArea) {
                for (auto &pix: vec) {
                    dispMap[pix.first * width + pix.second] = invalidVal;
                }
            }
        }
    }
}

void SGMUtil::medianFilter(const float32 *in, float32 *out, const sint32 &width, const sint32 &height,
                           const sint32 &wndSize) {
    const sint32 radius = wndSize / 2;
    const sint32 size = wndSize * wndSize;

    // 存储局部窗口内的数据
    std::vector<float32> wnd_data;
    wnd_data.reserve(size);

    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            wnd_data.clear();

            // 获取局部窗口数据
            for (sint32 r = -radius; r <= radius; r++) {
                for (sint32 c = -radius; c <= radius; c++) {
                    const sint32 row = i + r;
                    const sint32 col = j + c;
                    if (row >= 0 && row < height && col >= 0 && col < width) {
                        wnd_data.push_back(in[row * width + col]);
                    }
                }
            }

            // 排序
            std::sort(wnd_data.begin(), wnd_data.end());

            // 取中值
            out[i * width + j] = wnd_data[wnd_data.size() / 2];
        }
    }
}
