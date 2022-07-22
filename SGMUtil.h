//
// Created by tianhe on 2022/7/12.
//

#ifndef SGMUTIL_H
#define SGMUTIL_H

#include "SGMType.h"
#include <cassert>
#include <vector>
#include <cstring>
#include <algorithm>

namespace SGMUtil {
    //······ census工具集
    // census变换

    /**
     * \brief census变换
     * \param source    输入，图像数据
     * \param census    输出，census值数组
     * \param width     输入，图像宽
     * \param height    输入，图像高
     */
    void censusTransform5x5(const uint8 *source, uint32 *census, const uint32 &width, const uint32 &height);

    void censusTransform9x7(const uint8 *source, uint64 *census, const uint32 &width, const uint32 &height);

    // Hamming距离
    uint8 hamming32(const uint32 &x, const uint32 &y);

    uint8 hamming64(const uint64 &x, const uint64 &y);

    /**
     * \brief 左右路径聚合 → ←
     * \param img_data          输入，影像数据
     * \param width             输入，影像宽
     * \param height            输入，影像高
     * \param min_disparity     输入，最小视差
     * \param max_disparity     输入，最大视差
     * \param p1                输入，惩罚项P1
     * \param p2_init           输入，惩罚项P2_Init
     * \param cost_init         输入，初始代价数据
     * \param cost_aggr         输出，路径聚合代价数据
     * \param is_forward        输入，是否为正方向（正方向为从左到右，反方向为从右到左）
     */
    void
    costAggregateLeftRight(const uint8 *imgData, const sint32 &width, const sint32 &height, const sint32 &minDisparity,
                           const sint32 &maxDisparity, const sint32 &p1, const sint32 &p2Init, const uint8 *costInit,
                           uint8 *costAggr, bool isForward);

    /**
     * \brief 上下路径聚合 ↓ ↑
     * \param img_data           输入，影像数据
     * \param width              输入，影像宽
     * \param height             输入，影像高
     * \param min_disparity      输入，最小视差
     * \param max_disparity      输入，最大视差
     * \param p1                 输入，惩罚项P1
     * \param p2_init            输入，惩罚项P2_Init
     * \param cost_init          输入，初始代价数据
     * \param cost_aggr          输出，路径聚合代价数据
     * \param is_forward         输入，是否为正方向（正方向为从上到下，反方向为从下到上）
     */
    void
    costAggregateUpDown(const uint8 *imgData, const sint32 &width, const sint32 &height, const sint32 &minDisparity,
                        const sint32 &maxDisparity, const sint32 &p1, const sint32 &p2Init, const uint8 *costInit,
                        uint8 *costAggr, bool isForward);

    /**
     * \brief 对角线1路径聚合（左上<->右下）↘ ↖
     * \param img_data			输入，影像数据
     * \param width				输入，影像宽
     * \param height			输入，影像高
     * \param min_disparity		输入，最小视差
     * \param max_disparity		输入，最大视差
     * \param p1				输入，惩罚项P1
     * \param p2_init			输入，惩罚项P2_Init
     * \param cost_init			输入，初始代价数据
     * \param cost_aggr			输出，路径聚合代价数据
     * \param is_forward		输入，是否为正方向（正方向为从左上到右下，反方向为从右下到左上）
     */
    void
    costAggregateDiagonalLeftRight(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                   const sint32 &minDisparity,
                                   const sint32 &maxDisparity, const sint32 &p1, const sint32 &p2Init,
                                   const uint8 *costInit,
                                   uint8 *costAggr, bool isForward);

    /**
     * \brief 对角线2路径聚合（右上<->左下）↙ ↗
     * \param img_data			输入，影像数据
     * \param width				输入，影像宽
     * \param height			输入，影像高
     * \param min_disparity		输入，最小视差
     * \param max_disparity		输入，最大视差
     * \param p1				输入，惩罚项P1
     * \param p2_init			输入，惩罚项P2_Init
     * \param cost_init			输入，初始代价数据
     * \param cost_aggr			输出，路径聚合代价数据
     * \param is_forward		输入，是否为正方向（正方向为从上到下，反方向为从下到上）
     */
    void
    costAggregateDiagonalRightLeft(const uint8 *imgData, const sint32 &width, const sint32 &height,
                                   const sint32 &minDisparity,
                                   const sint32 &maxDisparity, const sint32 &p1, const sint32 &p2Init,
                                   const uint8 *costInit,
                                   uint8 *costAggr, bool isForward);

    /**
     * \brief 剔除小连通区
     * \param disparity_map		输入，视差图
     * \param width				输入，宽度
     * \param height			输入，高度
     * \param diff_insame		输入，同一连通区内的局部像素差异
     * \param min_speckle_aera	输入，最小连通区面积
     * \param invalid_val		输入，无效值
     */
    void removeSpeckles(float32 *dispMap, const sint32 &width, const sint32 &height, const sint32 &diffRange,
                        const uint32 &minSpeckleArea, const float32 &invalidVal);

    /**
     * \brief 中值滤波
     * \param in				输入，源数据
     * \param out				输出，目标数据
     * \param width				输入，宽度
     * \param height			输入，高度
     * \param wnd_size			输入，窗口宽度
     */
    void medianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 &wndSize);
}


#endif //SGMUTIL_H
