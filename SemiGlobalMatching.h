//
// Created by ZZK on 2022/7/10.
//

#ifndef SEMIGLOBALMATCHING_H
#define SEMIGLOBALMATCHING_H

#include "SGMType.h"
#include "SGMUtil.h"
#include <cstring>
#include <algorithm>

class SemiGlobalMatching {
public:
    SemiGlobalMatching();

    ~SemiGlobalMatching();

    struct SGMOption {
        uint8 numPaths;             // 聚合路径数 4 and 8
        sint32 minDisparity;        // 最小视差
        sint32 maxDisparity;        // 最大视差

        // P1,P2
        // P2 = P2 / (Ip-Iq)
        sint32 p1;              // 惩罚项参数P1
        sint32 p2Init;          // 惩罚项参数P2

        bool isCheckLR;        // 是否检查左右一致性
        float32 lrCheckThres;   // 左右一致性约束阈值

        bool isCheckUnique;    // 是否检查唯一性
        float32 uniquenessRatio;    // 唯一性约束阈值 （最小代价-次最小代价)/最小代价 > 阈值 为有效像素

        bool isRemoveSpeckles;  // 是否移除小的连通区
        sint32 minSpeckleArea;  // 最小的连通区面积（像素数）
        sint32 diffRange;

        SGMOption() : numPaths(8), minDisparity(0), maxDisparity(640), p1(10), p2Init(150),
                      isCheckLR(true), lrCheckThres(1.0f),
                      isCheckUnique(true), uniquenessRatio(0.95f),
                      isRemoveSpeckles(true), minSpeckleArea(20), diffRange(16) {}
    };

    /**
     * \brief 类的初始化，完成一些内存的预分配、参数的预设置等
     * \param width     输入，图像宽
     * \param height    输入，图像高
     * \param option    输入，SemiGlobalMatching参数
     */
    bool initialize(const uint32 &width, const uint32 &height, const SGMOption &option);

    /**
     * \brief 执行匹配
     * \param imgLeft      输入，左图像数据指针
     * \param imgLight     输入，右图像数据指针
     * \param dispLeft     输出，左图像视差图指针，预先分配和图像等尺寸的内存空间
     */
    bool match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft);

    /**
     * \brief 重设
     * \param width     输入，图像宽
     * \param height    输入，图像高
     * \param option    输入，SemiGlobalMatching参数
     */
    bool reset(const uint32 &width, const uint32 &height, const SGMOption &option);

private:
    /** \brief Census变换 */
    void censusTransform() const;

    /** \brief 代价计算 */
    void computeCost() const;

    /** \brief 代价聚合 */
    void costAggregation() const;

    /** \brief 视差计算 */
    void computeDisparity() const;

    /** \brief 右视差计算	 */
    void computeDisparityRight() const;

    /** \brief 一致性检查 */
    void lrCheck() const;

    /** \brief 内存释放	 */
    void Release();

    /** \brief SGM参数 */
    SGMOption option_;

    /** \brief 图像宽 */
    sint32 width_;

    /** \brief 图像高 */
    sint32 height_;

    /** \brief 左图像数据 */
    const uint8 *imgLeft_;

    /** \brief 右图像数据 */
    const uint8 *imgRight_;

    /** \brief 左图像census值 */
    uint32 *censusLeft_;

    /** \brief 右图像census值*/
    uint32 *censusRight_;

    /** \brief 初始匹配代价 */
    uint8 *costInit_;

    /** \brief 聚合匹配代价 Cost Aggregation */
    uint16 *costAggr_;

    /** \brief 左图像视差图 */
    float32 *dispLeft_;

    /** \brief 是否初始化标志 */
    bool isInitialized_;

    // ↘ ↓ ↙   5  3  7
    // →    ←	 1    2
    // ↗ ↑ ↖   8  4  6
    /** \brief 聚合匹配代价-方向1	*/
    uint8 *costAggr1_;

    /** \brief 聚合匹配代价-方向2	*/
    uint8 *costAggr2_;

    /** \brief 聚合匹配代价-方向3	*/
    uint8 *costAggr3_;

    /** \brief 聚合匹配代价-方向4	*/
    uint8 *costAggr4_;

    /** \brief 聚合匹配代价-方向5	*/
    uint8 *costAggr5_;

    /** \brief 聚合匹配代价-方向6	*/
    uint8 *costAggr6_;

    /** \brief 聚合匹配代价-方向7	*/
    uint8 *costAggr7_;

    /** \brief 聚合匹配代价-方向8	*/
    uint8 *costAggr8_;

    /** \brief 右图像视差图 */
    float32 *dispRight_;
};


#endif //SEMIGLOBALMATCHING_H
