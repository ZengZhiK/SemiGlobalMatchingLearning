//
// Created by tianhe on 2022/7/12.
//

#ifndef SGMUTIL_H
#define SGMUTIL_H

#include "SGMType.h"

namespace SGMUtil {
    //······ census工具集
    // census变换

    /**
     * \brief census变换
     * \param source	输入，图像数据
     * \param census	输出，census值数组
     * \param width		输入，图像宽
     * \param height	输入，图像高
     */
    void census_transform_5x5(const uint8 *source, uint32 *census, const uint32 &width, const uint32 &height);

    void census_transform_9x7(const uint8 *source, uint64 *census, const uint32 &width, const uint32 &height);

    // Hamming距离
    uint8 Hamming32(const uint32 &x, const uint32 &y);

    uint8 Hamming64(const uint64 &x, const uint64 &y);
};


#endif //SGMUTIL_H
