#include <string>
#include <opencv2/opencv.hpp>
#include "SemiGlobalMatching.h"

int main() {
    // ··· 读取影像
    std::string pathLeft = R"(..\data\cone\im2.png)";
    std::string pathRight = R"(..\data\cone\im6.png)";

    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_GRAYSCALE);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_GRAYSCALE);

    if (imgLeft.data == nullptr || imgRight.data == nullptr) {
        std::cout << "读取影像失败！" << std::endl;
        return -1;
    }
    if (imgLeft.rows != imgRight.rows || imgLeft.cols != imgRight.cols) {
        std::cout << "左右影像尺寸不一致！" << std::endl;
        return -1;
    }

    // ··· SGM匹配
    const auto width = static_cast<sint32>(imgLeft.cols);
    const auto height = static_cast<sint32>(imgRight.rows);

    SemiGlobalMatching::SGMOption sgmOption;
    sgmOption.minDisparity = 0;
    sgmOption.maxDisparity = 64;

    SemiGlobalMatching sgm;

    // 初始化
    if (!sgm.initialize(width, height, sgmOption)) {
        std::cout << "SGM初始化失败！" << std::endl;
        return -2;
    }

    // 匹配
    auto *disparity = new float32[width * height]();
    if (!sgm.match(imgLeft.data, imgRight.data, disparity)) {
        std::cout << "SGM匹配失败！" << std::endl;
        return -2;
    }

    // 显示视差图
    cv::Mat dispMat = cv::Mat(height, width, CV_8UC1);
    for (uint32 i = 0; i < height; i++) {
        for (uint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                dispMat.data[i * width + j] = 0;
            } else {
                dispMat.data[i * width + j] = 2 * static_cast<uchar>(disp);
            }
        }
    }
    cv::imwrite("../dispMap.png", dispMat);
    cv::imshow("dispMap", dispMat);
    cv::waitKey(0);

    delete[] disparity;
    disparity = nullptr;

    return 0;
}
