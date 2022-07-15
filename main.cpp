#include <string>
#include <opencv2/opencv.hpp>
#include "SemiGlobalMatching.h"

void multipleImage(std::vector<cv::Mat> imgVector, cv::Mat &dst, int imgCols) {
    const int MAX_PIXEL = 300;
    int imgNum = (int) imgVector.size();
    //选择图片最大的一边 将最大的边按比例变为300像素
    cv::Size imgOriSize = imgVector[0].size();
    int imgMaxPixel = std::max(imgOriSize.height, imgOriSize.width);
    //获取最大像素变为MAX_PIXEL的比例因子
    double prop = imgMaxPixel < MAX_PIXEL ? (double) imgMaxPixel / MAX_PIXEL : MAX_PIXEL / (double) imgMaxPixel;
    cv::Size imgStdSize(imgOriSize.width * prop, imgOriSize.height * prop); //窗口显示的标准图像的Size

    cv::Mat imgStd; //标准图片
    cv::Point2i location(0, 0); //坐标点(从0,0开始)
    //构建窗口大小 通道与imageVector[0]的通道一样
    cv::Mat imgWindow(imgStdSize.height * ((imgNum - 1) / imgCols + 1), imgStdSize.width * imgCols,
                      imgVector[0].type());
    for (int i = 0; i < imgNum; i++) {
        location.x = (i % imgCols) * imgStdSize.width;
        location.y = (i / imgCols) * imgStdSize.height;
        resize(imgVector[i], imgStd, imgStdSize, prop, prop, cv::INTER_LINEAR); //设置为标准大小
        imgStd.copyTo(imgWindow(cv::Rect(location, imgStdSize)));
    }
    dst = imgWindow;
}

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
    // 聚合路径数
    sgmOption.numPaths = 8;
    // 候选视差范围
    sgmOption.minDisparity = 0;
    sgmOption.maxDisparity = 64;
    // 惩罚项P1、P2
    sgmOption.p1 = 10;
    sgmOption.p2Init = 150;

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
    float32 minDisp = FLT_MAX, maxDisp = FLT_MIN;
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp != Invalid_Float) {
                minDisp = std::min(minDisp, disp);
                maxDisp = std::max(maxDisp, disp);
            }
        }
    }
    for (uint32 i = 0; i < height; i++) {
        for (uint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                dispMat.data[i * width + j] = 0;
            } else {
                dispMat.data[i * width + j] = static_cast<uchar>((disp - minDisp) / (maxDisp - minDisp) * 255);
            }
        }
    }
    cv::imwrite("../dispMap.png", dispMat);
    cv::imshow("dispMap", dispMat);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 64, 3, 10, 150, 1, 0, 0, 0, 0,
                                                          cv::StereoSGBM::MODE_HH);
    cv::Mat dispOpenCV;
    sgbm->compute(imgLeft, imgRight, dispOpenCV);
    dispOpenCV.convertTo(dispOpenCV, CV_8U, 255 / (64 * 16.));
    std::vector<cv::Mat> dispVector = {dispMat, dispOpenCV};
    multipleImage(dispVector, dispMat, 2);
    cv::imshow("dispMapCompare", dispMat);
    cv::waitKey(0);

    delete[] disparity;
    disparity = nullptr;

    return 0;
}
