//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width  = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        /**
         * Lecture 09 Shading 3
         * 00:26:35
         * 双线性插值
         */
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto x_img = u * width;
        auto y_img = (1 - v) * height;

        int xMin = static_cast<int>(std::floor(x_img));
        int xMax = static_cast<int>(std::ceil(x_img));
        int yMin = static_cast<int>(std::floor(y_img));
        int yMax = static_cast<int>(std::ceil(y_img));

        // 获取四个点的颜色
        // 纹理空间 (UV)      图像空间 (像素)
        // (0,1)---(1,1)      (0,0)---(width-1,0)
        //  |       |          |   image  |
        //  |       |          |          |
        // (0,0)---(1,0)      (0,height-1)---(width-1,height-1)
        auto colorUL = image_data.at<cv::Vec3b>(yMin, xMin); // 左上
        auto colorUR = image_data.at<cv::Vec3b>(yMin, xMax); // 右上
        auto colorDL = image_data.at<cv::Vec3b>(yMax, xMin); // 左下
        auto colorDR = image_data.at<cv::Vec3b>(yMax, xMax); // 右下

        // 计算插值权重
        float uLerp = (x_img - xMin) / (xMax - xMin);
        float vLerp = (y_img - yMin) / (yMax - yMin);

        // 水平插值（U方向）
        cv::Vec3f colorUp   = colorUL + uLerp * (colorUR - colorUL);
        cv::Vec3f colorDown = colorDL + uLerp * (colorDR - colorDL);

        // 垂直插值（V方向）
        cv::Vec3f color = colorDown + vLerp * (colorUp - colorDown);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
