#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

#define M_PI         (3.14159265358979323846) /* PI */
#define RAD          (57.3248407643312)       /* 1 rad = 57.32° */
#define DEG2RAD(val) (val / RAD)              /* 角度转弧度 */
#define RAD2DEG(val) (val * RAD)              /* 弧度转角度 */

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    /**
     * Lecture 04 Transformation
     * 00:12:30
     */
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f zRotate;
    float           rad = DEG2RAD(rotation_angle);
    zRotate << cos(rad), -sin(rad), 0, 0,
        sin(rad), cos(rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = zRotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    /**
     * Lecture 05 Resterization
     * 00:11:10
     * 正交投影矩阵，计算 t, r
     */
    float n = zNear;
    float f = zFar;
    float t = tan(DEG2RAD(eye_fov / 2)) * fabs(zNear);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    /**
     * Lecture 04 Transformation
     * 00:49:00
     * 平移矩阵 Mortho = translate * scale
     * 01:17:34
     * 正交投影矩阵 MperspToOrtho
     * Mpersp = Mortho * MperspToOrtho
     */
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(l + r) / 2,
        0, 1, 0, -(b + t) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f scale;
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    Eigen::Matrix4f perspToOrtho;
    perspToOrtho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    projection = translate * scale * perspToOrtho;

    // 为了使得三角形是正着显示的，这里需要把透视矩阵乘以下面这样的矩阵
    // 参考：http://games-cn.org/forums/topic/%e4%bd%9c%e4%b8%9a%e4%b8%89%e7%9a%84%e7%89%9b%e5%80%92%e8%bf%87%e6%9d%a5%e4%ba%86/
    Eigen::Matrix4f Mt;
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    projection = projection * Mt;

    return projection;
}

int main(int argc, const char** argv)
{
    float       angle        = 0;
    bool        command_line = false;
    std::string filename     = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename     = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{
        {2, 0, -2},
        {0, 2, -2},
        {-2, 0, -2},
        {3.5, -1, -5},
        {2.5, 1.5, -5},
        {-1, 0.5, -5}};

    std::vector<Eigen::Vector3i> ind{
        {0, 1, 2},
        {3, 4, 5}};

    std::vector<Eigen::Vector3f> cols{
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key         = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        // m 键切换 msaa 模式
        if (key == 'm')
        {
            r.msaa = !r.msaa;
            std::cout << "msaa mode: " << (r.msaa ? "ON" : "OFF") << '\n';
        }

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}