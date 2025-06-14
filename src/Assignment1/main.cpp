#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

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

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
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
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

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

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    /**
     * Lecture 04 Transformation
     * 00:16:20
     * 罗德里格旋转公式（Rodrigues' rotation formula）
     */
    float           rad    = DEG2RAD(angle);
    Vector3f        normal = axis.normalized();
    Eigen::Matrix3f N;
    N << 0, -normal.z(), normal.y(),
        normal.z(), 0, -normal.x(),
        -normal.y(), normal.x(), 0;

    Eigen::Matrix3f c1 = Eigen::Matrix3f::Identity() * cos(rad);
    Eigen::Matrix3f c2 = (1 - cos(rad)) * normal * normal.transpose();
    Eigen::Matrix3f c3 = sin(rad) * N;
    Eigen::Matrix3f R  = c1 + c2 + c3;

    Eigen::Matrix4f m_rotate   = Eigen::Matrix4f::Identity();
    m_rotate.block<3, 3>(0, 0) = R;

    return m_rotate;
}

int main(int argc, const char** argv)
{
    float       angle        = 0;
    bool        command_line = false;
    std::string filename     = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle        = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key         = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(1, 1, 0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
