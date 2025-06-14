//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 为了实现更小分块，把 int 改成了 float
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    /**
     * Lecture 05 Resterization 1 (Triangles)
     * 00:49:53
     * 叉乘的结果的z分量的正负号可以判断点是否在三角形内
     */
    Vector3f v0v1 = _v[1] - _v[0];
    Vector3f v1v2 = _v[2] - _v[1];
    Vector3f v2v0 = _v[0] - _v[2];
    Vector3f v0p  = Eigen::Vector3f(x, y, _v[0].z()) - _v[0];
    Vector3f v1p  = Eigen::Vector3f(x, y, _v[0].z()) - _v[1];
    Vector3f v2p  = Eigen::Vector3f(x, y, _v[0].z()) - _v[2];
    Vector3f c1   = v0v1.cross(v0p);
    Vector3f c2   = v1v2.cross(v1p);
    Vector3f c3   = v2v0.cross(v2p);
    return (c1.z() > 0 && c2.z() > 0 && c3.z() > 0) || (c1.z() < 0 && c2.z() < 0 && c3.z() < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle        t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        // Homogeneous division
        for (auto& vec : v)
        {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    /**
     * Lecture 05 Resterization 1 (Triangles)
     * 00:47:32
     * 遍历三角形的包围盒，左上取小，右下取大
     */
    int min_x = static_cast<int>(std::floor(std::min({t.v[0].x(), t.v[1].x(), t.v[2].x()})));
    int max_x = static_cast<int>(std::ceil(std::max({t.v[0].x(), t.v[1].x(), t.v[2].x()})));
    int min_y = static_cast<int>(std::floor(std::min({t.v[0].y(), t.v[1].y(), t.v[2].y()})));
    int max_y = static_cast<int>(std::ceil(std::max({t.v[0].y(), t.v[1].y(), t.v[2].y()})));

    for (int x = min_x; x <= max_x; ++x)
    {
        for (int y = min_y; y <= max_y; ++y)
        {
            if (msaa == false)
            {
                if (insideTriangle(x + 0.5f, y + 0.5f, t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal        = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated      = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int ind = get_index(x, y);
                    if (depth_buf[ind] > z_interpolated)
                    {
                        Vector3f color = t.getColor();
                        depth_buf[ind] = z_interpolated;
                        set_pixel(Eigen::Vector3f(x, y, 0), color);
                    }
                }
            }
            else
            {
                // 2 x 2 pixel sampling
                int   block = 2;
                int   count = 0;
                float min_z = std::numeric_limits<float>::max();
                for (int i = 0; i < block; ++i)
                {
                    for (int j = 0; j < block; ++j)
                    {
                        float sample_x = x + (i + 0.5f) / block;
                        float sample_y = y + (j + 0.5f) / block;
                        if (insideTriangle(sample_x, sample_y, t.v))
                        {
                            // 计算重心坐标
                            auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);

                            // 透视校正插值
                            float w_reciprocal   = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            // 更新最小深度
                            if (z_interpolated < min_z)
                            {
                                min_z = z_interpolated;
                            }

                            count++;
                        }
                    }
                }
                if (count > 0)
                {
                    int ind = get_index(x, y);
                    if (depth_buf[ind] > min_z)
                    {
                        // 计算平均颜色，加白变浅
                        Vector3f white         = Vector3f(255.0f, 255.0f, 255.0f);
                        float    average_color = count / (float)(block * block);
                        Vector3f color         = white * (1.0f - average_color) + t.getColor() * average_color;
                        depth_buf[ind] = min_z;
                        set_pixel(Eigen::Vector3f(x, y, 0), color);
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h)
    : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind       = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}