#include <array>
#include <limits>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "render_engine.h"
#include "../utils/math.hpp"

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;

// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    // Vector3f v[3];
    // for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};

    // Vector3f p(float(x), float(y), 1.0f);
    auto [alpha, beta, gamma] = compute_barycentric_2d(x, y, vertices);
    if(alpha < 0 || beta < 0 || gamma < 0)
        return false;
    else
        return true;
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;
    
    c1 = (-(x-v[1].x())*(v[2].y()-v[1].y())+(y-v[1].y())*(v[2].x()-v[1].x()))/
        (-(v[0].x()-v[1].x())*(v[2].y()-v[1].y())+(v[0].y()-v[1].y())*(v[2].x()-v[1].x()));
    c2 = (-(x-v[2].x())*(v[0].y()-v[2].y())+(y-v[2].y())*(v[0].x()-v[2].x()))/
        (-(v[1].x()-v[2].x())*(v[0].y()-v[2].y())+(v[1].y()-v[2].y())*(v[0].x()-v[2].x()));
    c3 = 1 - c1 - c2;
    return {c1, c2, c3};
}

// 对当前渲染物体的所有三角形面片进行遍历，进行几何变换以及光栅化
void Rasterizer::draw(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                      const std::list<Light>& lights, const Camera& camera)
{
    // iterate over all triangles in TriangleList
    for (const auto& t : TriangleList) {
        Triangle newtriangle;
        std::array<Vector3f, 3> worldspace_pos;

        VertexShaderPayload vertex[3];
        for (int i = 0; i < 3; i++) {
            vertex[i].position = t.vertex[i];
            vertex[i].normal = t.normal[i];

            // transform vertex position to world space for interpolating
            // 记录世界坐标顶点位置
            worldspace_pos[i] = (model * t.vertex[i]).head<3>(); 

            // Use vetex_shader to transform vertex attributes(position & normals) to
            // view port and set a new triangle
            vertex[i] = vertex_shader(vertex[i]);
            newtriangle.vertex[i] = vertex[i].position;
            newtriangle.normal[i] = vertex[i].normal;
        }

        // call rasterize_triangle()
        rasterize_triangle(newtriangle, worldspace_pos, material, lights, camera);
    }
}

// 对顶点的某一属性插值
Vector3f Rasterizer::interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
                                 const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3,
                                 const Eigen::Vector3f& weight, const float& Z)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1] +
                              gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}

// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(const Triangle& t, const std::array<Vector3f, 3>& world_pos,
                                    GL::Material material, const std::list<Light>& lights,
                                    Camera camera)
{


    // discard all pixels out of the range(including x,y,z)
    float minx = std::min(std::min(t.vertex[0].x(), t.vertex[1].x()), t.vertex[2].x());
    float miny = std::min(std::min(t.vertex[0].y(), t.vertex[1].y()), t.vertex[2].y());
    float maxx = std::max(std::max(t.vertex[0].x(), t.vertex[1].x()), t.vertex[2].x());
    float maxy = std::max(std::max(t.vertex[0].y(), t.vertex[1].y()), t.vertex[2].y());
    
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. fragment shading(use function:fragment_shader())
    // 4. set pixel
    Vector3f weight = {t.vertex[0].w(), t.vertex[1].w(), t.vertex[2].w()};
    for(int x = minx; x <= maxx; x++)
    {
        for(int y = miny; y <= maxy; y++)
        {
            if(inside_triangle(x, y, t.vertex))
            {
                auto [alpha, beta, gamma] = compute_barycentric_2d(x, y, t.vertex);
                float zt = 1 / (alpha / t.vertex[0].w() + beta / t.vertex[1].w() + gamma / t.vertex[2].w());
                float z = (alpha * t.vertex[0].z() / t.vertex[0].w()) + (beta * t.vertex[1].z() / t.vertex[1].w()) + (gamma * t.vertex[2].z() / t.vertex[2].w());
                z = z * zt;

                int index = get_index(x, y);
                if(z < depth_buf[index])
                {
                    depth_buf[index] = z;

                    Vector3f interpolated_pos = interpolate(alpha, beta, gamma, world_pos[0], world_pos[1], world_pos[2], weight, zt);
                    Vector3f interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], weight, zt);

                    FragmentShaderPayload fragshader_payload(interpolated_pos, interpolated_normal);
                    Vector3f color = fragment_shader(fragshader_payload, material, lights, camera);
                    set_pixel({x, y}, color);
                }
            }
        }
    }
}

// 初始化整个光栅化渲染器
void Rasterizer::clear(BufferType buff)
{
    if ((buff & BufferType::Color) == BufferType::Color) {
        fill(frame_buf.begin(), frame_buf.end(), RenderEngine::background_color * 255.0f);
    }
    if ((buff & BufferType::Depth) == BufferType::Depth) {
        fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

Rasterizer::Rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

// 给定像素坐标(x,y)，计算frame buffer里对应的index
int Rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

// 给定像素点以及fragement shader得到的结果，对frame buffer中对应存储位置进行赋值
void Rasterizer::set_pixel(const Vector2i& point, const Vector3f& res)
{
    int idx        = get_index(point.x(), point.y());
    frame_buf[idx] = res;
}
