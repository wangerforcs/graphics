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
    Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};
    //用于存储p-顶点向量值
    Vector3f pv[3];
    Vector3f p(float(x), float(y), 1.0f);
    //a,b,c存储向量的叉积值
    float a, b, c;
    for (int i = 0; i < 3; i++) {
        pv[i] = p - v[i];
    }
    a = pv[0].x() * pv[1].y() - pv[0].y() * pv[1].x();
    b = pv[1].x() * pv[2].y() - pv[1].y() * pv[2].x();
    c = pv[2].x() * pv[0].y() - pv[2].y() * pv[0].x();
    //三个叉积同号则说明点在三角形中
    return ((a>0&&b>0&&c>0) || (a<0&&b<0&&c<0));
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1, c2, c3;
    Eigen::Vector2f p[3];
    Eigen::Vector2f final;//矩阵运算结果
    Eigen::Vector2f qq;//中间向量
    Eigen::Matrix2f Q;//中间矩阵
    //记录顶点坐标
    for (int i = 0; i < 3; i++) {
        p[i] = {v[i].x(), v[i].y()};
    }
    Q(0, 0) = p[1].x() - p[0].x();
    Q(0, 1) = p[2].x() - p[0].x();
    Q(1, 0) = p[1].y() - p[0].y();
    Q(1, 1) = p[2].y() - p[0].y();
    qq.x()  = x - p[0].x();
    qq.y()  = y - p[0].y();
    //结果提取
    final   = Q.inverse() * qq;
    c2      = final.x();
    c3      = final.y();
    c1      = 1.0f - c2 - c3;
    return {c1, c2, c3};
}

// 对当前渲染物体的所有三角形面片进行遍历，进行几何变换以及光栅化
void Rasterizer::draw(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                      const std::list<Light>& lights, const Camera& camera)
{

    // iterate over all triangles in TriangleList
    for (const auto& t : TriangleList) {
        Triangle newtriangle;
        // transform vertex position to world space for interpolating
        std::array<Vector3f, 3> worldspace_pos;
        Vector4f xyzworld;
        for (int i = 0; i < 3; i++) {
            xyzworld = model * t.vertex[i];
            worldspace_pos[i].x() = xyzworld.x();
            worldspace_pos[i].y() = xyzworld.y();
            worldspace_pos[i].z() = xyzworld.z();
        }
        
        // Use vetex_shader to transform vertex attributes(position & normals) to
        // view port and set a new triangle
        VertexShaderPayload temp;
        for (int i=0;i<3;i++) {
            temp.position = t.vertex[i];
            temp.normal   = t.normal[i];
            temp          = vertex_shader(temp);
            newtriangle.normal[i] = temp.normal;
            newtriangle.vertex[i] = temp.position;
        }
        rasterize_triangle(newtriangle, worldspace_pos, material, lights, camera);

        // call rasterize_triangle()
        
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
    Vector4f triangle_scale[3];
    //深度插值zt
    float zt;
    //储存重心坐标alpha，beta和gamma；
    tuple<float, float, float> barycentric_coordinate;
    Vector3f weight_list = {1,1,1};
    Vector3f pos_inp;
    Vector3f normal_inp;
    Vector3f res;
    Vector2i points;
    float alpha, beta, gamma;
    for (int i = 0; i < 3; i++) {
        triangle_scale[i]=t.vertex[i];
    }
    //确定三角形覆盖区域
    float x_min, y_min, x_max, y_max;
    //确定像素遍历范围；
    int x_l, x_r, y_t, y_b;
    x_min = x_max = t.vertex[0].x();
    y_min = y_max = t.vertex[0].y();
    for (int i = 1; i < 3; i++) {
        if (t.vertex[i].x() > x_max) {
            x_max = t.vertex[i].x();
        }
        if (t.vertex[i].x() < x_min) {
            x_min = t.vertex[i].x();
        }
        if (t.vertex[i].y() > y_max) {
            y_max = t.vertex[i].y();
        }
        if (t.vertex[i].y() < y_min) {
            y_min = t.vertex[i].y();
        }
    }
    x_l = std::max(static_cast<int>(x_min)-1,0);
    x_r = std::min(static_cast<int>(x_max) + 1,Uniforms::width);
    y_b = std::max(static_cast<int>(y_min) - 1,0);
    y_t = std::min(static_cast<int>(y_max) + 1,Uniforms::height);
    
    for (int i = x_l; i <= x_r; i++) {
        for (int j = y_b; j <= y_t; j++) {
            if (inside_triangle(i, j, triangle_scale)) {
                points.x()             = i;
                points.y()             = j;
                barycentric_coordinate = compute_barycentric_2d((float)i, (float)j, triangle_scale);
                alpha                  = std::get<0>(barycentric_coordinate);
                beta                   = std::get<1>(barycentric_coordinate);
                gamma                  = std::get<2>(barycentric_coordinate);

                //深度差值
                float wt = 1.0f / ((alpha * 1.0f / t.vertex[0].w()) +
                             (beta * 1.0f / t.vertex[1].w()) +
                             (gamma  * 1.0f / t.vertex[2].w()));
                zt = (alpha * t.vertex[0].z() / t.vertex[0].w()) +
                     (beta * t.vertex[1].z() / t.vertex[1].w()) +
                     (gamma * t.vertex[2].z() / t.vertex[2].w());
                zt *= wt;
                if (zt < depth_buf[get_index(i,j)])
                {
                    pos_inp           = interpolate(alpha, beta, gamma, world_pos[0],
                                                    world_pos[1], world_pos[2], weight_list, wt);
                    normal_inp = interpolate(alpha, beta, gamma, t.normal[0],
                                                    t.normal[1], t.normal[2], weight_list, wt);
                    FragmentShaderPayload payload(pos_inp, normal_inp);
                    res = phong_fragment_shader(payload, material, lights, camera);
                    set_pixel(points, res);
                    depth_buf[get_index(i, j)] = zt;

                }
                
                
                
                
            }
        }
    }
    // discard all pixels out of the range(including x,y,z)

    
    
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. fragment shading(use function:fragment_shader())
    // 4. set pixel
            
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
