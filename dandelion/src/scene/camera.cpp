#include "camera.h"

#include <cmath>
#include <Eigen/Geometry>
#include "../utils/formatter.hpp"
#include <spdlog/spdlog.h>

#include "../utils/math.hpp"

#ifdef _WIN32
#undef near
#undef far
#endif

using Eigen::Affine3f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

Camera::Camera(const Eigen::Vector3f& position, const Eigen::Vector3f& target, float near_plane,
               float far_plane, float fov_y_degrees, float aspect_ratio)
    : position(position), target(target), near(near_plane), far(far_plane),
      fov_y_degrees(fov_y_degrees), aspect_ratio(aspect_ratio)
{
    world_up.x() = 0.0f;
    world_up.y() = 1.0f;
    world_up.z() = 0.0f;
}

Matrix4f Camera::view()
{
    // Compute the inverted view direction, up and right vectors related to
    // the camera itself.
    Vector3f inv_direction = (position - target).normalized();
    Vector3f right         = (world_up).cross(inv_direction).normalized();
    Vector3f up            = inv_direction.cross(right);
    // The view matrix is multiplication of a rotation and a translation
    // matrices: rotation represented by [right, up, inv_direction]^T and
    // translation represented by -position.
    Matrix4f view_matrix          = Matrix4f::Identity();
    view_matrix.block<1, 3>(0, 0) = right;
    view_matrix.block<1, 3>(1, 0) = up;
    view_matrix.block<1, 3>(2, 0) = inv_direction;
    view_matrix(0, 3)             = -right.dot(position);
    view_matrix(1, 3)             = -up.dot(position);
    view_matrix(2, 3)             = -inv_direction.dot(position);
    return view_matrix;
}

// Matrix4f Camera::projection()
// {
//     const float fov_y = radians(fov_y_degrees);
//     const float top   = (target - position).norm() * std::tan(fov_y / 2.0f);
//     const float right = top * aspect_ratio;

//     Matrix4f projection = Matrix4f::Zero();
//     // 使用平行投影时，用户并不能从画面上直观地感受到相机的位置，
//     // 因而会产生近处物体裁剪过多的错觉。为了产程更好的观察效果，
//     // 这里没有使用相机本身的 near 而是取 near = -far 来让相机能看到“背后”的物体。
//     projection(0, 0) = 1.0f / right;
//     projection(1, 1) = 1.0f / top;
//     projection(2, 2) = -1.0f / far;
//     projection(2, 3) = 0.0f;
//     projection(3, 3) = 1.0f;

//     return projection;
// }

Matrix4f Camera::projection()
{
    const float fov_y = radians(fov_y_degrees);
    // const float top   = (target - position).norm() * std::tan(fov_y / 2.0f);
    // const float right = top * aspect_ratio;
    const float top   = near * std::tan(fov_y / 2.0f);
    const float right = top * aspect_ratio;

    //scaling to norm
    Matrix4f mat_scale = Matrix4f::Zero();
    // 使用平行投影时，用户并不能从画面上直观地感受到相机的位置，
    // 因而会产生近处物体裁剪过多的错觉。为了产程更好的观察效果，
    // 这里没有使用相机本身的 near 而是取 near = -far 来让相机能看到“背后”的物体。
    mat_scale(0, 0) = 1.0f / right;
    mat_scale(1, 1) = 1.0f / top;
    /*frankly, why minus? it is just right this way*/
    mat_scale(2, 2) = -2.0f / (far-near);
    mat_scale(3, 3) = 1.0f;

    Matrix4f mat_trans = Matrix4f::Identity(); 
    mat_trans(2,3) = (near+far)/2;

    /*
    a little different from method on internet and class ppt beacuse some plus-minus
    from internet
    n   0   0   0
    0   n   0   0
    0   0   n+f -n*f
    0   0   1   0
    */
    Matrix4f mat_press = Matrix4f::Zero();
    mat_press(0,0) = near;
    mat_press(1,1) = near;
    mat_press(2,2) = (near+far);
    mat_press(2,3) = far*near;
    mat_press(3,2) = -1.0f;

    // final projection matrix
    Matrix4f mat_final=mat_scale*mat_trans*mat_press;

    // directly from class ppt
    Matrix4f mat_direct= Matrix4f::Zero();
    mat_direct(0,0) = near/right;
    mat_direct(1,1) = near/top;
    mat_direct(2,2) = -(near+far)/(far-near);
    mat_direct(2,3) = -2.0f*far*near/(far-near);
    mat_direct(3,2) = -1.0f;
    // return mat_direct;
    return mat_final;
}
