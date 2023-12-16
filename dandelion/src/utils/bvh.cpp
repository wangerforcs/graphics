#include "bvh.h"

#include <cassert>
#include <iostream>
#include <optional>

#include <Eigen/Geometry>
#include "formatter.hpp"
#include <spdlog/spdlog.h>

#include "math.hpp"

using Eigen::Vector3f;
using std::optional;
using std::vector;

BVHNode::BVHNode() : left(nullptr), right(nullptr), face_idx(0)
{
}

BVH::BVH(const GL::Mesh& mesh) : root(nullptr), mesh(mesh)
{
}

// 建立bvh，将需要建立BVH的图元索引初始化
void BVH::build()
{
    if (mesh.faces.count() == 0) {
        root = nullptr;
        return;
    }

    primitives.resize(mesh.faces.count());
    for (size_t i = 0; i < mesh.faces.count(); i++) primitives[i] = i;

    root = recursively_build(primitives);
    return;
}
// 删除bvh
void BVH::recursively_delete(BVHNode* node)
{
    if (node == nullptr)
        return;
    recursively_delete(node->left);
    recursively_delete(node->right);
    delete node;
    node = nullptr;
}
// 统计BVH树建立的节点个数
size_t BVH::count_nodes(BVHNode* node)
{
    if (node == nullptr)
        return 0;
    else
        return count_nodes(node->left) + count_nodes(node->right) + 1;
}
// 递归建立BVH
BVHNode* BVH::recursively_build(vector<size_t> faces_idx)
{
    BVHNode* node = new BVHNode();
    
    // if faces_idx.size()==1: return node;
    // if faces_idx.size()==2: recursively_build() & union_AABB(node->left->aabb,
    // node->right->aabb); else:
    // choose the longest dimension among x,y,z
    // devide the primitives into two along the longest dimension
    // recursively_build() & union_AABB(node->left->aabb, node->right->aabb)
    if (faces_idx.size() == 1) {
        node->face_idx = faces_idx[0];
        node->aabb = get_aabb(mesh, faces_idx[0]);
    }else if (faces_idx.size() == 2) {
        node->left = recursively_build(vector<size_t>(faces_idx.begin(), faces_idx.begin() + 1));
        node->right = recursively_build(vector<size_t>(faces_idx.begin() + 1, faces_idx.end()));
        node->aabb = union_AABB(node->left->aabb, node->right->aabb);
    }else {
        AABB aabb;
        for (size_t i = 0; i < faces_idx.size(); i++) {
            aabb = union_AABB(aabb, get_aabb(mesh, faces_idx[i]));
        }
        int max_dim = aabb.max_extent();
        std::sort(faces_idx.begin(), faces_idx.end(),
                  [this, max_dim](size_t a, size_t b) {
                      return get_aabb(mesh, a).centroid()[max_dim] <
                             get_aabb(mesh, b).centroid()[max_dim];
                  });
        node->left = recursively_build(vector<size_t>(faces_idx.begin(), faces_idx.begin() + faces_idx.size() / 2));
        node->right = recursively_build(vector<size_t>(faces_idx.begin() + faces_idx.size() / 2, faces_idx.end()));
        node->aabb = union_AABB(node->left->aabb, node->right->aabb);
    }
    return node;
}
// 使用BVH求交
optional<Intersection> BVH::intersect(const Ray& ray, [[maybe_unused]] const GL::Mesh& mesh,
                                      const Eigen::Matrix4f obj_model)
{
    model = obj_model;
    optional<Intersection> isect;
    if (!root) {
        isect = std::nullopt;
        return isect;
    }
    Ray modelray;
    modelray.origin = (model.inverse() * ray.origin.homogeneous()).head<3>();
    Eigen::Vector4f dir = ray.direction.homogeneous();
    dir[3] = 0;
    modelray.direction = (model.inverse() * dir).head<3>();
    isect = ray_node_intersect(root, modelray);
    return isect;
}
// 发射的射线与当前节点求交，并递归获取最终的求交结果
optional<Intersection> BVH::ray_node_intersect(BVHNode* node, const Ray& ray) const
{
    optional<Intersection> isect = std::nullopt;
    if(node == nullptr)
        return std::nullopt;
    // The node intersection is performed in the model coordinate system.
    // Therefore, the ray needs to be transformed into the model coordinate system.
    // The intersection attributes returned are all in the model coordinate system.
    // Therefore, They are need to be converted to the world coordinate system.    
    // If the model shrinks, the value of t will also change.
    // The change of t can be solved by intersection point changing simultaneously

    Vector3f inv;
    if(std::abs(ray.direction.x()) < std::numeric_limits<float>::epsilon())
        inv.x() = 0.0f;
    else
        inv.x() = 1.0f / ray.direction.x();
    if(std::abs(ray.direction.y()) < std::numeric_limits<float>::epsilon())
        inv.y() = 0.0f;
    else
        inv.y() = 1.0f / ray.direction.y();
    if(std::abs(ray.direction.z()) < std::numeric_limits<float>::epsilon())
        inv.z() = 0.0f;
    else
        inv.z() = 1.0f / ray.direction.z();

    if(node->aabb.intersect(ray, inv, {ray.direction.x()<0, ray.direction.y()<0, ray.direction.z()<0})){
        if(node->face_idx > 0){
            isect = ray_triangle_intersect(ray, mesh, node->face_idx);
            if(isect){
                Eigen::Vector4f normal = Eigen::Vector4f(isect->normal.x(), isect->normal.y(), isect->normal.z(), 0);
                isect->normal = (model.inverse().transpose() * normal).head<3>().normalized();
            }
        }else{
            optional<Intersection> isect_left = ray_node_intersect(node->left, ray);
            optional<Intersection> isect_right = ray_node_intersect(node->right, ray);
            if(isect_left && isect_right){
                if(isect_left->t < isect_right->t){
                    isect = isect_left;
                }else{
                    isect = isect_right;
                }
            }else if(isect_left){
                isect = isect_left;
            }else if(isect_right){
                isect = isect_right;
            }else{
                isect = std::nullopt;
            }
        }
    }
            
    return isect;
}
