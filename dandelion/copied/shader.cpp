#include "shader.h"
#include "../utils/math.hpp"

#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader & fragement shader can visit
// all the static variables below from Uniforms structure
Eigen::Matrix4f Uniforms::MVP;
Eigen::Matrix4f Uniforms::inv_trans_M;
int Uniforms::width;
int Uniforms::height;

// vertex shader
/*
 * 首先是将顶点坐标变换到投影平面，再进行视口变换；
 * 其次是将法线向量变换到世界坐标系，但文档中写的是相机坐标系
 *
 * \param payload 输入时顶点和法线均为模型坐标系
 * 输出时顶点经过视口变换变换到了屏幕空间，法线向量则为世界坐标系
*/
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;
    Vector4f mvp_payload;
    //将顶点坐标变换到投影平面
    mvp_payload = Uniforms::MVP * payload.position;
    mvp_payload.x() = mvp_payload.x() / mvp_payload.w();
    mvp_payload.y() = mvp_payload.y() / mvp_payload.w();
    // Viewport transformation
    output_payload.position.x() = 0.5 * Uniforms::width * (mvp_payload.x() + 1.0f);
    output_payload.position.y() = 0.5 * Uniforms::height * (mvp_payload.y() + 1.0f);
    

    output_payload.position.z() = mvp_payload.z();
    // Vertex normal transformation
    //法向量乘以模型变换矩阵的逆的转置即可变换到世界坐标系
    Vector4f norm_t;
    norm_t.x()            = payload.normal.x();
    norm_t.y()            = payload.normal.y();
    norm_t.z()            = payload.normal.z();
    norm_t.w()            = 1.0f;
    norm_t                = Uniforms::inv_trans_M * norm_t;
    output_payload.normal.x() = norm_t.x();
    output_payload.normal.y() = norm_t.y();
    output_payload.normal.z() = norm_t.z();

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{  
    Vector3f result = {0, 0, 0};
    Vector3f La;
    Vector3f Ld;
    Vector3f Ls;
    //光照方向和观察方向，均设为单位向量，法向量也设为单位向量计算
    Vector3f light_vector;
    Vector3f view_vector;
    //光源到物体表面的距离,用于光强衰减；
    float d;
    float dmodi;
    //h用于半程向量计算高光
    Vector3f h;
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    for (auto& lit : lights) {
    //光向量，pos指向light
    light_vector = (lit.position-payload.world_pos).normalized();
        d            = (payload.world_pos - lit.position).norm();
    //视点向量，pos指向view
    view_vector = (camera.position-payload.world_pos).normalized();
    // set ambient light intensity
    // Ambient
    La = material.ambient;
    // Diffuse
    Ld = material.diffuse; 
    Ld *= std::max(0.0f, ((payload.world_normal).normalized()).dot(light_vector));
    // Half Vector
    h = (light_vector + view_vector).normalized();
    // Specular
    Ls = material.specular *std::pow(std::max(0.0f, ((payload.world_normal).normalized()).dot(h)), material.shininess);
    //计算距离衰减
    dmodi = lit.intensity / (0.4+0.6*d+0.8*d*d);
    result += dmodi*(Ls + Ld) + La;
    // set rendering result max threshold to 255
    if (result.x() > 1.0)
        result(0) = 1.0;
    if (result.y() > 1.0)
        result(1) = 1.0;
    if (result.z() > 1.0)
        result(2) = 1.0;
   
    }
    return result * 255.f;
}
