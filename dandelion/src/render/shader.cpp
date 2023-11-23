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
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;

    // Vertex position transformation
    // 顶点变换
    output_payload.position = Uniforms::MVP * output_payload.position;
    output_payload.position[0] = output_payload.position[0] / output_payload.position.w();
    output_payload.position[1] = output_payload.position[1] / output_payload.position.w();
    output_payload.position[2] = output_payload.position[2] / output_payload.position.w();

    // Viewport transformation
    // 视口变换
    output_payload.position[0] = Uniforms::width * (output_payload.position[0] + 1.0f) / 2.0f;
    output_payload.position[1] = Uniforms::height * (output_payload.position[1] + 1.0f) / 2.0f;

    // Vertex normal transformation
    // 法向量变换
    Vector4f nm;
    nm.head<3>() = payload.normal;
    nm[3] = 0.0f;
    nm = Uniforms::inv_trans_M * nm;
    output_payload.normal = nm.head<3>().normalized();

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    
    Vector3f result = {0, 0, 0};
    for(auto& light:lights){
        // Light Direction
        Vector3f light_dir = light.position - payload.world_pos;
        light_dir.normalize();
        // View Direction
        Vector3f view_dir = camera.position - payload.world_pos;
        view_dir.normalize();
        // Half Vector
        Vector3f half_dir = light_dir + view_dir;
        half_dir.normalize();
        
        // Light Attenuation 衰减
        float dec = (light.position - payload.world_pos).norm();
        dec = dec * dec;
        // Ambient 环境光
        Vector3f ambient = material.ambient;

        Vector3f normal = payload.world_normal.normalized();
        // Diffuse 漫反射光
        Vector3f diffuse = material.diffuse * std::max(0.0f, normal.dot(light_dir));
        // Specular 镜面反射
        Vector3f specular = material.specular * std::pow(std::max(0.0f, normal.dot(half_dir)), material.shininess);
        result += light.intensity * (diffuse + specular) / dec + ambient;
    }
    // set ambient light intensity
        
    // set rendering result max threshold to 255
    for(int i=0;i<3;i++){
        if(result[i] > 1.0)
            result[i] = 1.0;
    }
    return result * 255.f;
}
