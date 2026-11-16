layout(std140) uniform Camera {
    mat4 view;
    mat4 projection;
    mat4 viewProj;
    vec3 cameraPos;
    float _pad0;
};

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;
layout(location = 2) in vec3 a_Color;

uniform mat4 u_Model;

out vec3 v_WorldPosition;
out vec3 v_WorldNormal;
out vec3 v_Color;

void main()
{
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);
    v_WorldPosition = worldPosition.xyz;
    v_WorldNormal = normalize(transpose(inverse(mat3(u_Model))) * a_Normal);
    v_Color = a_Color;
    gl_Position = projection * view * worldPosition;
}
