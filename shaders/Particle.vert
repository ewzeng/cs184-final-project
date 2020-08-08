#version 330 core
uniform mat4 u_view_projection;

layout (location = 0) in vec3 aPos;

void main()
{
    gl_Position = u_view_projection * vec4(aPos, 1.0);
}
