#version 450
#extension GL_ARB_separate_shader_objects : enable

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUv;

out gl_PerVertex {
    vec4 gl_Position;
};
layout (location = 0) out vec3 color;
layout (location = 1) out vec2 uv;

void main() {
    gl_Position = vec4(position, 1.0);

    uv = inUv;
    color = inColor;
}
