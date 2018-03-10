#version 450
#extension GL_ARB_separate_shader_objects : enable

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

out gl_PerVertex {
    vec4 gl_Position;
};
layout (location = 0) out vec3 fragColor;

layout(binding = 0) uniform UniformMvp {
    mat4 model;
    mat4 view;
    mat4 proj;
} mvp;

layout(binding = 1) uniform UniformCubeTransforms {
    mat4 vTransform[27];
} cubeTransforms;


void main() {
    mat4 m = mvp.proj * mvp.view * mvp.model;
    gl_Position = m * vec4(position, 1.0);

    fragColor = color;
}
