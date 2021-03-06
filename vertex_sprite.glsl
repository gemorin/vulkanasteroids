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
layout (location = 2) out int instanceIndex;

layout(binding = 0) uniform Proj {
    mat4 proj;
} u;
layout(push_constant) uniform PushConsts {
	mat4 transform[MAX_SPRITES_PER_DRAW];
} pushConsts;

void main() {
    mat4 m = u.proj * pushConsts.transform[gl_InstanceIndex];
    gl_Position = m * vec4(position, 1.0);

    uv = inUv;
    color = inColor;
    instanceIndex = gl_InstanceIndex;
}
