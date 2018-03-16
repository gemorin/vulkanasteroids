#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler s;
layout(binding = 2) uniform texture2D shipTextures[2];

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;

layout(location = 0) out vec4 outColor;
layout(push_constant) uniform PushConsts {
	mat4 transform;
	int idx;
} pushConsts;

void main() {
    outColor = texture(sampler2D(shipTextures[pushConsts.idx], s), uv);
}
