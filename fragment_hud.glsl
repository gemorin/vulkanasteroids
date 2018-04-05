#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform sampler2D health;

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;

layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConsts {
	float textureBypass;
} pushConsts;

void main() {
    outColor = pushConsts.textureBypass * texture(health, uv);
    outColor += ((1.0 - pushConsts.textureBypass) * vec4(color, 1.0));
    //outColor = vec4(color, 1.0);
}
