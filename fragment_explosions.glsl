#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler2D s;

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;

layout(location = 0) out vec4 outColor;
layout(push_constant) uniform PushConsts {
	layout (offset = 64) int idx;
} pushConsts;

void main() {
    int i = clamp(pushConsts.idx, 0, 64);
    int row = i / 8;
    int col = i % 8;
    vec2 realUv;
    realUv.x = uv.x / 8.0 + col * 0.125;
    realUv.y = uv.y / 8.0 + row * 0.125;
    outColor = texture(s, realUv);
    outColor.a = 0.01;//(64 - i) / 64;
    //outColor = vec4(color, 1.0);
}
