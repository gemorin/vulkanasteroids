#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler s;
// XXX define this as well
layout(binding = 2) uniform texture2D shipTextures[6];

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;
layout(location = 2) flat in int instanceIndex;

layout(location = 0) out vec4 outColor;
layout(push_constant) uniform PushConsts {
	layout (offset = (NUM_MAX_ASTEROIDS * 64)) int idx[NUM_MAX_ASTEROIDS];
} pushConsts;

void main() {
    outColor = texture(sampler2D(shipTextures[pushConsts.idx[instanceIndex]], s), uv);
    //outColor.a = 1.0;
    //outColor = vec4(color, 1.0);
}
