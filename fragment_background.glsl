#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler2D backgroundSampler;

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;

layout(location = 0) out vec4 outColor;

void main() {
    if (uv.x == 2.0) {
        outColor = vec4(color, 1.0f);
    }
    else {
        outColor = texture(backgroundSampler, uv);
    }
}
