#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform sampler2D font;

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;

layout(location = 0) out vec4 outColor;

void main() {
    float c = texture(font, uv).r;
    outColor = vec4(vec3(c), c);
    //outColor = vec4(color, 1.0f);
}
