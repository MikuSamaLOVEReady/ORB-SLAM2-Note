#version 450

// vetex attributes
layout(location = 0) in vec2 inposition;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragColor;

void main(){
    gl_Position = vec4( inposition , 0.0 , 1.0 );
    fragColor = inColor;
}