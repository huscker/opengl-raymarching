#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;

out vec2 pix_coord;

void main(){
	gl_Position =  vec4(vertexPosition_modelspace,1);
	pix_coord = (vertexPosition_modelspace - vec3(0,0,0)).xy;
}
