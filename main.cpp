/*
MIT License

Copyright (c) 2020 huscker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "misc/shader.hpp"

GLFWwindow *window;
glm::vec3 position(0,0,2.0);
float horizontalAngle = 3.14f;
float verticalAngle = 0.0f;
float slider1 = 3.14f/2;
float slider2 = 1.0f;
float slider3 = 1.0f;
float slider4 = 1.0f;
float slider5 = 1.0f;
float slider6 = 1.0f;
float mouseSpeed = 0.0005f;
float speed = 0.3f;
float sprint_speed = 1.0f;
bool isSprinting = false;
int input_mode = 0;

#define WIDTH 1024
#define HEIGHT 768

void updateInputs(){
    static double lastTime = glfwGetTime();
	
	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);

	// Get mouse position
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
    if (input_mode == 0){
        horizontalAngle += mouseSpeed * float(WIDTH/2 - xpos );
        verticalAngle   += mouseSpeed * float( HEIGHT/2 - ypos );
        while(horizontalAngle > 6.28f){
            horizontalAngle -= 6.28f;
        }
        while (horizontalAngle < -6.28f)
        {
            horizontalAngle += 6.28f;
        }
        while(verticalAngle > 1.57f){
            verticalAngle = 1.57f;
        }
        while (verticalAngle < -1.57f)
        {
            verticalAngle =-1.57f;
        }
    }else if (input_mode == 1)
    {
        slider1 += mouseSpeed * float(WIDTH/2 - xpos );
        slider2   += mouseSpeed * float( HEIGHT/2 - ypos );
    }
    else if (input_mode == 2)
    {
        slider3 += mouseSpeed * float(WIDTH/2 - xpos );
        slider4   += mouseSpeed * float( HEIGHT/2 - ypos );
    }
    else if (input_mode == 3)
    {
        slider5 += mouseSpeed * float(WIDTH/2 - xpos );
        slider6   += mouseSpeed * float( HEIGHT/2 - ypos );
    }
    
    glfwSetCursorPos(window, WIDTH / 2, HEIGHT / 2);
	glm::vec3 dir(
        cos(verticalAngle)*sin(horizontalAngle),
        sin(verticalAngle),
        cos(verticalAngle)*cos(horizontalAngle)
    );
    glm::vec3 right = glm::vec3(
        sin(horizontalAngle-3.14f/2.0f),
        0,
        cos(horizontalAngle-3.14f/2.0f)
    );
    if(isSprinting){
        speed += sprint_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS){
		position += dir * deltaTime * speed;
	}
	// Move backward
	if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS){
		position -= dir * deltaTime * speed;
	}
	// Strafe right
	if (glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS){
		position += right * deltaTime * speed;
	}
	// Strafe left
	if (glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS){
		position -= right * deltaTime * speed;
	}
    if (glfwGetKey( window, GLFW_KEY_SPACE ) == GLFW_PRESS){
		position += glm::vec3(0,1,0) * deltaTime * speed;
	}
    if (glfwGetKey( window, GLFW_KEY_LEFT_CONTROL ) == GLFW_PRESS){
		position -= glm::vec3(0,1,0) * deltaTime * speed;
	}
    if(isSprinting){
        speed -= sprint_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_LEFT_SHIFT ) == GLFW_PRESS){
        isSprinting = true;
    }
    if (glfwGetKey( window, GLFW_KEY_LEFT_SHIFT ) == GLFW_RELEASE){
        isSprinting = false;
    }
    if (glfwGetKey(window,GLFW_KEY_Z) ==GLFW_PRESS){
        input_mode = 1;
    }
    
    if (glfwGetKey(window,GLFW_KEY_X) ==GLFW_PRESS){
        input_mode = 2;
    }
    
    if (glfwGetKey(window,GLFW_KEY_C) ==GLFW_PRESS){
        input_mode = 3;
    }
    if (glfwGetKey(window,GLFW_KEY_C) == GLFW_RELEASE){
        if (glfwGetKey(window,GLFW_KEY_X) == GLFW_RELEASE){
            if (glfwGetKey(window,GLFW_KEY_Z) == GLFW_RELEASE){
                input_mode = 0;
            }
        }
    }
    lastTime = currentTime;
    glfwWaitEvents();
}

int main(){
    if(!glfwInit()){
        printf("Failed to initialise GLFW\n");
        return -1;
    }
    glfwWindowHint(GLFW_SAMPLES,4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(WIDTH,HEIGHT,"Raymarching",NULL,NULL);
    if(window == NULL){
        printf("Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glewExperimental = true;
    if(glewInit() != GLEW_OK){
        printf("Failed to initialise GLEW\n");
        return -1;
    }
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GLFW_TRUE);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwPollEvents();
    glfwSetCursorPos(window, WIDTH / 2, HEIGHT / 2);
    glClearColor(1.f, 1.f, 1.f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
    
    static const GLfloat g_quad_vertex_buffer_data[] = { 
		-1.0f, -1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
		 1.0f,  1.0f, 0.0f,
	};
	GLuint quad_vertexbuffer;
	glGenBuffers(1, &quad_vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);
	GLuint quad_programID = LoadShaders( "vertex.glsl", "fragment.glsl" );
    GLuint timeID = glGetUniformLocation(quad_programID,"time");
    GLuint posID = glGetUniformLocation(quad_programID,"pos");
    GLuint cam_hor_angleID = glGetUniformLocation(quad_programID,"cam_hor_angle");
    GLuint cam_ver_angleID = glGetUniformLocation(quad_programID,"cam_ver_angle");
    GLuint slider1ID = glGetUniformLocation(quad_programID,"slider1");
    GLuint slider2ID = glGetUniformLocation(quad_programID,"slider2");
    GLuint slider3ID = glGetUniformLocation(quad_programID,"slider3");
    GLuint slider4ID = glGetUniformLocation(quad_programID,"slider4");
    GLuint slider5ID = glGetUniformLocation(quad_programID,"slider5");
    GLuint slider6ID = glGetUniformLocation(quad_programID,"slider6");
    do{
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glUseProgram(quad_programID);
        glActiveTexture(GL_TEXTURE0);
        glUniform1f(timeID,(float)(glfwGetTime())*1.0f);
        updateInputs();
        glUniform3fv(posID,1,&position[0]);
        glUniform1fv(cam_hor_angleID,1,&horizontalAngle);
        glUniform1fv(cam_ver_angleID,1,&verticalAngle);
        glUniform1fv(slider1ID,1,&slider1);
        glUniform1fv(slider2ID,1,&slider2);
        glUniform1fv(slider3ID,1,&slider3);
        glUniform1fv(slider4ID,1,&slider4);
        glUniform1fv(slider5ID,1,&slider5);
        glUniform1fv(slider6ID,1,&slider6);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER,quad_vertexbuffer);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,(void*)0);
        glDrawArrays(GL_TRIANGLES,0,6);
        glDisableVertexAttribArray(0);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }while(glfwGetKey(window,GLFW_KEY_ESCAPE)!=GLFW_PRESS && glfwWindowShouldClose(window)==0);
    glDeleteVertexArrays(1,&VertexArrayID);
    glDeleteBuffers(1,&quad_vertexbuffer);
    glDeleteProgram(quad_programID);
    glfwTerminate();
    return 0;
}