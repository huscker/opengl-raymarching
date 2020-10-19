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
#version 330 core

in vec2 pix_coord;

out vec3 color;

uniform float time;
uniform vec3 pos;
uniform float cam_hor_angle;
uniform float cam_ver_angle;

const int MAX_ITERS = 300;
const float MIN_DIST = 0.0;
const float MAX_DIST = 1000.0;
const float EPSILON = 0.00001;

mat4 rotationMatrix(vec3 axis, float angle)
{
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat4(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,
                0.0,                                0.0,                                0.0,                                1.0);
}
vec3 opRep(vec3 p, vec3 c)
{
    vec3 q = mod(p+0.5*c,c)-0.5*c;
    return q;
}
float sphereSDF(vec3 p,float r){
    return length(p)- r;
}
float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}
float unionSDF(float distA, float distB) {
    return min(distA, distB);
}
float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}
float boxSDF( vec3 p, vec3 b )
{
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}
float sceneSDF(vec3 p){
    p = opRep(p,vec3(3,3,3));
    float t1 = 1.;
    float t2 = 0.3;
    float c = unionSDF(boxSDF(p,vec3(t1,t2,t2)),boxSDF(p,vec3(t2,t1,t2)));
    c = unionSDF(c,boxSDF(p,vec3(t2,t2,t1)));
    return c;
}

vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

float rayMarcher(vec3 eye,vec3 dir,float start,float end){
    float depth = start;
    for(int i =0;i<MAX_ITERS;i++){
        float dist = sceneSDF(eye + depth * dir);
        if (dist < EPSILON){
            return depth;
        }
        depth += dist;
        if(depth>= end){
            return end;
        }
    }
    return end;
}

vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,
                          vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));
    
    float dotLN = dot(L, N);
    float dotRV = dot(R, V);
    
    if (dotLN < 0.0) {
        return vec3(0.0, 0.0, 0.0);
    } 
    
    if (dotRV < 0.0) {
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,vec3 lightp) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    
    /*vec3 light1Pos = vec3(4.0 * sin(time),
                          0,
                          4.0 * cos(time));*/
        
    vec3 light1Intensity = vec3(1,1,1)*1 / sqrt(length(p-lightp));
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  lightp,
                                  light1Intensity);
       
    return color;
}

vec3 rayDirection(float FOV,vec2 d0){
    d0.y *= (768.0/1024.0);
    float z = 1/tan(FOV/2);
    return normalize(vec3(d0,-z));
}

void main(){
    vec3 dir = rayDirection(3.14/1.5,pix_coord);
    //dir = (rotationMatrix(vec3(1,0,0),cam_ver_angle) * rotationMatrix(vec3(0,1,0),-cam_hor_angle) * vec4(dir,0)).xyz;
    dir = (rotationMatrix(vec3(0,1,0),-cam_hor_angle) * rotationMatrix(vec3(1,0,0),-cam_ver_angle) * vec4(dir,0)).xyz;
    //dir = (rotationMatrix(vec3(1,0,0),-cam_ver_angle) * vec4(dir,0)).xyz
    vec3 eye = vec3(-pos.x,pos.y,-pos.z);
    float dist = rayMarcher(eye,dir,MIN_DIST,MAX_DIST);
    if(dist > MAX_DIST-EPSILON){
        color = vec3(0,0,0);
        return;
    }

    vec3 p = eye + dist * dir;

    vec3 ambientColor = vec3(0.2,0.2,0.2);
    vec3 diffuseColor = vec3(0,0.7,0.7);
    vec3 specularColor = vec3(1,1,1);
    float shininess = 10.0;

    color = phongIllumination(ambientColor,diffuseColor,specularColor,shininess,p,eye,eye);
}