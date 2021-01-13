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

uniform float slider1;
uniform float slider2;
uniform float slider3;
uniform float slider4;
uniform float slider5;
uniform float slider6;

const int MAX_ITERS = 150;
const float MIN_DIST = 0.001;
const float MAX_DIST = 100.0;
const float EPSILON = 0.0001;
const float MAX_SHADOW_DIST = 50.0;
const float MIN_SHADOW_DIST = 0.01;
const float SHADOW_EPSILON = 0.001;
const float MAX_AO_ITERS = 10;
const float AO_EPSILON = 0.01;
const float AO_COEFICIENT = 0.1;
const float MANDELBULB_ITERS = 100;
const float MANDELBULB_POWER = 8;
const vec3 global_light_pos = vec3(1000,1000,-500);

int rayMarcherIters = -1;

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
vec3 opPlaneFold(vec3 p, vec3 n, float d) {
    return p - 2.0 * min(0.0, dot(p.xyz, n) - d) * n;
}
vec3 boxFold(vec3 p, vec3 r) {
    return clamp(p, -r, r) * 2.0 - p;
}
vec3 sierpinskiFold(inout vec3 p) {
	p.xy -= min(p.x + p.y, 0.0);
	p.xz -= min(p.x + p.z, 0.0);
	p.yz -= min(p.y + p.z, 0.0);
    return p;
}
vec3 opMengerFold(vec3 p){
    float a = min(p.x - p.y, 0.0);
	p.x -= a;
	p.y += a;
	a = min(p.x - p.z, 0.0);
	p.x -= a;
	p.z += a;
	a = min(p.y - p.z, 0.0);
	p.y -= a;
	p.z += a;
    return p;
}
vec3 opRep(vec3 p, vec3 c)
{
    vec3 q = mod(p+0.5*c,c)-0.5*c;
    return q;
}
vec3 opSymX(vec3 p)
{
    p.x = abs(p.x);
    return p;
}
vec3 opSymY(vec3 p)
{
    p.y = abs(p.y);
    return p;
}
vec3 opSymZ(vec3 p)
{
    p.z = abs(p.z);
    return p;
}
vec3 opSymXYZ(vec3 p)
{
    p.xyz = abs(p.xyz);
    return p;
}
vec3 voxelise(vec3 p){
    //p = vec3(floor(p.x),floor(p.y),floor(p.z));
    return round(p*slider3)/slider3;
}
float sphereSDF(vec3 p,float r){
    return length(p)- r;
}
float roundSDF(float sdf,float r){
    return sdf-r;
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

vec4 menger_spong_inigo_quilez(vec3 p )
{
   float d = boxSDF(p,vec3(1.0));
   vec4 res = vec4( d, 1.0, 0.0, 0.0 );

   float s = 1.0;
   for( int m=0; m<5; m++ )
   {
      vec3 a = mod( p*s, 2.0 )-1.0;
      s *= 3.0;
      vec3 r = abs(1.0 - 3.0*abs(a)*slider3)/slider3;

      float da = max(r.x,r.y);
      float db = max(r.y,r.z);
      float dc = max(r.z,r.x);
      float c = (min(da,min(db,dc))-1.0)/s;

      if( c>d )
      {
          d = c;
          res = vec4( d, 0.2*da*db*dc, (1.0+float(m))/4.0, 0.0 );
       }
   }

   return res;
}
float mandelbulb(vec3 p){
    vec3 z = p;
    float dr = 1.0;
    float r = 0.0;
    int i;
    for(i = 0;i<MANDELBULB_ITERS;i++){
        r = length(z);
        if(r>2.0){break;}
        float theta = acos(z.z/r);
        float phi = atan(z.y,z.x);
        dr = pow(r,MANDELBULB_POWER-1.0)*MANDELBULB_POWER*dr+1.0;
        float zr = pow(r,MANDELBULB_POWER);
        theta *= MANDELBULB_POWER;
        phi *= MANDELBULB_POWER;
        z = zr*vec3(
            sin(theta)*cos(phi),
            sin(phi)*sin(theta),
            cos(theta)
        );
        z += p;
        z=opSymX(z);
    }
    return 0.5*log(r)*r/dr;
}
float some_fractal(vec3 p){
    for(int i =0;i<10;i++){
        p.xyz = abs(p.xyz);
        p = (rotationMatrix(vec3(1,0,0),sin(time/5))*vec4(p,0)).xyz;
        opMengerFold(p);
        p = (rotationMatrix(vec3(0,0,1),cos(time/7))*vec4(p,0)).xyz;
    }
    return menger_spong_inigo_quilez(p)[0];
}
float sceneSDF(vec3 p){
    //p = (rotationMatrix(vec3(sin(time),cos(time),0),time)*vec4(p,0)).xyz;
    //p = opPlaneFold(p,vec3(1,0,0),slider4);
    return menger_spong_inigo_quilez(p)[0];
    //return menger_spong_inigo_quilez(p)[0];
    //return mandelbulb(p);
}

vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

float rayMarcher(vec3 eye,vec3 dir,float start,float end,float step){
    float depth = start;
    int i;
    for(i =0;i<MAX_ITERS;i++){
        float dist = sceneSDF(eye + depth * dir);
        if (dist < step){
            if(rayMarcherIters==-1){
                rayMarcherIters=i;
            }
            return depth;
        }
        depth += dist;
        if(depth>= end){
            return end;
        }
    }
    return end;
}
float ambient_occlusion(vec3 p,vec3 n){
    float sum = 0;
    for (int i = 0;i<MAX_AO_ITERS;i++){
        sum += sceneSDF(p+n*(i+1)*AO_EPSILON);
    }
    return sum/(MAX_AO_ITERS*AO_EPSILON)*AO_COEFICIENT;
}
vec3 backgroundColor(vec3 dir){
    return vec3(dir[1]/2+0.5,dir[1]/2+0.5,dir[1]/2+0.5);
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
        return vec3(0,0,0);
    } 
    
    if (dotRV < 0.0) {
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}
vec3 palette( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d )
{
    return a + b*cos( 6.28318*(c*t+d) );
}
vec3 coloring(vec3 p){
    return palette(length(p)/slider5, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(2.0,1.0,0.0),vec3(0.5,0.20,0.25));
}

vec3 lightning(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    
    /*vec3 light1Pos = vec3(4.0 * sin(time),
                          0,
                          4.0 * cos(time));*/
        
    
    //AO
    //color *= ambient_occlusion(p,estimateNormal(p));
    //AO cheap
    color *= (1-float(rayMarcherIters)/float(MAX_ITERS-1));
    //shadow
    /*
    if (rayMarcher(p,normalize(global_light_pos-p),MIN_SHADOW_DIST,MAX_SHADOW_DIST,SHADOW_EPSILON) < MAX_SHADOW_DIST-SHADOW_EPSILON){
        return color;
    }
    */

    //point source
    //vec3 light1Intensity = vec3(1,1,1)*0.5 / sqrt(length(p-global_light_pos));
    //sun light
    /*
    vec3 light1Intensity = vec3(1,1,1)*0.5;
    color += phongContribForLight(k_d, k_s, alpha, p, eye,global_light_pos,light1Intensity);
    */
    return color;
}

vec3 rayDirection(float FOV,vec2 d0){
    d0.y *= (768.0/1024.0);
    float z = 1/tan(FOV/2);
    return normalize(vec3(d0,-z));
}

void main(){
    vec3 dir = rayDirection(slider1,pix_coord);
    dir = (rotationMatrix(vec3(0,1,0),-cam_hor_angle) * rotationMatrix(vec3(1,0,0),-cam_ver_angle) * vec4(dir,0)).xyz;
    vec3 eye = vec3(-pos.x,pos.y,-pos.z);
    float dist = rayMarcher(eye,dir,MIN_DIST,MAX_DIST,EPSILON);
    if(dist > MAX_DIST-EPSILON){
        color = backgroundColor(dir);
        return;
    }

    vec3 p = eye + dist * dir;

    vec3 ambientColor = vec3(0.2,0.6,1)*1;
    vec3 diffuseColor = vec3(0.332,0.664,1.0);
    vec3 specularColor = vec3(1,1,1);
    float shininess = 10;
    //color = coloring(p);
    color = lightning(ambientColor,diffuseColor,specularColor,shininess,p,eye);
    //color = lightning(ambientColor,diffuseColor,specularColor,shininess,p,eye);
}