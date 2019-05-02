#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#define USE_MATH_DEFINES
#include <math.h>
#include "omp.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 150
#define SCREEN_HEIGHT 150
#define FULLSCREEN_MODE false


struct Intersection{
  vec4 position;
  float distance;
  int triangleIndex;
};

/* ----------------------------------------------------------------------------*/
/* Globals                                                                     */

vector<Triangle> triangles;
vec4 cameraPos(0,0,-3,1);
float focalLength =  SCREEN_HEIGHT;

//Rotation
mat4 R;
float yaw = 0.1;
//Illumination
vec4 lightPos(0, -0.5, -0.7, 1.0);
vec3 lightColour = 14.f * vec3(1,1,1);
vec3 indirectLight =  0.5f* vec3 (1,1,1);
float antiAliasingValue = 4;
bool negativeDistanceFlag = false;


/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
vec3 FindIntersection(Triangle triangle, vec4 d, vec4 s);
bool ClosestIntersection(vec4 s, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
void updateRotationMatrix(float yaw);
mat4 LookAt(vec3 from, vec3 to);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  LoadTestModel(triangles);

  while ( Update())
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "everton.bmp" );

  KillSDL(screen);
  return 0;
}

vec3 FindIntersection(Triangle triangle, vec4 d, vec4 s){
  vec4 v0 = triangle.v0;
  vec4 v1 = triangle.v1;
  vec4 v2 = triangle.v2;

  vec3 e1 = vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
  vec3 e2 = vec3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
  //b = s - v0
  vec3 b  = vec3(s.x - v0.x, s.y - v0.y, s.z - v0.z);

  //(-d, e1, e2)[(t, u, v)] = s - v0
  //  A           x         = b
  //  x = inverse(A) * b

  mat3 A(vec3(-d), e1, e2);

// //CRAMERS (not fully implemented)
//   mat3 tempA = A;
//   for(int i = 0; i<3; i++){
//     tempA[0][i] = b[i];
//   }
//   float tt = glm::determinant(tempA) / glm::determinant(A);
//   if(tt < 0){
//     negativeDistanceFlag = true;
//     return vec3(-1,-1,-1);
//   }

  return inverse(A) * b;
}

bool ClosestIntersection(vec4 s, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection){
  int closestIndex = -1;
  float closestDistance = std::numeric_limits<float>::max() - 1;
  int isValidIntersection = 0;

  //loop through each triangle testing for intersection through a given ray direction
  //for(int i = 0; i<triangles.size(); i++){
  for(size_t i = 0; i<triangles.size(); i++){
    vec3 x = FindIntersection(triangles[i], dir, s);

    //Checks for Cramers rule (not fully implemented)
    // if(negativeDistanceFlag){
    //   printf("***\n");
    //   negativeDistanceFlag = false;
    //   skip;
    // }

    //if valid intersection
    if( x.y >= 0 && x.z >= 0 && (x.y + x.z) <= 1 && x.x >= 0 ){
      //if closer than previous valid intersection update shortest distance + store index
      if(x.x < closestDistance){
          closestIndex = i;
          closestDistance = x.x;
      }
      //variable recording the existence of a valid interesection
      isValidIntersection = 1;
    }
  }

  if(isValidIntersection){
    //update global values for the closestIntersection
    vec3 closestX = FindIntersection(triangles[closestIndex], dir, s);
    // r = s + td
    closestIntersection.position = s + closestX.x*dir;
    closestIntersection.distance = closestX.x;
    closestIntersection.triangleIndex = closestIndex;
  }
  return isValidIntersection;
}


vec3 DirectLight(Intersection& i){
  Intersection lightIntersection;

  //adjust intersection point along direction from i to light by scalar
  vec4 weight = float(0.01)*((lightPos - i.position));
  vec4 adjustedIntersection = i.position + weight;

  //vector from intersection to light
  vec4 r = lightPos - adjustedIntersection;
  //distnace between light and intersection
  float dist = length(vec3(r));

  vec3 temp_dir = normalize(vec3(r));
  vec4 dir(temp_dir.x, temp_dir.y, temp_dir.z, 1);

  //Calculating shadows
  if(ClosestIntersection(adjustedIntersection, dir, triangles, lightIntersection)){
    if(lightIntersection.distance < dist){
      return vec3(0,0,0);
    }
  }

  //B equals the amount of power hitting Intersection
  double radiusSquared = pow(dist,2);
  float temp = 1 / (4 * M_PI * radiusSquared);
  vec3 B = lightColour * temp;
  //interestion's normal vector
  vec3 n = normalize(vec3(triangles[i.triangleIndex].normal));
  float projection = dot(n,normalize(vec3(r)));

  //return either dot product of direction vectors, or 0.
  return (B * max(projection, float(0)));
}


//Function calculates direction matrix between two given vec3 points
mat4 LookAt(vec3 from, vec3 to){
  mat4 camToWorld;

  vec3 forward = glm::normalize(from - to);
  vec3 right = glm::cross(glm::normalize(vec3(0,1,0)), forward);
  vec3 up = glm::cross(forward, right);

  camToWorld[0][0] = right.x;
  camToWorld[0][1] = right.y;
  camToWorld[0][2] = right.z;
  camToWorld[1][0] = up.x;
  camToWorld[1][1] = up.y;
  camToWorld[1][2] = up.z;
  camToWorld[2][0] = forward.x;
  camToWorld[2][1] = forward.y;
  camToWorld[2][2] = forward.z;

  camToWorld[3][0] = from.x;
  camToWorld[3][1] = from.y;
  camToWorld[3][2] = from.z;

  camToWorld[0][3] = 0;
  camToWorld[1][3] = 0;
  camToWorld[2][3] = 0;
  camToWorld[3][3] = 1;

  return camToWorld;
}


/*Place your drawing here*/
void Draw(screen* screen){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 colour;
  Intersection closestIntersection;

  //#pragma omp parallel for schedule (static,10)
  for (int y=0; y<SCREEN_HEIGHT; y++){

    for(int x=0; x<SCREEN_WIDTH; x++){
      vec3 cumulativeColour = vec3(0,0,0);

      //anti-aliasing
      //calculate a pixel's value by including its (antiAliasingValue x antiAliasingValue) neighbours
      for(int j = 0; j < antiAliasingValue; j++){
        for(int i=0; i< antiAliasingValue; i++){

          //point defines the co-ordinate in the image plane
          vec4 point = vec4(-(x-(SCREEN_WIDTH/2)), y-(SCREEN_HEIGHT/2), -focalLength, 1.0);

          //calculating offset variable
          float xOffset = (i - (antiAliasingValue / 2))/(2*antiAliasingValue);
          float yOffset = (j - (antiAliasingValue / 2))/(2*antiAliasingValue);
          //offset vector to be added to 3D point co-ords to obtain points close
          vec4 offset = vec4(xOffset, yOffset, 0.f, 0.f);

          //storing directional vector from the camera position to centre of image
          mat4 camToWorld = LookAt(vec3(cameraPos), vec3(0,0,0));
          //update the point with camToWorld direction and set dir equal to vector from camPos to point
          vec3 temp_dir = normalize(vec3((camToWorld*(point+offset)) - cameraPos));

          vec4 dir(temp_dir.x, temp_dir.y, temp_dir.z, 1);

          if(ClosestIntersection(cameraPos, dir, triangles, closestIntersection)){
            vec3 colourTemp =  triangles[closestIntersection.triangleIndex].color;
            colour = colourTemp * (DirectLight(closestIntersection) + indirectLight);
          }
          else{
            colour = vec3(0,0,0);
          }
          //summing colour of central points and its neighbours for anti-aliasing
          cumulativeColour += colour;
        }
      }
      //find average colour of point and its neighbours
      int aa = antiAliasingValue * antiAliasingValue;
      colour = vec3(cumulativeColour.x / aa, cumulativeColour.y / aa, cumulativeColour.z / aa);
      //draw pixel
      PutPixelSDL(screen, x, y, colour);
    }
  }
}


//funtion updates global rotation matrix from a given yaw
void updateRotationMatrix(float yaw){
    R = mat4(cos(yaw) , 0, sin(yaw), 0,
           0        , 1, 0       , 0,
           -sin(yaw), 0, cos(yaw), 0,
           0       , 0, 0      , 1);
}


bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  SDL_Event e;
  while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
	{
	  return false;
	}
      else
	if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
    //__________________________________//
		/* Move camera forward */
	      case SDLK_UP:
          cameraPos.z += 0.5;
      	break;
    //__________________________________//
    /* Move camera backwards */
        case SDLK_DOWN:
          cameraPos.z -= 0.5;
		    break;
    //__________________________________//
    /* Move camera left */
	      case SDLK_LEFT:
          updateRotationMatrix(-yaw);
          cameraPos = R * cameraPos;
		    break;
    //__________________________________//
    /* Move camera right */
	      case SDLK_RIGHT:

          updateRotationMatrix(yaw);
          cameraPos = R * cameraPos;
		    break;
    //__________________________________//
    /* Move camera forward */
        case SDLK_w:
          lightPos.z += 0.2;
        break;
    //__________________________________//
    /* Move camera backwards */
        case SDLK_s:
          lightPos.z -= 0.2;
        break;
    //__________________________________//
    /* Move camera left */
        case SDLK_a:
          lightPos.x -= 0.2;
        break;
    //__________________________________//
    /* Move camera right */
        case SDLK_d:
          lightPos.x += 0.2;
        break;
    //__________________________________//
    /* Move camera up */
        case SDLK_q:
          lightPos.y += 0.2;
        break;
        //__________________________________//
    /* Move camera down  */
        case SDLK_e:
          lightPos.y -= 0.2;
        break;
    //__________________________________//
    /* Move camera quit */
	      case SDLK_ESCAPE:
        break;

		return false;
	      }
	  }
    }
  return true;
}
