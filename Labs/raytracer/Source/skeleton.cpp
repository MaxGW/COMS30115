      #include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#define USE_MATH_DEFINES
#include <math.h>

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


/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
vec3 FindIntersection(Triangle triangle, vec4 d, vec4 s);
bool ClosestIntersection(vec4 s, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
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

  //int detA = glm::determinant(A);
  //float tt = (b.x*A[0][0] + b.y*A[0][1] + b.z*A[0][2]) / detA;

/*
//CRAMERS (doesnt work)
  mat3 tempA = A;

  for(int i = 0; i<3; i++){
    tempA[i][0] = b[i];
  }
  float tt = glm::determinant(tempA) / glm::determinant(A);

  // cout << "b[i] = ("  << b[0] << ", " = v
  //      << b[1] << ", "
  //      << b[2] << ")" << "\n";
  // cout << "a[i] = ("  << tempA[0][0] << ", "
  //      << tempA[1][0] << ", "
  //      << tempA[2][0] << ")" << "\n";
  // cout << "Cramers tt = " << tt << "\n";
  // cout << "Inverse tt = "  << x.x << "\n";

  if(tt < 0){
    return vec3(-1,-1,-1);
  }*/


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

    if(x == vec3(-1,-1,-1)){

      printf("***\n");

      x = vec3(0,0,0);
      continue;
    }

  //  printf("===\n");

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

//  vec4 weight = float(0.01)*normalize(triangles[i.triangleIndex].normal);

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

  for (int y=0; y<SCREEN_HEIGHT; y++){

    for(int x=0; x<SCREEN_WIDTH; x++){
      vec3 cumColour = vec3(0,0,0);

      for(int j = 0; j < antiAliasingValue; j++){
        for(int i=0; i< antiAliasingValue; i++){


          //point defines the co-ordinate in the image plane
          vec4 point = vec4(-(x-(SCREEN_WIDTH/2)), y-(SCREEN_HEIGHT/2), -focalLength, 1.0);
          float xOffset = (i - (antiAliasingValue / 2))/antiAliasingValue;
          float yOffset = (j - (antiAliasingValue / 2))/antiAliasingValue;
          float temppp = (i - (antiAliasingValue / 2));
          // cout << temppp << "*\n";
          // cout << "i = " << i << "j = " << j << "\n" << antiAliasingValue;
          // cout << "x = " << xOffset << "y =" << yOffset;
          vec4 offset = vec4(xOffset, yOffset, 0.f, 0.f);

          //storing directional vector from the camera position to centre of image
          mat4 camToWorld = LookAt(vec3(cameraPos), vec3(0,0,0));
          //update the point with camToWorld direction and set dir equal to vector from camPos to point
          vec3 temp_dir = normalize(vec3((camToWorld*(point+offset)) - cameraPos));

          vec4 dir(temp_dir.x, temp_dir.y, temp_dir.z, 1);
          //cout << dir.x << dir.y;

          if(ClosestIntersection(cameraPos, dir, triangles, closestIntersection)){
            vec3 colourTemp =  triangles[closestIntersection.triangleIndex].color;
            colour = colourTemp * (DirectLight(closestIntersection) + indirectLight);
          }
          else{
            colour = vec3(0,0,0);
          }
          cumColour += colour;
        }
      }

      int aa = antiAliasingValue * antiAliasingValue;
      colour = vec3(cumColour.x / aa, cumColour.y / aa, cumColour.z / aa);
      PutPixelSDL(screen, x, y, colour);
    }
  }
}



/*Place updates of parameters here*/
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

    //cameraPos.x -= 0.5;
          R = mat4(cos(-yaw) , 0, sin(-yaw), 0,
                 0        , 1, 0       , 0,
                 -sin(-yaw), 0, cos(-yaw), 0,
                 0       , 0, 0      , 1);

          cameraPos = R * cameraPos;
		    break;
    //__________________________________//
    /* Move camera right */
	      case SDLK_RIGHT:

          R = mat4(cos(yaw) , 0, sin(yaw), 0,
                 0        , 1, 0       , 0,
                 -sin(yaw), 0, cos(yaw), 0,
                 0       , 0, 0      , 1);

          //cameraPos.x += 0.5;
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
