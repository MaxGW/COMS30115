#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::ivec2;
using glm::vec2;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

//struct containing inverse of shortest depth in variable zinv and co-ords (x,y)
struct Pixel{
  int x;
  int y;
  float zinv;
  vec4 pos3D;
};

struct Vertex{
  vec4 position;
  //vec4 normal;
  //Carls initial reflectance was vec2
  //vec3 reflectance;
};

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
void Interpolate(Pixel a, Pixel b, vector<Pixel>& result);
void DrawLineSDL(screen* screen, Pixel a, Pixel b, vec3 color);
void DrawPolygonEdges(screen* screen,  vector<vec4>& vertices);
void updateRotationMatrix(float cumYaw);
void ComputePolygonRows(vector<ivec2>& vertexPixels,vector<ivec2>& leftPixels,vector<ivec2>& rightPixels);
void DrawPolygonRows(screen* screen, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void DrawPolygon(screen* screen,  vector<Vertex>& vertices);
// void InterpolateP(Pixel a, Pixel b, vector<Pixel>& result);
void VertexShader( Vertex& v, Pixel &p);

/* ----------------------------------------------------------------------------*/
/*   GLOBALS                                                                   */

float f = SCREEN_HEIGHT;
vec4 cameraPos( 0, 0, -3.001,1 );
vector<Triangle> triangles;
mat4 R;
float yaw = 0.05;
float cumYaw = 0;
vector<ivec2> leftPixels();
vector<ivec2> rightPixels();
vec3 currentColour;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
//Lighting
vec4 lightPos(0,-0.5,-0.7,1.0);
vec3 lightPower = 10.0f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vec4 currentNormal;
vec3 currentReflectance;

int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);

// //______________________________________
// //testing ComputePolygonRows function
//   vector<ivec2> vertexPixels(3);
//   vertexPixels[0] = ivec2(10, 5);
//   vertexPixels[1] = ivec2( 5,10);
//   vertexPixels[2] = ivec2(15,15);
//   vector<ivec2> leftPixels;
//   vector<ivec2> rightPixels;
//   vector<ivec2> loine(11);
//   ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
//
//   for( uint row=0; row<leftPixels.size(); ++row ){
//     cout << "Start: ("<< leftPixels[row].x << ","<< leftPixels[row].y << "). "
//     << "End: ("<< rightPixels[row].x << ","<< rightPixels[row].y << "). " << endl;
//   }
//   return 0;
// //__________________________________


  //updating frame
  while ( Update())
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}


//projects 3D point v to 2D image plane storing co-ords in given p int vector
void VertexShader( Vertex& v, Pixel &p){
  vec4 pp;
  //update rotation matrix R from keyboard input
  updateRotationMatrix(cumYaw);
  //calculate rotated and transformed matrix pp
  pp = glm::inverse(R) * (v.position - cameraPos);

  //store projected co-ords in p
  p.x = (f * pp.x) / pp.z + (SCREEN_WIDTH / 2);
  p.y = (f * pp.y) / pp.z + (SCREEN_HEIGHT / 2);
  p.zinv = 1 / pp.z;
  p.pos3D = v.position;
}

//fucntion draws a line between two given points in the given colour
void DrawLineSDL(screen* screen, Pixel a, Pixel b, vec3 color){
  ivec2 delta;
  delta.x = glm::abs( a.x - b.x );
  delta.y = glm::abs( a.y - b.y );

  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<Pixel> line( pixels );
  Interpolate( a, b, line );

  for(uint32_t i = 0; i < line.size(); i++){
    int x = line[i].x;
    int y = line[i].y;
    float zinv = line[i].zinv;
    vec4 pos = line[i].pos3D;

    //check if co-ords are within image plane
    if(x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT){
      //if zinv is larger than currently stored pixel, the point is closer
      if(zinv > depthBuffer[y][x]){

        //D equals the amount of power hitting
        float r = glm::distance(pos,lightPos);
        double radiusSquared = pow(r,2);
        float temp = 1 / (4 * M_PI * radiusSquared);
        vec3 A = lightPower * temp;
        //interestion's normal vector
        vec3 n = normalize(vec3(currentNormal));
        float projection = dot(n, normalize(vec3(r)));

        vec3 D = max(projection, 0.f) * A;
        //R = p(D+N)
        vec3 illumination = currentReflectance * (D + indirectLightPowerPerArea);

        //update depthBuffer value
        depthBuffer[y][x] = zinv;
        PutPixelSDL(screen, line[i].x, line[i].y, illumination);
      }
    }
  }
}

//function interpolates between two given points and outputs to reults array
//size of given reults array determins the interpolation step size
void Interpolate(Pixel a, Pixel b, vector<Pixel>& result){

  //convert a,b pixels to ivecs
  ivec2 aa = ivec2(a.x, a.y);
  ivec2 bb = ivec2(b.x, b.y);

  int N = result.size();
  //calculate steps in x,y,z
  vec2 step = vec2(bb-aa) / float(max(N-1,1));
  float zStep = (b.zinv - a.zinv) / float(max(N-1,1));
  vec4 pStep = (b.pos3D - a.pos3D) / float(max(N-1,1));
  vec2 current( aa );
  float currentZ = a.zinv;
  vec4 currentP = a.pos3D;

  for( int i=0; i<N; ++i ){
    // cout << "current = " << current.x << "," << current.y << " rounded = " << round(current).x << "," << round(current).y << "\n";
    result[i].x = round(current.x);
    result[i].y = round(current.y);
    result[i].zinv = currentZ;
    result[i].pos3D = currentP;
    // cout << "i = " << i << " result = (" << result[i].x << "," << result[i].y << ")\n";
    current += step;
    currentZ += zStep;
    currentP += pStep;
  }
}

/* ----------------------------------------------------------------------------*/
/* DRAWING THE STUFF                                                           */


//fucntion receives array of projected vertices vertexPixels of a given shape
//function interpolates between points, outputting furthest left and right pixels into arrays
void ComputePolygonRows(vector<Pixel>& vertexPixels,vector<Pixel>& leftPixels,vector<Pixel>& rightPixels){
  // 1. Find max and min y-value of the polygon
  //    and compute the number of rows it occupies.
  int maxY = -numeric_limits<int>::max();
  int minY = numeric_limits<int>::max();
  int vertexNo = vertexPixels.size();

  //find min and max y co-ords from given projected points
  for(int i=0; i < vertexNo; i++){
    if(vertexPixels[i].y > maxY ){
      maxY = vertexPixels[i].y;
    }
    if(vertexPixels[i].y < minY){
      minY = vertexPixels[i].y;
    }
  }

  //calulate number of rows
  int rows = maxY - minY + 1;

  // 2. Resize leftPixels and rightPixels
  //    so that they have an element for each row.
  leftPixels.resize(rows);
  rightPixels.resize(rows);
  // 3. Initialize the x-coordinates in leftPixels
  //    to some really large value and the x-coordinates
  //    in rightPixels to some really small value.
  //  Initialise y values
  for( int i=0; i<rows; ++i ){
    leftPixels[i].x  = numeric_limits<int>::max();
    leftPixels[i].y = minY + i;
    rightPixels[i].x = -numeric_limits<int>::max();
    rightPixels[i].y = minY + i;
  }
  // 4. Loop through all edges of the polygon and use
  //    linear interpolation to find the x-coordinate for
  //    each row it occupies. Update the corresponding
  //    values in rightPixels and leftPixels.

  for(int i = 0; i < vertexNo; i++){
    //find next vertex
    int j = (i+1)%vertexNo;
    Pixel difference;
    difference.x = abs(vertexPixels[i].x - vertexPixels[j].x);
    difference.y = abs(vertexPixels[i].y - vertexPixels[j].y);


    //find number of rows between two vertices (different to ROWS)
    // int intArraySize = abs(vertexPixels[i].y - vertexPixels[j].y) + 1;
    //allocate vector array for interpolated co-ords
    int intArraySize = glm::max(difference.x, difference.y) + 1;

    vector<Pixel> line(intArraySize);
    //interpolate between two projected vertices, filling results array
    Interpolate(vertexPixels[i], vertexPixels[j], line);



    for(int y = 0; y < intArraySize; y++){
      //adjusting offset between array index and absolute co-ordinate position
      int offset = line[y].y - minY;
      //cout << "line = " << line[y].y << " minY = " << minY << " offset = " << offset << "\n " ;

      //if smaller than the current left pixel value, must be an edge, replace value

      //if larger  than right pixel value, must be an edge, replace value
      if(line[y].x > rightPixels[offset].x){
        rightPixels[offset].x = line[y].x;
        rightPixels[offset].zinv = line[y].zinv;
        rightPixels[offset].pos3D = line[y].pos3D;
      }
      if(line[y].x < leftPixels[offset].x){
        leftPixels[offset].x = line[y].x;
        leftPixels[offset].zinv = line[y].zinv;
        leftPixels[offset].pos3D = line[y].pos3D;
      }
    }
  }
}



//function draws row by row between leftPixels and rightPixels
void DrawPolygonRows(screen* screen,  vector<Pixel>& leftPixels,  vector<Pixel>& rightPixels){

  if(rightPixels.size() != leftPixels.size()){
    cout << "leftPixels rightPixels array size doesn't match";
  }
  else{
    for(uint i = 0; i < leftPixels.size(); i++){
      DrawLineSDL(screen, leftPixels[i], rightPixels[i], currentColour);
    }
  }
}

// void DrawPolygonEdges(screen* screen,  vector<vec4>& vertices ){
//   int V = vertices.size();
//   // Transform each vertex from 3D world position to 2D image position:
//   vector<ivec2> projectedVertices( V );
//   for( int i=0; i<V; ++i ){
//     VertexShader(vertices[i], projectedVertices[i]);
//   }
//   // Loop over all vertices and draw the edge from it to the next vertex:
//   for( int i=0; i<V; ++i ){
//     int j = (i+1)%V; // The next vertex
//     vec3 color( 1, 1, 1 );
//     DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
//   }
// }



//given array of 4D points making a polygon, function projects points and draws
void DrawPolygon(screen* screen,  vector<Vertex>& vertices){
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i ){
    VertexShader( vertices[i], vertexPixels[i] );
  }
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawPolygonRows(screen, leftPixels, rightPixels);

}


void Draw(screen* screen){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  //clear depth buffer
  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ){
      depthBuffer[y][x] = 0;
    }
  }

  for( uint32_t i=0; i<triangles.size(); ++i ){

    vector<Vertex> vertices(3);
    vertices[0].position = triangles[i].v0;
    vertices[1].position = triangles[i].v1;
    vertices[2].position = triangles[i].v2;

    // vertices[0].normal = triangles[i].normal;
    // vertices[0].reflectance = triangles[i].color;
    // vertices[1].normal = triangles[i].normal;
    // vertices[1].reflectance = triangles[i].color;
    // vertices[2].normal = triangles[i].normal;
    // vertices[2].reflectance = triangles[i].color;

    currentColour = triangles[i].color;
    currentNormal = triangles[i].normal;
    currentReflectance = triangles[i].color;

    //DrawPolygonEdges(screen, vertices);

    DrawPolygon(screen, vertices);
  }
}

//funtion updates global rotation matrix from a given yaw
void updateRotationMatrix(float yaw){
    R = mat4(cos(yaw) , 0, sin(yaw), 0,
           0        , 1, 0       , 0,
           -sin(yaw), 0, cos(yaw), 0,
           0       , 0, 0      , 1);
}


/*Place updates of parameters here*/
bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  vec4 forward = vec4(R[2][0], R[2][1], R[2][2], 1);

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
              cameraPos += forward;
          	break;
        //__________________________________//
        /* Move camera backwards */
            case SDLK_DOWN:
              cameraPos -= forward;
    		    break;
        //__________________________________//
        /* Move camera left */
    	      case SDLK_LEFT:
              updateRotationMatrix(yaw);
              cumYaw += yaw;
              cameraPos = R * cameraPos;
    		    break;
        //__________________________________//
        /* Move camera right */
    	      case SDLK_RIGHT:
              updateRotationMatrix(-yaw);
              cumYaw -= yaw;
              cameraPos = R * cameraPos;
    		    break;

	      case SDLK_ESCAPE:
		/* Move camera quit */
		return false;
	      }
	  }
    }
  return true;
}
