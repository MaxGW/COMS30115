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
};

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
void VertexShader( vec4& v, ivec2& p);
void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result);
void DrawLineSDL(screen* screen, ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(screen* screen,  vector<vec4>& vertices);
void updateRotationMatrix(float cumYaw);
void ComputePolygonRows(vector<ivec2>& vertexPixels,vector<ivec2>& leftPixels,vector<ivec2>& rightPixels);
void DrawPolygonRows(screen* screen, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels);
void DrawPolygon(screen* screen,  vector<vec4>& vertices);
void InterpolateP(Pixel a, Pixel b, vector<Pixel>& result);

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
void VertexShader( vec4& v, ivec2& p){
  vec4 pp;
  //update rotation matrix R from keyboard input
  updateRotationMatrix(cumYaw);
  //calculate rotated and transformed matrix pp
  pp = glm::inverse(R) * (v - cameraPos);

  //store projected co-ords in p
  p.x = (f * pp.x) / pp.z + (SCREEN_WIDTH / 2);
  p.y = (f * pp.y) / pp.z + (SCREEN_HEIGHT / 2);

}

//fucntion draws a line between two given points in the given colour
void DrawLineSDL(screen* screen, ivec2 a, ivec2 b, vec3 color){
  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels );
  Interpolate( a, b, line );

  for(uint32_t i = 0; i < line.size(); i++){
    PutPixelSDL(screen, line[i].x, line[i].y, color);
  }
}

//function interpolates between two given points and outputs to reults array
//size of given reults array determins the interpolation step size
void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result){

  int N = result.size();
  vec2 step = vec2(b-a) / float(max(N-1,1));
  vec2 current( a );

  for( int i=0; i<N; ++i ){
    // cout << "current = " << current.x << "," << current.y << " rounded = " << round(current).x << "," << round(current).y << "\n";
    result[i] = round(current);
    // cout << "i = " << i << " result = (" << result[i].x << "," << result[i].y << ")\n";
    current += step;
  }
}

//function interpolates with pixel inputs
void InterpolateP(Pixel a, Pixel b, vector<Pixel>& result){
  int N = result.size();
  float xStep = (b.x - a.x) / float(max(N-1,1));
  float yStep = (b.y - a.y) / float(max(N-1,1));
  float zStep = (b.zinv - a.zinv) / float(max(N-1,1));
  Pixel current(a);

  for(int i=0; i<N; ++i){
    result[i].x = round(current.x);
    result[i].y = round(current.y);
    result[i].zinv = current.zinv;
    current.x += xStep;
    current.y += yStep;
    current.zinv += zStep;
  }

}

/* ----------------------------------------------------------------------------*/
/* DRAWING THE STUFF                                                           */


//fucntion receives array of projected vertices vertexPixels of a given shape
//function interpolates between points, outputting furthest left and right pixels into arrays
void ComputePolygonRows(vector<ivec2>& vertexPixels,vector<ivec2>& leftPixels,vector<ivec2>& rightPixels){
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
    ivec2 difference = abs(vertexPixels[i] - vertexPixels[j]);

    //find number of rows between two vertices (different to ROWS)
    // int intArraySize = abs(vertexPixels[i].y - vertexPixels[j].y) + 1;
    //allocate vector array for interpolated co-ords
    int intArraySize = glm::max(difference.x, difference.y) + 1;

    vector<ivec2> line(intArraySize);
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
      }
      if(line[y].x < leftPixels[offset].x){
        leftPixels[offset].x = line[y].x;
      }
    }
  }
}



//function draws row by row between leftPixels and rightPixels
void DrawPolygonRows(screen* screen,  vector<ivec2>& leftPixels,  vector<ivec2>& rightPixels){

  if(rightPixels.size() != leftPixels.size()){
    cout << "leftPixels rightPixels array size doesn't match";
  }
  else{
    for(uint i = 0; i < leftPixels.size(); i++){
      //how many pixels between left and right
      int n = rightPixels[i].x - leftPixels[i].x + 1;
      // call PutPixelSDL for every pixel to be drawn
      for(int j = 0; j < n; j++){
        PutPixelSDL(screen, (leftPixels[i].x + j), leftPixels[i].y, currentColour);
      }
    }
  }
}





void DrawPolygonEdges(screen* screen,  vector<vec4>& vertices ){
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i ){
    VertexShader(vertices[i], projectedVertices[i]);
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i ){
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}



//given array of 4D points making a polygon, function projects points and draws
void DrawPolygon(screen* screen,  vector<vec4>& vertices){
  int V = vertices.size();
  vector<ivec2> vertexPixels( V );
  for( int i=0; i<V; ++i ){
    VertexShader( vertices[i], vertexPixels[i] );
  }
  vector<ivec2> leftPixels;
  vector<ivec2> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawPolygonRows(screen, leftPixels, rightPixels);

}


void Draw(screen* screen){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( uint32_t i=0; i<triangles.size(); ++i ){

    vector<vec4> vertices(3);
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;

    currentColour = triangles[i].color;

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
