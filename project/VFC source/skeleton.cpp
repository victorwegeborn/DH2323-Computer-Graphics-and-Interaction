#define _USE_MATH_DEFINES

#include <iostream>
#include "glm/glm.hpp"
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <math.h>
#include "Clipping.hpp"
#include "Vertex.h"


using namespace std;
using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES


const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
const int focalLength = SCREEN_HEIGHT;

Clipping clipper(SCREEN_WIDTH, focalLength);

float yaw = 0; // Yaw angle controlling camera rotation around y-axis
float translationSpeed = 0.2f;
float yawSpeed = 0.02f;
vec3 cameraPos( 0, 0, -5 );
mat3 R(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
/*
 vec3 fwd( R[2][0], R[2][1], R[2][2] );
 vec3 rgt( R[0][0], R[0][1], R[0][2] );
 vec3 dwn( R[1][0], R[1][1], R[1][2] );
 */


SDL_Surface* screen;
int t;
vector<Triangle> triangles;

vec3 currentColor;
vec3 currentNormal;
vec3 currentReflectance;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

struct Pixel
{
    int x;
    int y;
    float zinv;
    vec3 pos3d;
};

/*
struct Vertex
{
    vec3 position;
};
*/


vec3 lightPos(0,-0.5,-0.7);
vec3 lightPower = 14.1f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void VertexShader( const Vertex& v, Pixel& p );
void PixelShader( const Pixel& p );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec3>& vertices );
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels,
                        vector<Pixel>& rightPixels );
void DrawPolygonRows( const vector<ivec2>& leftPixels,
              const vector<ivec2>& rightPixels );
void DrawPolygon( const vector<Vertex>& vertices );
bool boundCheck(const Pixel& p);
vec3 illumination(const Pixel &p);



int main( int argc, char* argv[] )
{
	LoadTestModel( triangles );

    
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	t = SDL_GetTicks();	// Set start value for timer.
    
	while( NoQuitMessageSDL() )
	{
		Update();
		Draw();
	}

	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	Uint8* keystate = SDL_GetKeyState(0);
    
    
    if( keystate[SDLK_UP] ) {
        vec3 fwd( R[2][0], R[2][1], R[2][2] );;
        cameraPos += fwd*translationSpeed;
    }

    if( keystate[SDLK_DOWN] ) {
        vec3 fwd( R[2][0], R[2][1], R[2][2] );
        cameraPos -= fwd*translationSpeed;
    }

    if( keystate[SDLK_RIGHT] ) {
        yaw -= yawSpeed;
        if(yaw <= -180)
            yaw += 360;
        
        R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
    }

    if( keystate[SDLK_LEFT] ) {
        yaw += yawSpeed;
        if(yaw >= 180)
            yaw -= 360;
        
        R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
    }

	if( keystate[SDLK_RSHIFT] )
		;

	if( keystate[SDLK_RCTRL] )
		;

	if( keystate[SDLK_w] )
        lightPos += glm::vec3(0,0,0.02);
		;

	if( keystate[SDLK_s] )
        lightPos -= glm::vec3(0,0,0.02);
		;

	if( keystate[SDLK_d] )
        lightPos += glm::vec3(0.02,0,0);
		;

	if( keystate[SDLK_a] )
        lightPos -= glm::vec3(0.02,0,0);
        ;

	if( keystate[SDLK_e] )
		;

	if( keystate[SDLK_q] )
		;
    
    
}

void Draw()
{
	SDL_FillRect( screen, 0, 0 );

	if( SDL_MUSTLOCK(screen) )
		SDL_LockSurface(screen);
	
    for( int y=0; y<SCREEN_HEIGHT; ++y )
        for( int x=0; x<SCREEN_WIDTH; ++x )
            depthBuffer[y][x] = 0;
    
    vec3 right = glm::normalize(vec3( R[0][0], R[0][1], R[0][2] ));
    vec3 up = glm::normalize(vec3( -R[1][0], -R[1][1], -R[1][2] ));
    vec3 forward = glm::normalize(vec3( R[2][0], R[2][1], R[2][2] ));
    clipper.generateFrustum(cameraPos, forward, right, up);
    
    for( int i=0; i<triangles.size(); ++i )
    {
        currentColor = triangles[i].color;
        currentNormal = triangles[i].normal;
        currentReflectance = triangles[i].color;
        vector<Vertex> vertices(3);
        vertices[0].position = triangles[i].v0;
        vertices[1].position = triangles[i].v1;
        vertices[2].position = triangles[i].v2;
        clipper.clipToFrustum(vertices);
        DrawPolygon( vertices );
    }
	

    
	if ( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}


void VertexShader( const Vertex& v, Pixel& p ) {
    vec3 point = (v.position - cameraPos)*R;
    p.zinv = 1.f/point.z;
    p.x = int((float)focalLength*(point.x/point.z) + ((float)SCREEN_WIDTH/2.0f));
    p.y = int((float)focalLength*(point.y/point.z) + ((float)SCREEN_HEIGHT/2.0f));
    p.pos3d = v.position*p.zinv;
}

void PixelShader( const Pixel& p )
{
    int x = p.x;
    int y = p.y;
    if(boundCheck(p) && p.zinv > depthBuffer[y][x] )
    {
        depthBuffer[y][x] = p.zinv-0.00001f;
        PutPixelSDL( screen, x, y, illumination(p));
    }
}

/*
 *  Interpolates Pixel with 1/z-depth
 */
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ) {
    int N = int(result.size());
    float d = float(max(N-1,1));
    float xStep = float(b.x-a.x)/d;
    float yStep = float(b.y-a.y)/d;
    float zStep = float(b.zinv-a.zinv)/d;
    vec3 posStep = (b.pos3d-a.pos3d)*(1.f/d);
    float xCurrent = a.x;
    float yCurrent = a.y;
    float zCurrent = a.zinv;
    vec3 posCurrent = a.pos3d;
    for( int i=0; i<N; ++i )
    {
        result[i].x = int(xCurrent);
        result[i].y = int(yCurrent);
        result[i].zinv = zCurrent;
        result[i].pos3d = posCurrent;
        xCurrent += xStep;
        yCurrent += yStep;
        zCurrent += zStep;
        posCurrent += posStep;
    }
}



void DrawLineSDL( SDL_Surface* surface, Pixel a, Pixel b, vec3 color ) {
    
    ivec2 delta = glm::abs(ivec2( a.x - b.x, a.y - b.y ));
    int pixels = glm::max( delta.x, delta.y ) + 1;
    vector<Pixel> line( pixels );
    Interpolate( a, b, line );
    
    for(int i = 0; i < line.size(); ++i) {
        PutPixelSDL( screen, line[i].x, line[i].y, color );
    }
}

void DrawPolygonEdges( const vector<Vertex>& vertices )
{
    int V = int(vertices.size());
    // Transform each vertex from 3D world position to 2D image position:
    vector<Pixel> projectedVertices( V );
    for( int i=0; i<V; ++i )
    {
        VertexShader( vertices[i], projectedVertices[i] );
    }
    // Loop over all vertices and draw the edge from it to the next vertex:
    for( int i=0; i<V; ++i )
    {
        int j = (i+1)%V; // The next vertex
        vec3 color( 1, 1, 1 );
        DrawLineSDL( screen, projectedVertices[i], projectedVertices[j],
                    color );
    }
}


void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels,
                        vector<Pixel>& rightPixels )
{
    // 1. Find max and min y-value of the polygon
    // and compute the number of rows it occupies.
    int yMin = +numeric_limits<int>::max();
    int yMax = -numeric_limits<int>::max();;
    
    for(int i = 0; i < vertexPixels.size(); i++) {
        if(vertexPixels[i].y < yMin)
            yMin = vertexPixels[i].y;
        if(vertexPixels[i].y > yMax)
            yMax = vertexPixels[i].y;
    }
    
    int rows = yMax - yMin + 1;
    
    // 2. Resize leftPixels and rightPixels
    // so that they have an element for each row.
    leftPixels.resize(rows);
    rightPixels.resize(rows);
    
    // 3. Initialize the x-coordinates in leftPixels
    // to some really large value and the x-coordinates
    // in rightPixels to some really small value.
    for( int i=0; i<rows; ++i )
    {
        leftPixels[i].x = +numeric_limits<int>::max();
        rightPixels[i].x = -numeric_limits<int>::max();
    }
    
    // 4. Loop through all edges of the polygon and use
    // linear interpolation to find the x-coordinate for
    // each row it occupies. Update the corresponding
    // values in rightPixels and leftPixels.
    for(int i = 0; i < vertexPixels.size(); i++) {
        Pixel a = vertexPixels[i];
        Pixel b = vertexPixels[(i+1)%vertexPixels.size()];
        int edgeRows = abs( a.y - b.y ) + 1;
        vector<Pixel> edge(edgeRows);
        (a.y < b.y)?Interpolate(a, b, edge):Interpolate(b, a, edge);
        
        
        for(int j = 0; j < edge.size(); j++) {
            if(leftPixels[edge[j].y-yMin].x > edge[j].x)
                leftPixels[edge[j].y-yMin] = edge[j];
            if(rightPixels[edge[j].y-yMin].x < edge[j].x)
                rightPixels[edge[j].y-yMin] = edge[j];
        }
    }
}


void DrawPolygonRows( const vector<Pixel>& leftPixels,
                      const vector<Pixel>& rightPixels ) {
    for(int i = 0; i < leftPixels.size(); i++) {
        int pixels = abs(leftPixels[i].x - rightPixels[i].x) + 1;
        vector<Pixel> pixelRow(pixels);
        Interpolate(leftPixels[i], rightPixels[i], pixelRow);
        for(Pixel p : pixelRow) {
                p.pos3d = p.pos3d*(1.f/p.zinv);
                PixelShader(p);
        }
    }
}


void DrawPolygon( const vector<Vertex>& vertices )
{
    int V = int(vertices.size());
    vector<Pixel> vertexPixels( V );
    for( int i=0; i<V; ++i )
        VertexShader( vertices[i], vertexPixels[i] );
    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawPolygonRows( leftPixels, rightPixels );
}

bool boundCheck(const Pixel& p) {
    if(p.x < 0 || p.y < 0 || p.x > SCREEN_WIDTH || p.y > SCREEN_HEIGHT)
        return false;
    return true;
}

vec3 illumination(const Pixel &p) {
    vec3 pointToLight = glm::normalize(lightPos-p.pos3d);
    float pointToLightDistance = glm::distance(lightPos, p.pos3d);
    float dot = glm::dot(pointToLight, currentNormal);
    vec3 directLightPerArea = lightPower*max(dot,0.f)*(1.f/(4*(float)M_PI*pointToLightDistance*pointToLightDistance));
    return currentReflectance*(directLightPerArea+indirectLightPowerPerArea);
}

