#include <iostream>
#include "glm/glm.hpp"
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::ivec2;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
const int focalLength = SCREEN_HEIGHT;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;
vec3 currentColor;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

struct Pixel
{
    int x;
    int y;
    float zinv;
};

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

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void VertexShader( const vec3& v, ivec2& p );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec3>& vertices );
void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels,
                        vector<ivec2>& rightPixels );
void DrawPolygonRows( const vector<ivec2>& leftPixels,
              const vector<ivec2>& rightPixels );
void DrawPolygon( const vector<vec3>& vertices );


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
        yaw += yawSpeed;
        if(yaw >= 180)
            yaw -= 360;
        
        R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
    }

    if( keystate[SDLK_LEFT] ) {
        yaw -= yawSpeed;
        if(yaw <= -180)
            yaw += 360;
        
        R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
    }

	if( keystate[SDLK_RSHIFT] )
		;

	if( keystate[SDLK_RCTRL] )
		;

	if( keystate[SDLK_w] )
		;

	if( keystate[SDLK_s] )
		;

	if( keystate[SDLK_d] )
		;

	if( keystate[SDLK_a] )
		;

	if( keystate[SDLK_e] )
		;

	if( keystate[SDLK_q] )
		;
    
    cout << "yaw::" << yaw << "\n";
    
    
}

void Draw()
{
	SDL_FillRect( screen, 0, 0 );

	if( SDL_MUSTLOCK(screen) )
		SDL_LockSurface(screen);
	
    for( int i=0; i<triangles.size(); ++i )
    {
        currentColor = triangles[i].color;
        vector<vec3> vertices(3);
        vertices[0] = triangles[i].v0;
        vertices[1] = triangles[i].v1;
        vertices[2] = triangles[i].v2;
        DrawPolygon( vertices );
    }
	

    
	if ( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}


void VertexShader( const vec3& v, ivec2& p ) {
    
    vec3 point = R*(v - cameraPos);
    p.x = round((float)focalLength*(point.x/point.z) + ((float)SCREEN_WIDTH/2.0f));
    p.y = round((float)focalLength*(point.y/point.z) + ((float)SCREEN_HEIGHT/2.0f));
}

/*
 *  Interpolates Pixel with 1/z-depth
 */
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ) {
    int N = int(result.size());
    float xStep = float(b.x-a.x)/float(max(N-1,1));
    float yStep = float(b.y-a.y)/float(max(N-1,1));
    float zStep = float(b.zinv-a.zinv)/float(max(N-1,1));
    float xCurrent = a.x;
    float yCurrent = a.y;
    float zCurrent = a.zinv;
    //vec2 step = vec2(b-a) / float(max(N-1,1));
    //ivec2 current( a );
    for( int i=0; i<N; ++i )
    {
        result[i].x = int(xCurrent);
        result[i].y = int(yCurrent);
        result[i].zinv = zCurrent;
        xCurrent += xStep;
        yCurrent += yStep;
        zCurrent += zStep;
    }
}

/*
 *  Interpolates ivec2
 */
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
    int N = int(result.size());
    float xStep = float(b.x-a.x)/float(max(N-1,1));
    float yStep = float(b.y-a.y)/float(max(N-1,1));
    float xCurrent = a.x;
    float yCurrent = a.y;
    //vec2 step = vec2(b-a) / float(max(N-1,1));
    //ivec2 current( a );
    for( int i=0; i<N; ++i )
    {
        result[i] = ivec2(xCurrent,yCurrent);
        xCurrent += xStep;
        yCurrent += yStep;
        //current += step;
    }
}


void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color ) {
    
    ivec2 delta = glm::abs( a - b );
    int pixels = glm::max( delta.x, delta.y ) + 1;
    vector<ivec2> line( pixels );
    Interpolate( a, b, line );
    
    for(int i = 0; i < line.size(); ++i) {
        PutPixelSDL( screen, line[i].x, line[i].y, color );
    }
}

void DrawPolygonEdges( const vector<vec3>& vertices )
{
    int V = int(vertices.size());
    // Transform each vertex from 3D world position to 2D image position:
    vector<ivec2> projectedVertices( V );
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


void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels,
                        vector<ivec2>& rightPixels )
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
        ivec2 a = vertexPixels[i];
        ivec2 b = vertexPixels[(i+1)%vertexPixels.size()];
        int edgeRows = abs( a.y - b.y ) + 1;
        vector<ivec2> edge(edgeRows);
        (a.y < b.y)?Interpolate(a, b, edge):Interpolate(b, a, edge);
        
        //DEBUG INTERPOLATION
        /*
        cout << "LINE FROM (" << edge[0].x << ", " << edge[0].y << ") TO (" << edge[edgeRows-1].x << ", " << edge[edgeRows-1].y << ")\n";
        for(ivec2 v : edge) {
            cout << "(" << v.x << ", " << v.y << ")\n"
        }
        */
        
        for(int j = 0; j < edge.size(); j++) {
            if(leftPixels[edge[j].y-yMin].x > edge[j].x)
                leftPixels[edge[j].y-yMin] = edge[j];
            if(rightPixels[edge[j].y-yMin].x < edge[j].x)
                rightPixels[edge[j].y-yMin] = edge[j];
        }
    }
}


void DrawPolygonRows( const vector<ivec2>& leftPixels,
              const vector<ivec2>& rightPixels ) {
    for(int i = 0; i < leftPixels.size(); i++) {
        int pixels = abs(leftPixels[i].x - rightPixels[i].x) + 1;
        vector<ivec2> pixelRow(pixels);
        Interpolate(leftPixels[i], rightPixels[i], pixelRow);
        for(ivec2 pixel : pixelRow)
            PutPixelSDL(screen, pixel.x, pixel.y, currentColor);
    }
}


void DrawPolygon( const vector<vec3>& vertices )
{
    int V = int(vertices.size());
    vector<ivec2> vertexPixels( V );
    for( int i=0; i<V; ++i )
        VertexShader( vertices[i], vertexPixels[i] );
    vector<ivec2> leftPixels;
    vector<ivec2> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawPolygonRows( leftPixels, rightPixels );
}