// Introduction lab that covers:
// * C++
// * SDL
// * 2D graphics
// * Plotting pixels
// * Video memory
// * Color representation
// * Linear interpolation
// * glm::vec3 and std::vector

#include "SDL.h"
#include <iostream>
#include "glm/glm.hpp"
#include <vector>
#include "SDLauxiliary.h"

using namespace std;
using glm::vec3;

// --------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
SDL_Surface* screen;
vector<vec3> stars(1000);
float focalLength = SCREEN_HEIGHT/2;
int t;
float velocity = 0.0002;

// --------------------------------------------------------
// FUNCTION DECLARATIONS

void Draw();
void Interpolate( float a, float b, vector<float>& result );
void Interpolate( vec3 a, vec3 b, vector<vec3>& result );
void Projection( const vec3 &coord, int &u, int &v );
void Update();

// --------------------------------------------------------
// FUNCTION DEFINITIONS

int main( int argc, char* argv[] )
{
    
    // initialize starts
    for(int i = 0; i < 1000; i++)
    {
        float x = float(rand()/ float(RAND_MAX));
        float y = float(rand()/ float(RAND_MAX));
        float z = float(rand()/ float(RAND_MAX));
        int a = rand() % 2;
        int b = rand() % 2;
        (a == 0) ? x = -x : x;
        (b == 0) ? y = -y : y;
        
        stars[i].x = x;
        stars[i].y = y;
        stars[i].z = z;
    }
    
	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	while( NoQuitMessageSDL() )
	{
        Update();
		Draw();
	}
	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}



void Draw()
{
    SDL_FillRect( screen, 0, 0 );
    if( SDL_MUSTLOCK(screen) )
        SDL_LockSurface(screen);
    for( size_t s=0; s<stars.size(); ++s )
    {
        // Add code for projecting and drawing each star
        int u, v;
        Projection(stars[s], u, v);
        vec3 color = 0.2f * vec3(1,1,1) / (stars[s].z*stars[s].z);
        PutPixelSDL( screen, u, v, color );

    }
    if( SDL_MUSTLOCK(screen) )
        SDL_UnlockSurface(screen);
    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

void Projection(const vec3 &coord, int &u, int &v) {
    u = focalLength*(coord.x/coord.z) + (SCREEN_WIDTH/2);
    v = focalLength*(coord.y/coord.z) + (SCREEN_HEIGHT/2);
}
 

void Update()
{
    int t2 = SDL_GetTicks();
    float dt = float(t2-t);
    t = t2;

    
    for( int s=0; s<stars.size(); ++s )
    {
        // Add code for update of stars
        if( stars[s].z <= 0 )
            stars[s].z += 1;
        if( stars[s].z > 1 )
            stars[s].z -= 1;
        stars[s].z = stars[s].z-velocity*dt;
    }
}


/* PART 1
void Draw()
{
    vec3 topLeft(1,0,0); // red
    vec3 topRight(0,0,1); // blue
    vec3 bottomLeft(1,1,0); // green
    vec3 bottomRight(0,1,0); // yellow
    vector<vec3> leftSide( SCREEN_HEIGHT );
    vector<vec3> rightSide( SCREEN_HEIGHT );
    Interpolate( topLeft, bottomLeft, leftSide );
    Interpolate( topRight, bottomRight, rightSide );

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
        vector<vec3> rowColor( SCREEN_WIDTH );
        Interpolate(leftSide[y], rightSide[y], rowColor);
        
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
			// vec3 color(0,0,0); // black
            // vec3 color(1,1,1); // White
            // vec3 color(1,0,0); // red
            // vec3 color(0,1,0); // green
            // vec3 color(0,0,1); // blue
           
			PutPixelSDL( screen, x, y, rowColor[x] );
		}
	}

	if( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}
 */


void Interpolate( float a, float b, vector<float>& result )
{
    if(result.size()==1)
        result[0] = (a+b)/2;
        
    for(int i=0; i<result.size(); i++)
    {
        result[i]=a+((b-a)/(result.size()-1))*i;
    }
}

void Interpolate( vec3 a, vec3 b, vector<vec3>& result )
{
    if(result.size()==1)
        result[0] = (a+b)*0.5f;
    
    for(int i=0; i<result.size(); i++)
    {
        result[i].x=a.x+((b.x-a.x)/(result.size()-1))*i;
        result[i].y=a.y+((b.y-a.y)/(result.size()-1))*i;
        result[i].z=a.z+((b.z-a.z)/(result.size()-1))*i;
    }
    
}