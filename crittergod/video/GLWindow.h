#ifndef GLWINDOW_H
#define GLWINDOW_H

using namespace std;
#include <vector>

#include <SDL/SDL.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/glx.h>
//#include <SDL/SDL_opengl.h>
#include <SDL/SDL_thread.h>

#include "displaylists.h"
#include "Spacetime.h"
#include "SpaceProcess.h"

class GLWindow {

	public:
		GLWindow();
		~GLWindow();

		//	create the XFree86 window, with a GLX context.
		void create(const char* title, int width, int height);
		//	Destroy window and OpenGL Context, close the Display
// 		void destroy();
		//	Main loop for the program.
		void start(Spacetime* s);


	private:
		int run(void* unused);
		//	Resize Window

		Spacetime* spacetime;	//current spacetime

		void resize();
		void toggleFs();
		unsigned int w_bpp;		// Bits Per Pixel. With XFree86, highest = 24
		int w_width;
		int w_height;
		int n_width;
		int n_height;
		unsigned int fs;
		SDL_Surface* surface;
		const SDL_VideoInfo* vidInfo;
		int vidFlags;
		bool hwaccel;
		const unsigned int* settingsfs;
		int mousex;
		int mousey;
		SDL_Event event;
};

int runGLWindow(int argc, char **argv,int width,int height, const char* title, SpaceProcess* p);
int runGLWindow(int argc, char **argv,int width,int height,const char* title, std::vector<SpaceProcess*>* initialProcesses);

#endif	// GLWINDOW_H
