
using namespace std;
#include <vector>

#include "GLWindow.h"
#include "SpaceProcess.h"
#include <OpenGL/GlutStuff.h>

GLWindow::GLWindow()
{
}

void GLWindow::create(const char* title, int width, int height)
{

	if( SDL_Init(SDL_INIT_VIDEO) < 0 )
	{
		cerr << "Video initialization failed: " << SDL_GetError() << endl;
		exit(1);
	}
	vidInfo = SDL_GetVideoInfo();
	
	w_bpp = vidInfo->vfmt->BitsPerPixel;
	
	if ( !vidInfo )
	{
		cerr << "Cannot get video information: " <<  SDL_GetError() << endl;
		exit(1);
	}

	vidFlags = SDL_OPENGL | SDL_GL_DOUBLEBUFFER | SDL_HWPALETTE;

	hwaccel = false;
	if( vidInfo->hw_available )
	{
		hwaccel = true;
		vidFlags |= SDL_HWSURFACE;
	}
	else
		vidFlags |= SDL_SWSURFACE;

        if( vidInfo->blit_hw != 0 )
                vidFlags |= SDL_HWACCEL;

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 4);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 4);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 4);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 4);
// 	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	SDL_WM_SetCaption(title, 0);

	w_width = width;
	w_height = height;

	n_width = width;
	n_height = height;

	//Settings::Instance()->winWidth = &w_width;
	//Settings::Instance()->winHeight = &w_height;

	//settingsfs = Settings::Instance()->getCVarPtr("fullscreen");

	fs = 0; //*settingsfs;

	if ( fs == 1 )
		surface = SDL_SetVideoMode( w_width, w_height, w_bpp, vidFlags | SDL_FULLSCREEN );
	else
		surface = SDL_SetVideoMode( w_width, w_height, w_bpp, vidFlags | SDL_RESIZABLE );

	cerr << "SDL: subsystem initalized\n";
// 	cerr << "Video " << front.width() << "x" << front.height() << "x" << int(front.getSurface()->format->BitsPerPixel) << "\n";
// 	cerr << "Render Mode: " <<  ((hwaccel) ? "Direct Rendering" : "Software Rendering")   << "\n";
// 	cerr << "Hardware Blit Acceleration: " << ((vidInfo->blit_hw) ? "Yes": "No") << "\n";
}

void GLWindow::resize()
{
	if ( w_height == 0 ) w_height = 1;
	if ( w_width == 0 ) w_width = 1;

#ifndef _WIN32
	SDL_FreeSurface(surface);
	surface = SDL_SetVideoMode( w_width, w_height, w_bpp, vidFlags | SDL_RESIZABLE );
#endif

	spacetime->resize(w_width, w_height);
}

void GLWindow::toggleFs()
{
	if ( fs )
	{
		if ( w_height == 0 ) w_height = 1;
		if ( w_width == 0 ) w_width = 1;

		SDL_FreeSurface(surface);
		n_width = w_width;
		n_height = w_height;
		//w_width = Settings::Instance()->getCVar("fsX");
		//w_height = Settings::Instance()->getCVar("fsY");

		surface = SDL_SetVideoMode( w_width, w_height, w_bpp, vidFlags | SDL_FULLSCREEN );
		Displaylists::Instance()->generateList();
		//Textprinter::Instance()->setUpFonts();
	}
	else
	{
		SDL_FreeSurface(surface);
		w_width = n_width;
		w_height = n_height;

		surface = SDL_SetVideoMode( w_width, w_height, w_bpp, vidFlags | SDL_RESIZABLE );

		Displaylists::Instance()->generateList();
		//Textprinter::Instance()->setUpFonts();
	}
}

int runFunc(void* vs) {

}

void GLWindow::start(Spacetime* s) {
	spacetime = s;

	s->init(w_width, w_height);

	bool stop = false;

	while(!stop)
	{
		while(SDL_PollEvent(&event)) 	{

	      if(event.type == SDL_VIDEORESIZE)
			{
				w_width = event.resize.w;
				w_height = event.resize.h;
	            resize();
			}
			else if(event.type == SDL_QUIT)
				stop = true;
			else {
				s->forward(&event);
			}
		}



//		if ( fs != *settingsfs )
//		{
//			fs = *settingsfs;
//			toggleFs();
//		}

		s->draw();
	}

    SDL_Quit();

//    thread = SDL_CreateThread(runFunc, this);
//    if ( thread == NULL ) {
//        fprintf(stderr, "Unable to create thread: %s\n", SDL_GetError());
//        return;
//    }

//    for ( i=0; i<5; ++i ) {
//        printf("Changing value to %d\n", i);
//        global_data = i;
//        SDL_Delay(1000);
//    }
//
//    printf("Signaling thread to quit\n");
//    global_data = -1;
//    SDL_WaitThread(thread, NULL);
}


GLWindow::~GLWindow()
{

}


/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


//glut is C code, this global gDemoApplication links glut to the C++ demo

std::vector<SpaceProcess*> processes;


static	void glutKeyboardCallback(unsigned char key, int x, int y) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->onKeyboard(key,x,y);
	}
}

static	void glutKeyboardUpCallback(unsigned char key, int x, int y) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->keyboardUpCallback(key,x,y);
	}
}

static void glutSpecialKeyboardCallback(int key, int x, int y) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->specialKeyboard(key,x,y);
	}
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y){
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->specialKeyboardUp(key,x,y);
	}
}


static void glutReshapeCallback(int w, int h) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->reshape(w,h);
	}
}

static void glutMoveAndDisplayCallback() {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->moveAndDisplay();
	}
}

static void glutMouseFuncCallback(int button, int state, int x, int y) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->onMouseButton(button,state,x,y);
	}
}


static void	glutMotionFuncCallback(int x,int y) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->onMouseMove(x,y);
	}
}


static void glutDisplayCallback(void) {
	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->draw();
	}
}

int runGLWindow(int argc, char **argv,int width,int height, const char* title, SpaceProcess* p) {
	std::vector<SpaceProcess*> vp;
	vp.push_back(p);
	runGLWindow(argc, argv, width, height, title, &vp);
}


int runGLWindow(int argc, char **argv,int width,int height,const char* title, std::vector<SpaceProcess*>* initialProcesses) {

	for (unsigned i = 0; i < initialProcesses->size(); i++)
		processes.push_back((*initialProcesses)[i]);

	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(width, height);
    glutCreateWindow(title);

#ifdef BT_USE_FREEGLUT
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

	for (unsigned i = 0; i < processes.size(); i++) {
		SpaceProcess* sp = processes[i];
		sp->preDraw();
	}

	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
	glutSpecialFunc(glutSpecialKeyboardCallback);
	glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

	glutReshapeFunc(glutReshapeCallback);
    //createMenu();
	glutIdleFunc(glutMoveAndDisplayCallback);
	glutMouseFunc(glutMouseFuncCallback);
	glutPassiveMotionFunc(glutMotionFuncCallback);
	glutMotionFunc(glutMotionFuncCallback);
	glutDisplayFunc( glutDisplayCallback );

	glutMoveAndDisplayCallback();

    glutMainLoop();
    return 0;
}


