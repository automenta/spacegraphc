#ifndef CONTAINER_H
#define CONTAINER_H

#include "widget.h"

using namespace std;

class Container : public Widget
{
	public:
		Container();
		~Container();
		void	draw();

                virtual bool onMouseMove(int x, int y);
                virtual bool onMouseButton(int button, int state, int x, int y);

		// children map, publically accessible
		map<string, Widget*>	children;
		bool	mouseOverChild( Widget** fWidget, int x, int y );

		Widget* addPanel( const string& name, Widget* nwidget );
		Widget* addText( const string& name, unsigned int posx, unsigned int posy, const string& textstring );
		Widget* addText( const string& name, const string& textstring );
		Widget* addText( const string& name, unsigned int posx, unsigned int posy, const unsigned int* uintp );
		//Widget*	addWidgetButton( const string& name, const Vector2i& pos, const Vector2i& dimensions, const string& textstring, const Vector2i& textpos, const cmdsettings& cmds, unsigned int responsetime, unsigned int minfresponsetime, unsigned int fresponseinterval );
		//Widget*	addWidgetButton( const string& name, const Vector2i& pos, const Vector2i& dimensions, const string& textstring, const cmdsettings& cmds, unsigned int responsetime, unsigned int minfresponsetime, unsigned int fresponseinterval );

        protected:
		void	drawChildren();


		void	updateAbsPosition();
	
		// children map iterator
		typedef map <string, Widget*>::const_iterator children_iterator;
		children_iterator	childit;

	private:

};

#endif
