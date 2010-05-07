/* 
 * File:   TextRect.h
 * Author: seh
 *
 * Created on February 22, 2010, 11:05 AM
 */

#ifndef _TEXTRECT_H
#define	_TEXTRECT_H

#include "Rect.h"

using namespace std;
#include <string>

class TextRect : public Rect {

public:
    string text;


    TextRect(const char* _text, float w=1.0, float h=1.0) : Rect(0,0,0,w,h)  {
        text = _text;
        this->fillColor = NULL;
    }

    virtual ~TextRect() {

    }
    
private:

};

#endif	/* _TEXTRECT_H */

