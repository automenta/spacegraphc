/* 
 * File:   Button.h
 * Author: seh
 *
 * Created on February 22, 2010, 10:33 AM
 */

#ifndef _BUTTON_H
#define	_BUTTON_H

#include <widget3d/Panel.h>

class ButtonBox : public PanelBox {
    bool pressed, lastPressed;

public:
    ButtonBox(btVector3* _position, btVector3* _size) : PanelBox(_position, _size) {
        pressed = lastPressed = false;
    }

    virtual void init() {
        PanelBox::init();
        updateButton();
    }

    virtual void onUntouched() {
        pressed = false;
        updateButton();
    }

    virtual void onTouch(btVector3 *touchPosWorld, btVector3* touchPosLocal, int button) {
        pressed = (button & (1));

        if ((!pressed) && (lastPressed)) {
            onClicked();
        }

        updateButton();
        
        lastPressed = pressed;
    }

    virtual void updateButton() {
        if (pressed) {
            body->setColor(btVector3(0.5, 0.5, 0.5));
        }
        else {
            body->setColor(btVector3(0.8, 0.8, 0.8));
        }
    }

    virtual void onClicked() { }
    
    virtual bool isDraggable(btVector3* touchPosLocal) {
        return false;
    }

};

//class Button {
//public:
//    Button();
//    Button(const Button& orig);
//    virtual ~Button();
//private:
//
//};

#endif	/* _BUTTON_H */

