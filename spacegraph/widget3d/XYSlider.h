/* 
 * File:   XYSlider.h
 * Author: seh
 *
 * Created on May 5, 2010, 8:11 PM
 */

#ifndef _XYSLIDER_H
#define	_XYSLIDER_H

#include <widget3d/Panel.h>

class XYSlider : public PanelBox {
public:
    btVector3 touchPos;
    Rect * r;

    XYSlider(btVector3* pos, btVector3 *size) : PanelBox(pos, size) {
        r = new Rect();
        front()->push_back(r);
        updateSlider();
    }

    virtual void onUntouched() {
        updateSlider();
    }

    virtual void onTouch(btVector3 *touchPosWorld, btVector3* touchPosLocal, int button) {
        if (button & (1)) {
            if (!isDraggable(touchPosLocal)) {
                touchPos = *touchPosLocal;
                updateSlider();
            }
        }
    }

    virtual bool isDraggable(btVector3* touchPosLocal) {
        return false;
    }

    virtual float getKnobSizeX() {
        return 0.05;
    }
    virtual float getKnobSizeY() {
        return 0.05;
    }

    virtual void updateSlider() {
        //*(r->pos) = btVector3(touchPos.getX() / size->getX() / 2.0, touchPos.getY() / size->getY() / 2.0, -0.5);
        *(r->pos) = btVector3(touchPos.getX() / size->getX()/2.0, touchPos.getY() / size->getY()/2.0, 0.5);
        float sx = getKnobSizeX();
        float sy = getKnobSizeY();
        float sz = 0.5 * (sx + sy);
        *(r->size) = btVector3(sx, sy, sz);
    }


};

class XSlider : public XYSlider {
public:
    XSlider(btVector3* pos, btVector3 *size) : XYSlider(pos, size) {
        updateSlider();
    }
    
    virtual float getKnobSizeX() {
        return 0.05;
    }
    virtual float getKnobSizeY() {
        return 1.0;
    }
    virtual void updateSlider() {
        *(r->pos) = btVector3(touchPos.getX() / size->getX() / 2.0, 0, 0.5);
        float sx = getKnobSizeX();
        float sy = getKnobSizeY();
        float sz = 0.5 * (sx + sy);
        *(r->size) = btVector3(sx, sy, sz);
    }
    
};

class YSlider : public XYSlider {
public:
    YSlider(btVector3* pos, btVector3 *size) : XYSlider(pos, size) {
        updateSlider();
    }

    virtual float getKnobSizeY() {
        return 0.05;
    }
    virtual float getKnobSizeX() {
        return 1.0;
    }
    virtual void updateSlider() {
        *(r->pos) = btVector3(0, touchPos.getY() / size->getY() / 2.0, 0.5);
        float sx = getKnobSizeX();
        float sy = getKnobSizeY();
        float sz = 0.5 * (sx + sy);
        *(r->size) = btVector3(sx, sy, sz);
    }

};

#endif	/* _XYSLIDER_H */

