/* 
 * File:   XYSlider.h
 * Author: seh
 *
 * Created on May 5, 2010, 8:11 PM
 */

#ifndef _XYSLIDER_H
#define	_XYSLIDER_H

#include <widget3d/Panel.h>

class XYSlider : public PanelBody {
    btVector3 touchPos;
    Rect * r;
public:

    XYSlider(btVector3* pos, btVector3 *size) : PanelBody(pos, size) {
        r = new Rect();
        front()->push_back(r);
        update();
    }

    virtual void onTouch(btVector3 *touchPosWorld, btVector3* touchPosLocal, int button) {
        if (button & (1)) {
            if (!isDraggable(touchPosLocal)) {
                touchPos = *touchPosLocal;
                update();
            }
        }
    }

    virtual bool isDraggable(btVector3* touchPosLocal) {
        float x = touchPosLocal->x();
        float y = touchPosLocal->y();
        float z = touchPosLocal->z();
        float frontFaceZ = size->getZ();
        //printf("%f %f %f : %f %f : %f\n", x, y, z, getMarginX(), getMarginY(), frontFaceZ);
        if ((fabs(z - frontFaceZ) < size->getZ()/10.0) && (z > 0)) //on or near the front face (not back or sides)
            if (fabs(x) <= getMarginX())
                if (fabs(y) <= getMarginY())
                    return false;
        return true;
    }

    virtual float getMarginPercent() {
        return 0.25;
    }

    virtual float getMarginX() {
        return 0.5 * size->getX() * (1.0 - getMarginPercent());
    }

    virtual float getMarginY() {
        return 0.5 * size->getY() * (1.0 - getMarginPercent());
    }

    virtual float getKnobSize() {
        return 0.05;
    }

    void update() {
        *(r->pos) = btVector3(touchPos.getX() / size->getX() / 2.0, touchPos.getY() / size->getY() / 2.0, -0.5);
        float s = getKnobSize();
        *(r->size) = btVector3(s, s, s);
    }


};


#endif	/* _XYSLIDER_H */
