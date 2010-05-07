/* 
 * File:   Panel.h
 * Author: seh
 *
 * Created on March 5, 2010, 5:33 PM
 */

#ifndef _PANEL_H
#define	_PANEL_H

#include <space/BoxBody.h>

class PanelBody : public BoxBody {
public:
    btQuaternion* facingNormal;
    float speed;
    
    PanelBody(btVector3* _position, btVector3* _size) : BoxBody(_position, _size), speed(0.1) {
//        facingNormal = NULL;
//        normal = btQuaternion(0,0,0,0);
//        normal.setEulerZYX(0,0,0);
    }

    void setFacing(btVector3* groundMask, btQuaternion* nextNormal) {
        facingNormal = nextNormal;
    }

    virtual void preDraw() {
        if (facingNormal!=NULL) {
            btQuaternion q = body->getWorldTransform().getRotation().slerp(*facingNormal, speed);
            body->getWorldTransform().setRotation(q);

            btVector3 groundProjection = body->getWorldTransform().getOrigin();
            groundProjection.setZ(0);
            btVector3 r = body->getWorldTransform().getOrigin().lerp(groundProjection, speed);
            body->getWorldTransform().setOrigin(r);
        }
    }

    
private:

};

#endif	/* _PANEL_H */

