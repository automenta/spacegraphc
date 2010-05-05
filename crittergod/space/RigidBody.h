/* 
 * File:   RigidBody.h
 * Author: seh
 *
 * Created on May 4, 2010, 6:35 PM
 */

#ifndef _RIGIDBODY_H
#define	_RIGIDBODY_H

using namespace std;
#include <stdio.h>

#include <btBulletDynamicsCommon.h>

class RigidBody : public btRigidBody {
    
public:
    btVector3* color;
    bool dragMovable, touchable, visible;

    RigidBody(const btRigidBodyConstructionInfo& constructionInfo) : btRigidBody(constructionInfo) {
        color = NULL;
        dragMovable = true;
        touchable = true;
        visible = true;
    }

    void setColor(btVector3* c) {
        color = c;
    }

    virtual ~RigidBody() {

    }
private:

};

#endif	/* _RIGIDBODY_H */

