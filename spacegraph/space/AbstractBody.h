/*
 * BoxBody.h
 *
 *  Created on: Feb 13, 2010
 *      Author: seh
 */

#ifndef ABSTRACTBODY_H_
#define ABSTRACTBODY_H_

using namespace std;
#include <vector>

#include <btBulletDynamicsCommon.h>
#include <space/DefaultSpace.h>
#include <space/RigidBody.h>


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_8
#define M_PI_8     0.5 * M_PI_4
#endif

class AbstractBody {
public:
    btDynamicsWorld* dyn;

    vector<btCollisionShape*> shapes;
    vector<RigidBody*> bodies;
    vector<btTypedConstraint*> joints;

    DefaultSpace* space;

    AbstractBody() {

    }

    virtual void init(DefaultSpace* ds) {
        space = ds;
        dyn = ds->dynamicsWorld;
        init();
    }

    virtual void init() {
    }

    virtual ~AbstractBody() {

    }

    virtual void process(float deltaTime) {
    }

    virtual bool isDraggable(btVector3* touchPosLocal) {
        return true;
    }

    /** called when an object is not touched any longer */
    virtual void onUntouched() { }

    /** called when an object is touched by a pointer (Ex: mouse cursor) */
    virtual void onTouch(btVector3 *touchPosWorld, btVector3* touchPosLocal, int buttonState) { }

    int indexOfPart(btRigidBody* part) {
        for (unsigned i = 0; i < bodies.size(); i++) {
            if (bodies[i] == part)
                return i;
        }
        return -1;
    }

    int indexOfShape(btCollisionShape* shape) {
        for (unsigned i = 0; i < shapes.size(); i++) {
            if (shapes[i] == shape)
                return i;
        }
        return -1;
    }

    void addJoint(btTypedConstraint* c) {
        joints.push_back(c);
        dyn->addConstraint(c);
    }
    
    btRigidBody* createRigidShape(btScalar mass, const btVector3& pos, btCollisionShape* shape) {
        btTransform t;
        t.setIdentity();
        t.setOrigin(pos);
        return createRigidShape(mass, t, shape);
    }

    RigidBody* createRigidShape(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
        shapes.push_back(shape);
        RigidBody* rb = localCreateRigidBody(mass, startTransform, shape);
        bodies.push_back(rb);
        
        //rb->setDamping(0.05, 0.85);
        rb->setDamping(0.0, 0);

        rb->setDeactivationTime(2.0);
        //m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        rb->setSleepingThresholds(0, 0);
        //rb->setActivationState(ISLAND_SLEEPING);
        
        return rb;
    }

    //Deprecated

    RigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            shape->calculateLocalInertia(mass, localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
        RigidBody* body = new RigidBody(rbInfo);

        body->setUserPointer(this);
        dyn->addRigidBody(body);

        return body;
    }

    virtual void draw() { }
};

#endif
