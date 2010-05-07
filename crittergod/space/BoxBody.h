/*
 * BoxBody.h
 *
 *  Created on: Feb 13, 2010
 *      Author: seh
 */

#ifndef BOXBODY_H_
#define BOXBODY_H_

using namespace std;
#include <iostream>
#include <list>
#include <map>

#include "DefaultSpace.h"
#include "AbstractBody.h"

#include "../widget3d/Rect.h"
#include "../widget3d/TextRect.h"

#include <FTGL/ftgl.h>

typedef list<Rect*> RectList;

static FTFont *font;

#define DAMPENING 0.8

class BoxBody : public AbstractBody {
public:

    btVector3* position;
    btVector3* size;
    list<Rect*> frontRects;
    RigidBody* body;
    btScalar mass;

    BoxBody(btVector3* _position, btVector3* _size, btScalar _mass=1.0) : AbstractBody(), position(_position), size(_size), mass(_mass) {

    }

    virtual void init() {
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(*position);

//        btVector3 halfSize(size->getX()/2.0, size->getY()/2.0, size->getZ()/2.0);
        body = createRigidShape(btScalar(mass), transform, new btBoxShape(*size));
        body->setDamping(DAMPENING, DAMPENING);

        char const *file = "media/font/OCRA.ttf";

        if (font == NULL) {
            font = new FTPolygonFont(file);
            //font = new FTOutlineFont(file);
            //font = new FTBufferFont(file);
            //font = new FTBitmapFont(file);

            if (font->Error()) {
                fprintf(stderr, "could not load font `%s'\n", file);
                return;
            }

            font->FaceSize(1);
            font->Depth(10);
            font->CharMap(ft_encoding_unicode);
        }

    }

    list<Rect*>* front() {
        return &frontRects;
    }

    virtual void preDraw() {
    }

    virtual void drawFront() {
        float sx = size->getX();
        float sy = size->getY();
        float sz = size->getZ();

        for (RectList::const_iterator ri = frontRects.begin(); ri != frontRects.end(); ++ri) {
            glPushMatrix();
            Rect *r = (*ri);

            btVector3* rpos = r->pos;
            btVector3* rsize = r->size;

            float d = 0.1;

            //glScalef(0.5 * rsize->getX() * sx, 0.5 * rsize->getY() * sy, 1.0 / 2.0);
            //glTranslatef(-0.5 + rpos->getX()*sx, -0.5 + rpos->getY()*sy, -d * 3);

            float x1 = (rpos->getX() - rsize->getX()/2.0) * sx * 2.0;
            float x2 = (rpos->getX() + rsize->getX()/2.0) * sx * 2.0;
            float y1 = (rpos->getY() - rsize->getY()/2.0) * sy * 2.0;
            float y2 = (rpos->getY() + rsize->getY()/2.0) * sy * 2.0;
            float xw = fabs(x2 - x1);
            float yh = fabs(y2 - y1);
            float z = (rpos->getZ() + 1.0)* sz;
            
            btVector3* bgColor = r->fillColor;


            if (bgColor != NULL) {
                glBegin(GL_QUADS);
                glColor4f(bgColor->getX(), bgColor->getY(), bgColor->getZ(), 1.0);

                glVertex3f(x2, y1, z + d);
                glVertex3f(x2, y2, z + d);
                glVertex3f(x1, y2, z + d);
                glVertex3f(x1, y1, z + d);

                glEnd();
            }

            TextRect* tr = dynamic_cast<TextRect*> (r);
            if (tr != NULL) {
                glBegin(GL_QUADS);
                glColor4f(0.2,0,0, 1.0);
                glVertex3f(x2, y1, z + d);
                glVertex3f(x2, y2, z + d);
                glVertex3f(x1, y2, z + d);
                glVertex3f(x1, y1, z + d);
                glEnd();

                glPushMatrix();
                //glMaterialfv(GL_FRONT, GL_AMBIENT, front_ambient);
                //glColorMaterial(GL_FRONT, GL_DIFFUSE);
                //glEnable(GL_TEXTURE);

                FTBBox b = font->BBox(tr->text.c_str());
                float cw = fabs(b.Upper().X() - b.Lower().X()) / rsize->getX();
                float ch = fabs(b.Upper().Y() - b.Lower().Y()) / rsize->getY();
                glTranslatef((x1+x2)/2.0 - sx/cw/2.0, (y1+y2)/2.0, (rpos->getZ() + 1.0)* sz + d*2);
                glScalef(1*sx/cw, 1*sy/ch, 1);

                glColor4f(1.0, 1.0, 1.0, 1.0);
                font->Render(tr->text.c_str());
                glPopMatrix();

            }

            glPopMatrix();
        }

    }

    void attachFront(BoxBody* bb, btVector3 pos) {
//        btTransform t1;
//        t1.setOrigin(pos);
//        btTransform t2;
//        //t2.setOrigin(btVector3(0,0,-bb->size->getZ()));
//        t2.setOrigin(pos);
//        btGeneric6DofConstraint* p = new btGeneric6DofConstraint(*(rb), *(bb->rb), t1, t2, false);


        btVector3 pivA = pos;
        pivA -= btVector3(bb->size->getX(), 0, 0);
        btVector3 pivB = btVector3(-bb->size->getX()*1.1, 0, 0);
        btVector3 axA = btVector3(0,1,0);
        btHingeConstraint* p = new btHingeConstraint(*(body), *(bb->body), pivA, pivB, axA, axA, false);
        p->setLimit(0, 0, 0.5, 0.5);

        //btPoint2PointConstraint* p = new btPoint2PointConstraint((*rb), *(bb->rb), pos, btVector3(0,0,-bb->size->getZ()));
        dyn->addConstraint(p, true);
    }

    virtual void draw() {
    }

    virtual ~BoxBody();
};

#endif /* BOXBODY_H_ */
