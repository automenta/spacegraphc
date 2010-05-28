/* 
 * File:   RunRobots.cpp
 * Author: seh
 * 
 * Created on April 25, 2010, 8:04 PM
 */
#include <iostream>
#include <string.h>

using namespace std;

#include <video/GLWindow.h>

#include <neural/Brain.h>

#include <bio/ServoHinge.h>

#include <widget2d/BrainPanel.h>
#include <widget2d/PointerPanel.h>
#include <widget2d/NeuralSignalsPanel.h>
#include <widget2d/RetinaPanel.h>
#include <widget2d/FDBrainBody.h>

#include <space/DefaultSpace.h>
#include <video/GLWindow.h>
#include <video/SpaceProcess.h>

#include <video/font/FontDemo1.h>

#include <space/AbstractBody.h>
#include <space/SpiderBody.h>
#include <space/SnakeBody.h>
#include <space/Humanoid.h>
#include <space/BoxBody.h>

#include <audio/Audio.h>

#include <widget2d/container.h>
#include <widget2d/panel.h>
#include <widget2d/button.h>
#include <widget2d/text.h>

#include <widget3d/Panel.h>
#include <widget3d/Rect.h>
#include <widget3d/TextRect.h>

#include "RunRobots.h"

void runServoDemo() {
    Audio* audio = new Audio();

    DefaultSpace* ds = new DefaultSpace(audio);

    ds->addGround(50, 5, 50, 0, -10, 0);

    float torsoThick = 0.25;
    float torsoLength = 3.0;
    float torsoWidth = 1.0;
    
    BoxBody* torso = new BoxBody(new btVector3(0,0,0), new btVector3(torsoWidth, torsoLength, torsoThick), 16);
    ds->addBody(torso);


    for (int s = 0; s < 2; s++) {
        for (int i = -2; i < 2; i++) {

            ServoHinge* sh = new ServoHinge(new btVector3(0.3, torsoThick/2.0, torsoThick/2.0), new btVector3(1.3, torsoThick/2.0, torsoThick/2.0));
            ds->addBody(sh);


            btVector3 pa(s == 0 ? torsoWidth : -torsoWidth, i, 0);
            btVector3 pb(-0.1, -0.5, -0.9);
            btVector3 aa(0,1,0);
            btVector3 ab(0,1,0);

            btHingeConstraint* c = new btHingeConstraint(*(torso->body), *(sh->fingerA), pa, pb, aa, ab, false);
            c->setAngularOnly(true);
    //        btGeneric6DofConstraint* c = new btGeneric6DofConstraint(*(torso->rb), *(sh->fingerA), localA, localB, false);
    //        c->setAngularLowerLimit(btVector3(M_PI,0,0));
    //        c->setAngularUpperLimit(btVector3(M_PI,0,0));
    //        c->setLinearLowerLimit(btVector3(0,0,0));
    //        c->setLinearUpperLimit(btVector3(0,0,0));
    //
            ds->dynamicsWorld->addConstraint(c);
        }
    }

//    for (unsigned l = 0; l < numLegs; l++) {
//        RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
//        string panelName = "retina_";
//        panelName[5] = 'a' + l;
//        ds->getFace()->addPanel(panelName, rp);
//
//        int w = 130;
//        rp->setSize(w, w);
//        rp->setPosition(w * l, 0);
//    }
//
//    BrainPanel *bp = new BrainPanel(spider->brain);
//    ds->getFace()->addPanel("brainControl", bp);
//    bp->setPosition(15, 15);

    {
//    FDBrainBody fdb(spider->brain, 25, 5, 25);
//    fdb.neuronSize = 0.1;
//    gds->addBody(&fdb);
    }

//    BrainInsPanel bip(spider->brain, 100);
//    ds->getFace()->addPanel("brainIns", &bip);
//    bip.setPosition(600, 600);
//    bip.setSize(100, 600);

//    BrainOutsPanel bop(spider->brain, 100);
//    ds->getFace()->addPanel("brainOuts", &bop);
//    bop.setPosition(800, 600);
//    bop.setSize(100, 600);

    runGLWindow(0, NULL, 1024, 800, "", ds);

    delete audio;
    
}
