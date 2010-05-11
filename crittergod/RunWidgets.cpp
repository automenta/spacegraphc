/* 
 * File:   RunWidgets.cpp
 * Author: seh
 * 
 * Created on February 23, 2010, 3:59 PM
 */

#include <iostream>
#include <string.h>

using namespace std;

#include <video/GLWindow.h>

#include <space/DefaultSpace.h>
#include <space/AbstractBody.h>


#include <audio/Audio.h>

#include <objects/BrainPanel.h>
#include <objects/PointerPanel.h>
#include <objects/NeuralSignalsPanel.h>
#include <objects/RetinaPanel.h>
#include <objects/FDBrainBody.h>

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
//#include <widget2d/button.h>
#include <widget2d/text.h>

#include <widget3d/Panel.h>
#include <widget3d/Rect.h>
#include <widget3d/Button.h>
#include <widget3d/XYSlider.h>
#include <widget3d/TextRect.h>

#include "RunWidgets.h"

class StrobeBox : public BoxBody {
    float t;
    btVector3 color;
    float frequency;

public:

    StrobeBox(btVector3* _position, btVector3* _size, btVector3 _color, float _frequency) : BoxBody(_position, _size) {
        t = 0;
        color = _color;
        frequency = _frequency;
    }

    virtual void process(float dt) {
        t += dt;
        float i = 0.5 + 0.5 * sin(t * M_PI * frequency);
        body->setColor(color * i);
    }

};

class SliderDemoPanel : public PanelBox {
public:
    btHingeConstraint* h1;
    btHingeConstraint* h2;
    btHingeConstraint* h3;

    class HingeButton : public ButtonBox {
    public:
        SliderDemoPanel* sdp;
        int numClicks;

        HingeButton(SliderDemoPanel* _sdp, btVector3* _position, btVector3* _size) : ButtonBox(_position, _size) {
            sdp = _sdp;
            numClicks = 0;
        }

        virtual void onClicked() {
            float angle = numClicks % 2 ? 0 : M_PI_2;

            sdp->h1->setLimit(angle, angle, 0.99, 0.99);
            sdp->h2->setLimit(angle, angle, 0.99, 0.99);
            sdp->h3->setLimit(angle, angle, 0.99, 0.99);
            numClicks++;
        }

    };

    SliderDemoPanel(btVector3* _position, btVector3* _size) : PanelBox(_position, _size) {

    }

    virtual void init() {
        BoxBody::init();

        float sd = 0.05;

        HingeButton* b = new HingeButton(this, new btVector3(-4, -4, -4), new btVector3(3, 0.75, sd));
        space->addBody(b);

        XSlider* trb3 = new XSlider(new btVector3(-4, -4, -4), new btVector3(3, 0.75, sd));
        space->addBody(trb3);

        YSlider* trb4 = new YSlider(new btVector3(-4, -4, -4), new btVector3(0.75, 3, sd));
        space->addBody(trb4);

        XYSlider* trb = new XYSlider(new btVector3(-4, -4, -4), new btVector3(2.5, 2.5, sd));
        space->addBody(trb);
        trb->body->setColor(btVector3(0.3, 0.3, 0.3));

        attachFront(b, btVector3(0, 4.0, 0.6));
        h3 = attachFront(trb4, btVector3(-4.0, 0, 0.6));
        h2 = attachFront(trb3, btVector3(0, -4.0, 0.6));
        h1 = attachFront(trb, btVector3(0, 0, 0.6));

    }

};

void runWidgets3D() {
    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    *(ds->getBackgroundColor()) = btVector3(0.2, 0.2, 0.2);

    double d = 0.4;

    PanelBox* bc = new PanelBox(new btVector3(0, 0.5, 0.25), new btVector3(6, 4, d));
    {
        float blD = 0.1;
        TextRect* ul = new TextRect("BL(0.1)");
        ul->span(-0.5, -0.5, -0.5 + blD, -0.5 + blD);
        bc->front()->push_back(ul);

        TextRect* ul2 = new TextRect("BL(0.1)");
        ul2->span(-0.5 + blD, -0.5, -0.5 + blD * 2, -0.5 + blD);
        bc->front()->push_back(ul2);

        TextRect * br = new TextRect("UR");
        br->span(0.5, 0.5, 0.2, 0.2);
        bc->front()->push_back(br);

        TextRect * ur = new TextRect("UL");
        ur->span(-0.5, 0.5, -0.35, 0.35);
        bc->front()->push_back(ur);

        Rect* r1 = new Rect(0.4, -0.4, 0, 0.2, 0.2);
        *(r1->fillColor) = btVector3(0, 1, 0);
        bc->front()->push_back(r1);

    }
    ds->addBody(bc);

    for (int i = 0; i < 5; i++) {
        PanelBox* bb = new PanelBox(new btVector3(0, 0, 1.0), new btVector3(2, 1, d));
        bb->front()->push_back(new TextRect("!@#$%", 0.3, 0.3));
        ds->addBody(bb);
    }

    PanelBox* ba = new PanelBox(new btVector3(0, 0, 0.5), new btVector3(3, 2, d));
    {
        float d = 0.0;

        Rect* r1 = new Rect(-0.3, -0.3, d, 0.1, 0.2);
        *(r1->fillColor) = btVector3(0, 1, 0);
        ba->front()->push_back(r1);

        Rect* r2 = new Rect(0.3, -0.3, d, 0.2, 0.2);
        *(r2->fillColor) = btVector3(0, 0, 1);
        ba->front()->push_back(r2);

        Rect* r3 = new Rect(0.3, 0.3, d, 0.2, 0.2);
        *(r3->fillColor) = btVector3(1, 0, 0);
        ba->front()->push_back(r3);

        Rect* r4 = new Rect(-0.3, 0.3, d, 0.2, 0.2);
        ba->front()->push_back(r4);

    }
    ds->addBody(ba);

    PanelBox* cx = new PanelBox(new btVector3(0, 0, 0.5), new btVector3(3, 2, d));
    {
        float d = 0.1;

        float w = 0.1;
        float h = 0.1;
        for (float x = -0.5 + w / 2.0; x < 0.5 - w / 2.0; x += w) {
            for (float y = -0.5 + h / 2.0; y < 0.5 - h / 2.0; y += h) {
                Rect* r1 = new Rect(x, y, d, w, h);
                float r = x + 0.5;
                float g = y + 0.5;
                float b = 0.0;
                *(r1->fillColor) = btVector3(r, g, b);
                cx->front()->push_back(r1);
            }

        }


    }
    ds->addBody(cx);

    {
        int numLegs = 3;
        vector<btScalar>* legLengths = new vector<btScalar > ();
        vector<btScalar>* legRadii = new vector<btScalar > ();
        legLengths->push_back(0.8);
        legRadii->push_back(0.1);
        legLengths->push_back(0.5);
        legRadii->push_back(0.10);
        legLengths->push_back(0.5);
        legRadii->push_back(0.08);
        SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, legRadii, btVector3(0, 10, 4), 48);
        ds->addBody(spider);
        spider->setDamping(0.5);

        for (unsigned l = 0; l < numLegs; l++) {
            RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
            string panelName = "retina_";
            panelName[5] = 'a' + l;
            ds->getFace()->addPanel(panelName, rp);

            int w = 110;
            rp->setSize(w, w);
            rp->setPosition(w * l, 0);
        }


    }

    {
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 0, 0), 2));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 1.0, 0), 3));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 1.0, 0), 3.5));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 0, 1.0), 1.5));
    }

    {
        SliderDemoPanel* scx = new SliderDemoPanel(new btVector3(-2, -2, 0.5), new btVector3(4, 4, d));
        btQuaternion* d = new btQuaternion(0, 0, 0, 0);
        d->setEulerZYX(0, 0, 0);
        scx->setFacing(new btVector3(1, 1, 0), d);

        ds->addBody(scx);


        //    XYSlider* trb2 = new XYSlider(new btVector3(-4, -4, -4), new btVector3(3, 3, d));
        //    btQuaternion forward;
        //    forward.setEuler(-M_PI_2, 0, 0);
        //    trb2->setFront(forward);
        //    ds->addBody(trb2);

    }

    //    {
    //        Humanoid* h = new Humanoid(btVector3(0, 8, 0));
    //        ds->addBody(h);
    //
    //        RetinaPanel* rp2 = new RetinaPanel(h->lhRetina);
    //        ds->getFace()->addPanel("lhRetina", rp2);
    //        rp2->setSize(150, 150);
    //        rp2->setPosition(600, 600);
    //
    //    }


    runGLWindow(0, NULL, 1024, 800, "SpaceGraph-C", ds);

}

#define brainNeurons 8000
#define armNeurons 1000
#define armPixels 8

class ArmedPanelBox : public PanelBox {
    float rx;
    Brain* brain;

public:

    ArmedPanelBox(btVector3* pos, btVector3* size) : PanelBox(pos, size) {
        rx = 0.0;
    }

    virtual void init() {
        mass = 0.1;

        PanelBox::init();

        brain = new Brain();

        float sx = size->getX();
        float sy = size->getY();
        float pad = 0.3;
        addArm(btVector3(-sx, 0, 0), btVector3(-pad, 0, 0), 0);
        addArm(btVector3(sx, 0, 0), btVector3(pad, 0, 0), M_PI);
        addArm(btVector3(0, -sy, 0), btVector3(0, -pad, 0), -M_PI / 2.0);
        addArm(btVector3(0, sy, 0), btVector3(0, pad, 0), M_PI / 2.0);

        for (unsigned n = 0; n < brainNeurons; n++)
            addNeuron();
        brain->printSummary();

    }

    void addNeuron() {
        unsigned minSynapsesPerNeuron = 32;
        unsigned maxSynapsesPerNeuron = 64;
        float percentInhibitoryNeuron = 0.5f;
        float percentInputSynapse = 0.25f;
        float percentOutputNeuron = 0.1f;
        float percentInhibitorySynapse = 0.5f;
        float minSynapseWeight = 0.001f;
        float maxSynapseWeight = 2.0f;
        float neuronPotentialDecay = 0.98f;
        brain->wireRandomly(minSynapsesPerNeuron, maxSynapsesPerNeuron,
                percentInhibitoryNeuron, percentInputSynapse, percentOutputNeuron, percentInhibitorySynapse,
                minSynapseWeight, maxSynapseWeight, neuronPotentialDecay);
    }

    void addArm(btVector3 pos, btVector3 armPos, float headPhase) {
        int numLegs = 1;
        vector<btScalar>* legLengths = new vector<btScalar > ();
        vector<btScalar>* legRadii = new vector<btScalar > ();
        legLengths->push_back(0.9);
        legRadii->push_back(0.15);
        legLengths->push_back(0.8);
        legRadii->push_back(0.1);
        legLengths->push_back(0.7);
        legRadii->push_back(0.1);
        legLengths->push_back(0.6);
        legRadii->push_back(0.1);
        legLengths->push_back(0.5);
        legRadii->push_back(0.10);
        SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, legRadii, btVector3(0, 10, 4), armPixels, armNeurons);
        spider->setHeadPhase(headPhase);
        space->addBody(spider);
        spider->setDamping(0.5);

        brain->addInputs(&(spider->brain->ins));
        brain->addOutputs(&(spider->brain->outs));

        for (unsigned l = 0; l < numLegs; l++) {
            RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
            string panelName = "retina_";
            panelName[5] = 'a' + l;
            panelName[4] = rand() % 26 + 'a';
            space->getFace()->addPanel(panelName, rp);

            int w = 64;
            rp->setSize(w, w);
            rp->setPosition(rx, 0);
            rx += w;
        }

        attachFrontPoint(spider->bodies[0], pos, armPos);

    }


};

class InputLED : public BoxBody {
    InNeuron* input;
public:

    InputLED(InNeuron* i, btVector3* pos, btVector3* size) : BoxBody(pos, size) {
        input = i;
    }

    virtual void process(float dt) {
        float f = input->getInput();
        if (f < 0) {
            body->setColor(btVector3(0, 0, -f));
        } else {
            body->setColor(btVector3(0, f, 0));
        }
    }
};

class OutputLED : public BoxBody {
    OutNeuron* output;
public:

    OutputLED(OutNeuron* o, btVector3* pos, btVector3* size) : BoxBody(pos, size) {
        output = o;
    }

    virtual void process(float dt) {
        float f = output->getOutput();
        float s;
        if (f < 0) {
            body->setColor(btVector3(0.0, 0.0, 0.5 + 0.5 * -f));
            s = -f;
        } else {
            body->setColor(btVector3(0.0, 0.5 + 0.5 * f, 0.0));
            s = f;
        }
        body->getCollisionShape()->setLocalScaling(btVector3(0.5 + s,0.5 + s, 0.5 + s));
    }
};

void runLiveWidgets() {
    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    ds->setTexturing(false);
    //ds->addGround(15, 5, 15, 0, -10, 0);

    *(ds->getBackgroundColor()) = btVector3(0.2, 0.2, 0.2);

    ArmedPanelBox* b = new ArmedPanelBox(new btVector3(0, 0.5, 0.25), new btVector3(1.0, 1.0, 0.2));
    ds->addBody(b);

    {
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 0, 0), 2));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 1.0, 0), 3));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 1.0, 0), 3.5));
        ds->addBody(new StrobeBox(new btVector3(0, 1, 0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 0, 1.0), 1.5));
    }

    runGLWindow(0, NULL, 1024, 800, "SpaceGraph-C", ds);

}

void runArm() {
    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    *(ds->getBackgroundColor()) = btVector3(1.0, 0.7, 0.1);


    //    {
    //        ds->addBody(new StrobeBox(new btVector3(0,1,0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 0, 0), 2));
    //        ds->addBody(new StrobeBox(new btVector3(0,1,0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 1.0, 0), 3));
    //        ds->addBody(new StrobeBox(new btVector3(0,1,0), new btVector3(1.5, 1.5, 1.5), btVector3(1.0, 1.0, 0), 3.5));
    //        ds->addBody(new StrobeBox(new btVector3(0,1,0), new btVector3(1.5, 1.5, 1.5), btVector3(0, 0, 1.0), 1.5));
    //    }


    int numLegs = 3;
    vector<btScalar>* legLengths = new vector<btScalar > ();
    vector<btScalar>* legRadii = new vector<btScalar > ();
    legLengths->push_back(0.8);
    legRadii->push_back(0.1);
    legLengths->push_back(0.5);
    legRadii->push_back(0.1);
    legLengths->push_back(0.5);
    legRadii->push_back(0.1);
    legLengths->push_back(0.5);
    legRadii->push_back(0.1);
    legLengths->push_back(0.5);
    legRadii->push_back(0.1);
    legLengths->push_back(0.5);
    legRadii->push_back(0.08);
    legLengths->push_back(0.5);
    legRadii->push_back(0.08);
    legLengths->push_back(0.5);
    legRadii->push_back(0.08);
    legLengths->push_back(0.3);
    legRadii->push_back(0.08);
    SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, legRadii, btVector3(0, 0, 0), 32, 36000);
    ds->addBody(spider);
    spider->setDamping(0.8);

    for (unsigned l = 0; l < numLegs; l++) {
        RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
        string panelName = "retina_";
        panelName[5] = 'a' + l;
        ds->getFace()->addPanel(panelName, rp);

        int w = 130;
        rp->setSize(w, w);
        rp->setPosition(w * l, 0);
    }

    float x = 0;
    for (unsigned i = spider->kinestheticInputsStart; i < spider->kinestheticInputsStart + 6; i++) {
        ds->addBody(new InputLED(spider->brain->ins[i], new btVector3(x - 5, 0, 0), new btVector3(1.0, 0.1, 1.0)));
        x += 2.0;
    }

    unsigned o = 0;
    float r = 20.0;
    for (float z = -30; z < 30; z+=6) {
        for (float a = 0; a < M_PI*2; a+=0.6) {
            float x = cos(a)*r;
            float y = sin(a)*r;
            
            //ds->addBody(new StrobeBox(new btVector3(x,y,z), new btVector3(1.5, 1.5, 1.5), btVector3(0.5 + (x/2.0), 0.5 + (y/2.0), 0), 2));
            if (o < spider->brain->outs.size()) {
                ds->addBody(new OutputLED(spider->brain->outs[o], new btVector3(x, y, z), new btVector3(1.0, 1.0, 1.0)));
                o++;
            }
        }
    }

    BrainPanel *bp = new BrainPanel(spider->brain);
    ds->getFace()->addPanel("brainControl", bp);
    bp->setPosition(15, 150);

    BrainOutsPanel* bop = new BrainOutsPanel(spider->brain, 50);
    ds->getFace()->addPanel("brainOuts", bop);
    bop->setSize(200, 800);
    bop->setPosition(400, 400);

    runGLWindow(0, NULL, 1024, 800, "SpaceGraph-C", ds);

}