/* 
 * File:   RunSims.cpp
 * Author: seh
 * 
 * Created on February 23, 2010, 3:57 PM
 */

#include "RunSims.h"

#include <iostream>
#include <string.h>

using namespace std;

#include <video/GLWindow.h>

#include <neural/Brain.h>

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
#include <widget2d/button.h>
#include <widget2d/text.h>

#include <widget3d/Panel.h>
#include <widget3d/Rect.h>
#include <widget3d/XYSlider.h>
#include <widget3d/TextRect.h>

void runSim() {
    Audio* audio = new Audio();

    DefaultSpace* ds = new DefaultSpace(audio);

    {
        Container* c = ds->getFace();

        static Panel* panel = new Panel();
        panel->setSize(200, 200);
        panel->setPosition(50, 50);

        Button* b = new Button("Clear");

        class TestButtonModel : public ButtonModel {

            virtual void onChange(Button* b, int state) {
                cout << "button pressed : " << state << "\n";
                panel->bgColor[0] = 1.0;
            }
        };
        b->setModel(new TestButtonModel());
        b->setSize(65, 25);
        b->setPosition(5, 5);

        panel->addPanel("b", b);


        c->addPanel("a", panel);


    }


    {
        ds->addGround(50, 5, 50, 0, -10, 0);

        for (unsigned i = 2; i < 5; i++) {
            SnakeBody* snake1 = new SnakeBody(btVector3(0, 0, 0), i * 2);
            ds->addBody(snake1);

        }

        for (unsigned i = 0; i < 5; i++) {
            SpiderBody* spider = new SpiderBody(1 + i, btVector3(0, 0, 5 + i * 2));
            ds->addBody(spider);
        }
    }
    //
    //    BoxBody* box = new BoxBody(btVector3(3, 3, 3), btVector3(1, 0.5, 0.5));
    //    ds->addBody(box);

    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

    delete audio;

}

void runBrainDemo() {
    Brain* b = new Brain();
    b->printSummary();

    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    ds->setShadows(false);
    ds->setTexturing(false);

    FDBrainBody fdb(b, 100, 100, 100);
    ds->addBody(&fdb);

    BrainPanel *bp = new BrainPanel(b);
    ds->getFace()->addPanel("brainControl", bp);
    bp->setPosition(15, 15);

    FDBrainPanel fdo(&fdb);
    ds->getFace()->addPanel("fdPanel", &fdo);
    fdo.span(350, 500, 450, 600);

    BrainInsPanel bip(b, 100);
    ds->getFace()->addPanel("brainIns", &bip);
    bip.setPosition(600, 600);

    BrainOutsPanel bop(b, 100);
    ds->getFace()->addPanel("brainOuts", &bop);
    bop.setPosition(800, 600);


    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

}

void runCritterLab() {

    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    Brain *b;

    ds->addGround(50, 50, 5, 0, -10, 0);

    {
        SpiderBody* spider = new SpiderBody(3, btVector3(0, 0, 5 + 1 * 2));
        ds->addBody(spider);
    }


    SnakeBody* snake1 = new SnakeBody(btVector3(0, 0, 0), 6);
    ds->addBody(snake1);

    b = snake1->brain;

    ds->getFace()->addPanel("brainControl", new BrainPanel(b));

    //    BrainInsPanel bip(b, 50);
    //    ds->getFace()->addWidgetPanel("brainIns", &bip);
    //    bip.setPosition(300, 300);
    //    bip.setSize(300, 300);

    BrainOutsPanel bop(b, 50);
    ds->getFace()->addPanel("brainOuts", &bop);
    bop.setPosition(400, 400);

    RetinaPanel rp(snake1->eyeRetina);
    ds->getFace()->addPanel("retina", &rp);
    rp.setSize(150, 150);
    rp.setPosition(300, 300);

    ds->addBody(new BoxBody(new btVector3(1, 0.5, 0.5), new btVector3(3, 3, 3)));
    ds->addBody(new BoxBody(new btVector3(0, 2.5, 0.5), new btVector3(3, 2, 3)));
    ds->addBody(new BoxBody(new btVector3(1, 0.5, 0.8), new btVector3(2, 3, 1)));
    ds->addBody(new BoxBody(new btVector3(3, 0.5, 1.5), new btVector3(2, 2, 1)));

    runGLWindow(0, NULL, 1224, 1000, VERSION, ds);

}

void runInteractionDemo() {

    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    ds->addGround(30, 30, 5, 0, -10, 0);

    {
        SpiderBody* spider = new SpiderBody(3, btVector3(0, 0, 5 + 1 * 2));
        ds->addBody(spider);

        ds->addBody(new BoxBody(new btVector3(1, 0.5, 0.5), new btVector3(3, 3, 3)));
        ds->addBody(new BoxBody(new btVector3(0, 2.5, 0.5), new btVector3(3, 2, 3)));
        ds->addBody(new BoxBody(new btVector3(1, 0.5, 0.8), new btVector3(2, 3, 1)));
        ds->addBody(new BoxBody(new btVector3(3, 0.5, 1.5), new btVector3(2, 2, 1)));

    }

    ds->addBody(new TouchedBodyHilite(ds));

    PointerPanel pp(ds);
    ds->getFace()->addPanel("pointerPanel", &pp);
    pp.setPosition(400, 400);
    pp.setSize(300, 300);

    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

}

void runHumanoid() {

    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    ds->addGround(15, 1, 15, 0, -10, 0);

    Humanoid* h = new Humanoid(btVector3(0, 3, 0));
    ds->addBody(h);

//    Humanoid* h2 = new Humanoid(btVector3(0, 1.5, 0));
//    ds->addBody(h2);

    RetinaPanel rp(h->eyeRetina);
    ds->getFace()->addPanel("retina", &rp);
    rp.setSize(230, 150);
    rp.setPosition(300, 600);

    RetinaPanel rp2(h->lhRetina);
    ds->getFace()->addPanel("lhRetina", &rp2);
    rp2.setSize(150, 150);
    rp2.setPosition(600, 600);

    BrainOutsPanel bop(h->brain, 50);
    ds->getFace()->addPanel("brainOuts", &bop);
    bop.setPosition(400, 400);


    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

}

class SignalGeneratorBox : public BoxBody {
    NeuralSignalsPanel *nsp;

public:

    SignalGeneratorBox(btVector3* _pos, btVector3* _size, vector<float>* b, int _historySize, const char *label) : BoxBody(_pos, _size) {
        nsp = new NeuralSignalsPanel(b, _historySize);

        TextRect* labelRect = new TextRect(label, 0.2, 0.2);
        *(labelRect->pos) = btVector3(-1, 1.0, 0);
        front()->push_back(labelRect);

        TextRect* labelRect2 = new TextRect(label, 0.2, 0.2);
        *(labelRect2->pos) = btVector3(1, -1.0, 0);
        front()->push_back(labelRect2);

    }

    void drawRect(float r, float g, float b, float x1, float y1, float w, float h, float z) {
        glColor4f(r, g, b, 1);
        glVertex3f(x1, y1, z);
        glVertex3f(x1, y1 + h, z);
        glVertex3f(x1 + w, y1 + h, z);
        glVertex3f(x1 + w, y1, z);
    }

    void drawSignals(double z) {
        nsp->update();

        float hWidth = 1.0 / float(nsp->historySize);
        float oHeight = 1.0 / float(nsp->numSignals);

        double min = -1, max = 1;

        bool normalize = true;
        if (normalize) {
            min = max = nsp->v[0][0];
            for (unsigned o = 0; o < nsp->numSignals; o++) {
                for (unsigned h = 0; h < nsp->historySize; h++) {
                    double a = nsp->v[o][h];
                    if (a < min) min = a;
                    if (a > max) max = a;
                }
            }
            if (abs(min) > abs(max)) {
                max = -min;
            } else if (abs(min) < abs(max)) {
                min = -max;
            }
        }

        float x = 0;
        float y = oHeight / 2;


        for (int o = 0; o < nsp->numSignals; o++) {
            for (int h = 0; h < nsp->historySize; h++) {
                float a = fmin(a, max);
                a = fmax(a, min);
                a = (nsp->v[o][h] - min) / (max - min);

                double r, g, b;
                if (nsp->v[o][h] < 0) {
                    r = a;
                    g = a / 4.0;
                    b = a / 2.0;
                } else {
                    b = a / 4.0;
                    g = a;
                    r = a / 2.0;
                }
                float bh = (1.0 * abs(a - 0.5)) * oHeight;
                drawRect(r, g, b, -0.5 + x, -1.0 + y + (1.0 - bh) / 2, hWidth, bh, z);
                x += hWidth;
            }
            x = 0;
            y += oHeight;

        }
    }

    virtual void drawFront() {
        BoxBody::drawFront();

        float d = size->getZ() + 0.1;

        glPushMatrix();
        glScalef(-size->getX()*2, size->getY()*2, -1.0);


        glBegin(GL_QUADS);
        //drawRect(0, 0, 0, -0.5, -0.5, 1.0, 1.0, d);
        drawSignals(d * 2);
        glEnd();

        glPopMatrix();

    }
};




void runBrain2Demo() {
    Brain* b = new Brain();
    b->printSummary();

    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    ds->setShadows(false);
    //ds->setTexturing(false);

    btRigidBody* g = ds->addGround(50, 5, 50, 0, -20, 0);

    FDBrainBody fdb(b, 100, 100, 35);
    ds->addBody(&fdb);


    AbstractBody* gen = new SignalGeneratorBox(new btVector3(-10, 10, 0), new btVector3(4, 3, 0.1), &(b->inValues), 128, "Inputs");
    ds->addBody(gen);

    BrainPanel *bp = new BrainPanel(b);
    ds->getFace()->addPanel("brainControl", bp);
    bp->setPosition(15, 15);

    //FDBrainPanel fdo(&fdb);
    //ds->getFace()->addPanel("fdPanel", &fdo);
    //fdo.span(350, 500, 450, 600);

    //    BrainInsPanel bip(b, 100);
    //    ds->getFace()->addPanel("brainIns", &bip);
    //    bip.setPosition(600, 600);


    BrainOutsPanel bop(b, 100);
    ds->getFace()->addPanel("brainOuts", &bop);
    bop.setPosition(800, 600);


    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

}

//void runSpiderLab() {
//    Audio* audio = new Audio();
//
//    DefaultSpace* ds = new DefaultSpace(audio);
//
//    ds->addGround(15, 1, 15, 0, -10, 0);
//
//    int numLegs = 6;
//    vector<btScalar>* legLengths = new vector<btScalar> ();
//    legLengths->push_back(0.5);
//    legLengths->push_back(0.5);
//    legLengths->push_back(0.2);
//    SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, btVector3(0, 10, 0), 48);
//    ds->addBody(spider);
//
//    for (unsigned l = 0; l < numLegs; l++) {
//        RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
//        string panelName = "retina_";
//        panelName[5] = 'a' + l;
//        ds->getFace()->addPanel(panelName, rp);
//
//        int w = 56;
//        rp->setSize(w, w);
//        rp->setPosition(w * l, 0);
//    }
//
//
//    //add rocks
//    float rw = 10;
//    float rh = 10;
//    float rx = 0.3;
//    float ry = 0.15;
//    float rz = 0.3;
//    for (float x = -rw; x < rw; x+=4.0) {
//        for (float y = -rh; y < rh; y+=4.0) {
//            ds->addBody(new BoxBody(new btVector3(x, 0.0, y), new btVector3(rx, ry, rz)));
//        }
//    }
//
////    BrainInsPanel bip(spider->brain, 100);
////    ds->getFace()->addPanel("brainIns", &bip);
////    bip.setPosition(600, 600);
////    bip.setSize(100, 600);
//
//    BrainOutsPanel bop(spider->brain, 10);
//    ds->getFace()->addPanel("brainOuts", &bop);
//    bop.setPosition(800, 600);
//    bop.setSize(100, 600);
//
//    runGLWindow(0, NULL, 1024, 800, VERSION, ds);
//
//    delete audio;
//
//}

//void runMultiSpiders() {
//    Audio* audio = new Audio();
//
//    DefaultSpace* ds = new DefaultSpace(audio);
//
//    ds->addGround(35, 1, 35, 0, -10, 0);
//
//    {
//        int numLegs = 3;
//        vector<btScalar>* legLengths = new vector<btScalar> ();
//        legLengths->push_back(0.6);
//        legLengths->push_back(0.4);
//        legLengths->push_back(0.4);
//        legLengths->push_back(0.4);
//        legLengths->push_back(0.2);
//        SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, btVector3(0, 10, 0), 25);
//        ds->addBody(spider);
//
//        for (unsigned l = 0; l < numLegs; l++) {
//            RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
//            string panelName = "retina_";
//            panelName[5] = 'a' + l;
//            ds->getFace()->addPanel(panelName, rp);
//
//            int w = 100;
//            rp->setSize(w, w);
//            rp->setPosition(w * l, 300);
//        }
//
//    }
//
//    {
//        int numLegs = 6;
//        vector<btScalar>* legLengths = new vector<btScalar> ();
//        legLengths->push_back(0.86);
//        legLengths->push_back(0.86);
//        SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, btVector3(0, 10, 0), 25);
//        ds->addBody(spider);
//
//        for (unsigned l = 0; l < numLegs; l++) {
//            RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
//            string panelName = "xretina_";
//            panelName[5] = 'a' + l;
//            ds->getFace()->addPanel(panelName, rp);
//
//            int w = 100;
//            rp->setSize(w, w);
//            rp->setPosition(w * l, 700);
//        }
//
//    }
//
//    //add rocks
//    float rw = 10;
//    float rh = 10;
//    float rx = 0.3;
//    float ry = 0.15;
//    float rz = 0.3;
//    for (float x = -rw; x < rw; x+=4.0) {
//        for (float y = -rh; y < rh; y+=4.0) {
//            ds->addBody(new BoxBody(new btVector3(x, 0.0, y), new btVector3(rx, ry, rz)));
//        }
//    }
//
//    runGLWindow(0, NULL, 1024, 800, VERSION, ds);
//
//    delete audio;
//
//}

void runSpiderWithBrains() {
    Audio* audio = new Audio();

    DefaultSpace* ds = new DefaultSpace(audio);
    ds->setBackgroundColor(0.8, 0.8, 1.0f);

    int numLegs = 3;
    vector<btScalar>* legLengths = new vector<btScalar> ();
    vector<btScalar>* legRadii = new vector<btScalar> ();
    legLengths->push_back(0.8);    legRadii->push_back(0.1);
    legLengths->push_back(0.5);    legRadii->push_back(0.10);
    legLengths->push_back(0.5);    legRadii->push_back(0.08);
    SpiderBody2* spider = new SpiderBody2(numLegs, legLengths, legRadii, btVector3(0, 10, 0), 64);
    ds->addBody(spider);

//    for (unsigned s = 0; s < 4; s++)    {
//            vector<btScalar>* legLengthsB = new vector<btScalar> ();
//            vector<btScalar>* legRadiiB = new vector<btScalar> ();
//            legLengthsB->push_back(0.8);    legRadiiB->push_back(0.15);
//            legLengthsB->push_back(0.7);    legRadiiB->push_back(0.10);
//            legLengthsB->push_back(0.6);    legRadiiB->push_back(0.10);
//            legLengthsB->push_back(0.5);    legRadiiB->push_back(0.10);
//            legLengthsB->push_back(0.5);    legRadiiB->push_back(0.10);
//            legLengthsB->push_back(0.5);    legRadiiB->push_back(0.10);
//
//            SpiderBody2* spiderB = new SpiderBody2(1, legLengthsB, legRadiiB, btVector3(4, 10, 0), 2);
//            ds->addBody(spiderB);
//            spiderB->setDamping(0.9);
//    }
    
//    {
//            vector<btScalar>* legLengthsB = new vector<btScalar> ();
//            vector<btScalar>* legRadiiB = new vector<btScalar> ();
//            legLengthsB->push_back(0.42);    legRadiiB->push_back(0.20);
//            legLengthsB->push_back(0.32);    legRadiiB->push_back(0.20);
//            legLengthsB->push_back(0.21);    legRadiiB->push_back(0.25);
//            legLengthsB->push_back(0.1);    legRadiiB->push_back(0.25);
//
//            SpiderBody2* spiderB = new SpiderBody2(2, legLengthsB, legRadiiB, btVector3(4, 10, 0), 4);
//            ds->addBody(spiderB);
//    }

    ds->addGround(10, 1, 10, 0, -10, 0);
    {
        float wallHeight = 0.75;
        float h = 8;
        spider->setDamping(0.5);
        ds->addBody(new BoxBody(new btVector3(0, 0, -h/2.0), new btVector3(4, wallHeight, 0.5)));
        ds->addBody(new BoxBody(new btVector3(0, 0, h/2.0), new btVector3(4, wallHeight, 0.5)));
        ds->addBody(new BoxBody(new btVector3(-h/2.0, 0, 0), new btVector3(0.5, wallHeight, 4)));
        ds->addBody(new BoxBody(new btVector3(h/2.0, 0, 0), new btVector3(0.5, wallHeight, 4)));

    }

    for (unsigned l = 0; l < numLegs; l++) {
        RetinaPanel* rp = new RetinaPanel(spider->legEye[l]);
        string panelName = "retina_";
        panelName[5] = 'a' + l;
        ds->getFace()->addPanel(panelName, rp);

        int w = 130;
        rp->setSize(w, w);
        rp->setPosition(w * l, 0);
    }

    BrainPanel *bp = new BrainPanel(spider->brain);
    ds->getFace()->addPanel("brainControl", bp);
    bp->setPosition(15, 15);

    {
//    FDBrainBody fdb(spider->brain, 25, 5, 25);
//    fdb.neuronSize = 0.1;
//    gds->addBody(&fdb);
    }

//    BrainInsPanel bip(spider->brain, 100);
//    ds->getFace()->addPanel("brainIns", &bip);
//    bip.setPosition(600, 600);
//    bip.setSize(100, 600);

    BrainOutsPanel bop(spider->brain, 100);
    ds->getFace()->addPanel("brainOuts", &bop);
    bop.setPosition(800, 600);
    bop.setSize(100, 600);

    runGLWindow(0, NULL, 1024, 800, VERSION, ds);

    delete audio;

}

class SliderTestPanel : public Panel {

public:
    float a;
    float b;

    SliderTestPanel() : Panel() {

        a = b = 0.5;


        HSlider* hs = new HSlider(&a, 0, 1.0);
        hs->span(15, 15, 200, 40);
        addPanel("hslider", hs);

        VSlider* vs = new VSlider(&b, 0, 1.0);
        vs->span(15, 50, 50, 250);
        addPanel("vslider", vs);

    }

};

void runSliders() {
    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    SliderTestPanel *stp = new SliderTestPanel();
    stp->span(5,5,400,400);
    ds->getFace()->addPanel("stp", stp);

    runGLWindow(0, NULL, 1024, 800, "Sliders", ds);

    delete audio;

}

void runFont3D() {
    Audio* audio = new Audio();
    DefaultSpace* ds = new DefaultSpace(audio);

    char const *file = "media/font/OCRA.ttf";

    // Initialise FTGL stuff
    static FTPolygonFont* font = new FTPolygonFont(file);

    if(font->Error())
    {
        fprintf(stderr, "could not load font `%s'\n", file);
        return;
    }

    font->FaceSize(40);
    font->CharMap(ft_encoding_unicode);

    class FontBody : public AbstractBody {
        float n;
        virtual void draw() {
            glEnable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE);

            glPushMatrix();
                //glColorMaterial(GL_FRONT, GL_DIFFUSE);
                glTranslatef(0.0, 0.0, 20.0);
                glScalef(-0.1, 0.1, 0.1);
//                glRotatef(n / 1.11, 0.0, 1.0, 0.0);
//                glRotatef(n / 2.23, 1.0, 0.0, 0.0);
//                glRotatef(n / 3.17, 0.0, 0.0, 1.0);
                //glColor3f(1.0, 0.5, 1.0);
                //drawText3D(font[fontindex], "SpaceGraph", 0, 0, 20.0);
                font->Render("SpaceGraph");
            glPopMatrix();

            //n += 1.0;
        }
    };

    ds->addBody(new FontBody());

    runGLWindow(0, NULL, 1024, 800, "Sliders", ds);

    delete audio;

}
