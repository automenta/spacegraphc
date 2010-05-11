/*
 * Neuron.h
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#ifndef NEURON_H_
#define NEURON_H_

using namespace std;
#include <list>

class AbstractNeuron {
public:

    virtual float getOutput() {
        return 0;
    }

};

class Synapse {
public:
    AbstractNeuron* inputNeuron; // InputNeuron's Output Pointer

    float weight; // its synaptic weight -1.0f <-> +1.0f

    void print() {
        cout << "       Synapse(input?=" << inputNeuron << ", weight=" << weight << ")\n";
    }

    Synapse(AbstractNeuron* _inputNeuron, float _synapticWeight) {
        inputNeuron = _inputNeuron;
        weight = _synapticWeight;
    }

//    void setInput(AbstractNeuron* input) {
//        inputNeuron = input;
//    }
    
    virtual float getInput() {
        return inputNeuron->getOutput();
    }

};

class InNeuron : public AbstractNeuron {
private:
    float input; //sensed input

public:

    InNeuron() {
        AbstractNeuron();
        input = 0;
    }

    float getInput() {
        return input;
    }

    void setInput(float i) {
        if (i > 1.0) i = 1.0;
        if (i < -1.0) i = -1.0;
        input = i;
    }

    virtual float getOutput() {
        //cout << "inNeuron gotOutput: " << input << "\n";
        return input;
    }
};

class OutNeuron : public AbstractNeuron {
    float potential;
    float stimulationFactor; //sensitivity to input stimulus.  defines the discreteness or detail level of its output spectrum
    float potentialDecay;

public:

    OutNeuron() {
        AbstractNeuron();
        stimulationFactor = 0.1;
        potentialDecay = 0.99;
    }

    void setStimulationFactor(double newStimulationFactor) {
        stimulationFactor = newStimulationFactor;
    }

    void setDecay(double newDecay) {
        potentialDecay = newDecay;
    }

    void reset() {
        potential = 0;
    }

    void stimulate(float p) {
        potential += p * stimulationFactor;
        if (potential > 1.0) potential = 1.0;
        if (potential < -1.0) potential = -1.0;
        potential *= potentialDecay;
    }

    virtual float getOutput() {
        return potential;
    }
};

class SynapseBuilder {
public:
    // determines if id referes to an interneuron or sensorneuron
    bool isSensorNeuron;

    // id of neuron which axon connects to this synapse
    //InterNeuron neuron;

    //int realneuronID;
    int neurontargetlayer;

    // its "weight"
    float weight;
    float percentChanceInhibitorySynapses;

    AbstractNeuron* inNeuron;

    SynapseBuilder() {
        isSensorNeuron = false;
        neurontargetlayer = 0;
        weight = 0.0F;
        percentChanceInhibitorySynapses = 0.5f;
    }



};

/** inter-neuron */
class Neuron : public OutNeuron {
public:
    OutNeuron* target;

    // Consistent Synapses flag
    bool hasConsistentSynapses;
    // inhibitory synapses flag
    bool hasInhibitorySynapses;


    float output, nextOutput;
    bool isInhibitory;
    bool active;
    float firingThreshold;
    float potential;
    float potentialDecay;
    bool isPlastic;
    float plasticityStrengthen;
    float plasticityWeaken;
    float maxSynapseWeight;
    float minSynapseWeight;

    void print() {
        cout << "  Neuron" << '\n';
        cout << "    output? " << (target != NULL) << '\n';
        cout << "    inhibitory? " << isInhibitory << '\n';
        cout << "    plastic? " << isPlastic << '\n';
        cout << "    maxSynapticWeightMagnitude? " << maxSynapseWeight << '\n';
        cout << "    plasticity strengthen=" << plasticityStrengthen << ", weaken=" << plasticityWeaken << '\n';
        cout << "    potential decay " << potentialDecay << '\n';
        cout << "    firing threshold " << firingThreshold << '\n';

//        for (unsigned i = 0; i < synapses.size(); i++) {
//            Synapse* s = synapses[i];
//            s->print();
//        }
    }

    Neuron() {
        isInhibitory = false;

        active = true;

        isPlastic = false; // plasticity up & down

        potential = 0.0;
        output = nextOutput = 0;

        target = NULL;
    }

    void setActive(bool nextActive) {
        active = nextActive;
    }

    //returns how much weight has been changed
    double forward(float dt, list<Synapse*>* synapses) {
        double learnedTotal = 0;

        if (!active) {
            potential = 0.0f;
            nextOutput = 0.0f;
        }

        //TODO handle 'dt' appropriately

        // potential decay
        potential *= potentialDecay;

        // make every connection do it's influence on the neuron's total potential
        for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
            Synapse* s = *list_iter;

            // lower synaptic weights
            if (isPlastic) {
                double preWeight = s->weight;
                
                s->weight *= plasticityWeaken;
                
                learnedTotal += fabs(s->weight - preWeight);
            }

            potential += s->weight * s->getInput() /** s->dendriteBranch * */;
            //cout << "Synapse " << s << " " << s->getInput() << " :: pot=" << potential << "\n";
        }

        if (isInhibitory) {
            learnedTotal += forwardInhibitory(synapses);
        } else {
            learnedTotal += forwardExhibitory(synapses);
        }

        return learnedTotal;
    }

    double forwardInhibitory(list<Synapse*>* synapses) {
        double learnedTotal = 0;
        // do we spike/fire
        if (potential <= -1.0f * firingThreshold) {
            // reset neural potential
            //potential = 0.0f;
            potential += firingThreshold;

            // fire the neuron
            nextOutput = -firingThreshold;

            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
            if (isPlastic) {
                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
                    Synapse* s = *list_iter;
                    double o = s->getInput();

                    double preWeight = s->weight;
                    // if synapse fired, strengthen the weight
                    if ((o < 0.0f && s->weight > 0.0f) || (o > 0.0f && s->weight < 0.0f)) {
                        s->weight *= plasticityStrengthen;
                    }

                    // clamp weight
                    clampWeight(s);

                    learnedTotal += fabs(preWeight - s->weight);
                }
            }
        }// don't fire the neuron
        else {
            nextOutput = 0;
            // reset potential if < 0
            if (potential > 0.0f) {
                potential = 0.0f;
            }
        }
        return learnedTotal;
    }

    double forwardExhibitory(list<Synapse*>* synapses) {
        double learnedTotal = 0;
        // do we spike/fire
        if (potential >= +1.0f * firingThreshold) {
            // reset neural potential
            //potential = 0.0f;
            potential -= firingThreshold;

            // fire the neuron
            nextOutput = firingThreshold;

            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
            if (isPlastic) {
                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
                    Synapse* s = *list_iter;
                    double o = s->getInput();

                    double preWeight = s->weight;
                    // if synapse fired, strengthen the weight
                    if ((o > 0.0f && s->weight > 0.0f) || (o < 0.0f && s->weight < 0.0f)) {
                        s->weight *= plasticityStrengthen;
                    }

                    // if weight > max back to max
                    clampWeight(s);

                    learnedTotal += fabs(preWeight - s->weight);
                }
            }
        }// don't fire the neuron
        else {
            nextOutput = 0;
            // reset potential if < 0
            if (potential < 0.0f) {
                potential = 0.0f;
            }
        }
        return learnedTotal;
    }

    void clampWeight(Synapse* s) {
        s->weight = fmax(fmin(s->weight, maxSynapseWeight), minSynapseWeight);
    }


};

#endif /* NEURON_H_ */
