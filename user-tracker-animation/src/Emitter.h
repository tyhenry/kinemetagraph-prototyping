//
//  Emitter.h
//  user-tracker-animation
//
//  Created by Tyler Henry on 5/9/15.
//
//

#pragma once
#include "ofMain.h"
#include "Particle.h"


class Emitter{
public:
    
    Emitter(ofVec2f _pos, int size, float hue);
    void resetForces();
    void applyForce(ofVec2f _force);
    void update();
    void draw();
    
    ofVec2f pos;
    int size;
    float hue;
    vector<Particle> particles;
    
    bool dead; //track life of emitter
};