//
//  Particle.h
//  user-tracker-animation
//
//  Created by Tyler Henry on 5/9/15.
//
//

#pragma once
#include "ofMain.h"

class Particle {
public:
    
    Particle(ofVec2f _pos, float initVel, float _radius); //constructor
    Particle(ofVec2f _pos, float _hue, float _span, float _radius); //overload for old emitter code
    void resetForces();
    void applyForce(ofVec2f force);
    void applyNoise();
    
    void update();
    void update(ofImage& img, ofVec2f imgOrigin, ofVec2f imgEnd);
    void draw();
    
    ofVec2f pos, vel, acc;
    float lifespan, countdown; //in frames
    bool dead;
    
    float span, radius, hue, timeOffset, yOffset;
    ofColor color;
};
