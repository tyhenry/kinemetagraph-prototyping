//
//  Emitter.cpp
//  user-tracker-animation
//
//  Created by Tyler Henry on 5/9/15.
//
//

#include "Emitter.h"


Emitter::Emitter(ofVec2f _pos, int _size, float _hue){
    
    pos.set(_pos);
    size = _size;
    hue = _hue;
    
    for (int i=0; i<size; i++){
        Particle particle(pos, (size/25), 5, hue);
        particles.push_back(particle);
    }
    
    dead = false;
    
}

void Emitter::resetForces(){
    for (int i=0; i<size; i++){
        particles[i].resetForces();
    }
    
}

void Emitter::applyForce(ofVec2f force){
    for (int i=0; i<size; i++){
        particles[i].applyForce(force);
    }
}

void Emitter::update(){
    dead = true;
    for (int i=0; i<size; i++){
        particles[i].update();
        
        if (!particles[i].dead) dead = false;
    }
    if (dead) {
        cout << "dead emitter" << endl;
    }
    
}

void Emitter::draw(){
    for (int i=0; i<size; i++){
        particles[i].draw();
    }
    
}

