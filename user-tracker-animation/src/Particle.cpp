//
//  Particle.cpp
//  user-tracker-animation
//
//  Created by Tyler Henry on 5/9/15.
//
//

#include "Particle.h"


Particle::Particle(ofVec2f _pos, float initVel, float _radius){
    
    pos.set(_pos);
    radius = _radius;
    
    vel.x = cos(ofRandom(0, TWO_PI)); //random start direction
    vel.y = sin(ofRandom(0, TWO_PI)); //random start direction
    
    lifespan = 5; //in frames
    countdown = lifespan;
    
    timeOffset = ofRandom(100000);
    yOffset = 1000;
    
    dead = false;
}

Particle::Particle(ofVec2f _pos, float _span, float _radius, float _hue){
    
    pos.set(_pos);
    span = _span;
    radius = _radius;
    hue = _hue;
    
    vel.x = ofRandom(-2*(_span),2*_span);
    vel.y = ofRandom(-3*(_span),_span);
    
    lifespan = 180; //3 seconds @ 60fps
    countdown = lifespan;
    
    dead = false;
}

void Particle::resetForces(){
    acc *= 0;
    
}

void Particle::applyForce(ofVec2f force){
    acc += force;
}

void Particle::applyNoise(){
    float noiseX = ofNoise(ofGetElapsedTimef()+timeOffset);
    float noiseY = ofNoise(ofGetElapsedTimef()+timeOffset+yOffset);
    
    noiseX = ofMap(noiseX,0,1,-1,1);
    noiseX = ofMap(noiseY,0,1,-1,1);
    
    ofVec2f noise(noiseX,noiseY);
    
    acc += noise;
}

void Particle::update(){
    
    if (countdown > 0){ //if still alive, update
        
        vel += acc;
        pos += vel;
        
        color.set(255); //if no image, set to white
        
        countdown--; //otherwise, set to dead
    } else {
        dead = true;
    }
}

void Particle::update(ofImage& img, ofVec2f imgOrigin, ofVec2f imgEnd){
    
    if (countdown > 0){ //if still alive, update
        
        vel += acc;
        pos += vel;
        
        ////////////////////////////////////////////
        //get color value of image at particle pos//
        ////////////////////////////////////////////
        
        ofVec2f imgScaleDims(imgEnd - imgOrigin); //scaled size of image
        ofVec2f imgPos(pos - imgOrigin); //distance from imgOrigin
        
        //find place of particle within original image size
        imgPos.x = ofMap(imgPos.x, 0, imgScaleDims.x, 0, img.getWidth());
        imgPos.y = ofMap(imgPos.y, 0, imgScaleDims.y, 0, img.getHeight());
        
        //if the particle is within the image, update the color of the particle
        if (imgPos.x >= 0 && imgPos.x <= img.getWidth() && imgPos.y >=0 && imgPos.y <= img.getHeight()){
            
            //find the color of the pixel at imgPos
            color = img.getColor(imgPos.x,imgPos.y);
            
        }
        
        countdown--;
        
    } else {
        dead = true; //otherwise, set to dead
    }
}

void Particle::draw(){
    float brightness = 255;
    ofSetColor(color);
    
    ofCircle(pos, radius);
}