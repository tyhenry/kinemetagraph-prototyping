#pragma once

#include "ofMain.h"

#include "ofxNI2.h"
#include "ofxNiTE2.h"

//#include "Emitter.h"
#include "Particle.h"


//CONTAINS A FLIC IMAGE AND POSE DATA
struct ImgPose {
    
    int imgID = 0;
    
    string imgPath;
    
    //left = left side of image
    //(right side of body)
    
    ofVec2f lSho; //left shoulder x,y point (col. 2,3)
    ofVec2f lElb; //left elbow (col. 4,5)
    ofVec2f lWri; //left wrist (col. 6,7)
    ofVec2f rSho; //right shoulder (col. 8,9)
    ofVec2f rElb; //right elbow (col. 10,11)
    ofVec2f rWri; //right wrist (col. 12,13)
    ofVec2f lHip; //left hip (col. 14,15)
    ofVec2f rHip; //right hip (col. 16,17)
    ofVec2f lEye; //left eye (col. 18,19)
    ofVec2f rEye; //right eye (col. 20,21)
    ofVec2f nose; //nose (col. 22,23)
    ofVec2f lUto; //left upper torso corner (col. 24,25)
    ofVec2f rLto; //right lower torso corner (col. 26,27)
    ofVec2f cSho; //center point between shoulders
    ofVec2f cHip; //center point between hips
    ofVec2f lKne; //left knee
    ofVec2f lFot; //left foot
    ofVec2f rKne; //right knee
    ofVec2f rFot; //right foot
    
    //ANGLES
    float lWriElb;
    float lElbSho;
    float lShoHip;
    float rWriElb;
    float rElbSho;
    float rShoHip;
    float lShoSho;
    float lHipHip;
    
    unsigned long long weightedAngle; //weighted angleSum
    
    int foundCount = 0;
    
    bool operator==(const ImgPose& img2) const
    {
        //return (x == a.x && y == a.y);
        return (imgID == img2.imgID);
    }
    
};

struct vectComp {
    string axis;
    vectComp(string axis){
        this->axis = axis;
    }
    bool operator()(const ofVec2f& lhs, const ofVec2f& rhs){
        if (axis == "x"){
            return lhs.x < rhs.x;
        } else {
            return lhs.y < rhs.y;
        }
    }
};



class ofApp : public ofBaseApp
{
public:
    void setup();
    void exit();
    
    void update();
    void draw();
    
    int angleToInt(float ang);  //takes angle -180,180, returns int 0-99
    void calcAngles(ImgPose& img); //stores angles within ImgPose based on joint positions
    
    //HELPER FUNCTIONS FOR STORING KINECT SKELETON DATA
    bool isUserDisplayable(ofxNiTE2::User::Ref& user);
    void setUserPose(map<string, ofVec3f>& tempPose, ofxNiTE2::User::Ref& user);
    void setUserImg(ImgPose& tempImg, map<string, ofVec3f>& tempPose);
    void setUserImg(ImgPose& tempImg, map<string, ofVec3f>& tempPose, ofCamera& camera);
    
    //POLYLINE ANIMATION
    void setUserOutline(ofPolyline& outline, ImgPose& user);
    void skelAddVerts(vector<ofVec2f>& skel, ofVec2f pt1, ofVec2f pt2, float num);
    ofVec2f findClosest(vector<ofPoint>& polyline, ofVec2f pt);
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofxNI2::Device device;
    ofxNiTE2::UserTracker tracker;
    
    ofImage depth_image;
    ofEasyCam cam;
    ofCamera screenCam;
    bool bUserTarget;
    
    ofImage currentImage; //current FLIC image being drawn
    
    map<string, ofVec3f> userPose; //user pose joint data in 3d world space
    ImgPose userImg; //user pose in 2d space
    bool bDisplayUser;
    
    //POLYLINE ANIMATION
    ofPolyline userOutline;
    ofRectangle bounds;
    vector<ofVec2f> joints, matrix;
    vector<ofPoint> bodyPts;
    float userScale = 0;
    
    //PARTICLES ANIMATION
    //vector<Emitter> emitters;
    vector<Particle> particles;
    
    //DEBUG
    bool debug, bDrawSkeleton;
    int nUsers;
    
};