#pragma once

#include "ofMain.h"

//KINECT OPENNI
#include "ofxNI2.h"
#include "ofxNiTE2.h"

//CSV IMPORT
#include "ofxCsv.h"
#include <limits>


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

//COMPARES TWO FLIC IMAGES BY JOINT ANGLES
//FOR VECTOR SORTING AND SEARCHING
struct ImgComp {
    string angleName;
    ImgComp(string angleName){
        this->angleName = angleName;
    }
    bool operator()(const ImgPose& img1, const ImgPose& img2){
        if (angleName == "lWriElb"){
            return img1.lWriElb < img2.lWriElb;
        }
        else if (angleName == "lElbSho"){
            return img1.lElbSho < img2.lElbSho;
        }
        else if (angleName == "lShoHip"){
            return img1.lShoHip < img2.lShoHip;
        }
        else if (angleName == "rWriElb"){
            return img1.rWriElb < img2.rWriElb;
        }
        else if (angleName == "rElbSho"){
            return img1.rElbSho < img2.rElbSho;
        }
        else if (angleName == "rShoHip"){
            return img1.rShoHip < img2.rShoHip;
        }
        else if (angleName == "lShoSho"){
            return img1.lShoSho < img2.lShoSho;
        }
        else if (angleName == "lHipHip"){
            return img1.lHipHip < img2.lHipHip;
        }
        else if (angleName == "weighted"){
            return img1.weightedAngle < img2.weightedAngle;
        }
        else if (angleName == "path"){ //sort imgPaths alphabetically
            int i=0;
            while ((i < img1.imgPath.length()) && (i < img2.imgPath.length()))
            {
                if (tolower (img1.imgPath[i]) < tolower (img2.imgPath[i])) return true;
                else if (tolower (img1.imgPath[i]) > tolower (img2.imgPath[i])) return false;
                i++;
            }
            
            if (img1.imgPath.length() < img2.imgPath.length()) return true;
            else return false;
        }
        else if (angleName == "id"){
            return img1.imgID < img2.imgID;
        }
    }
};

//compares two pairs made up of <int imgID,int foundCount>
//for foundImgs temp vector sort

struct countComp {
    bool operator()(const pair<int,int>& lhs, const pair<int,int>& rhs){
        return lhs.second > rhs.second; //compare counts (greater than to reverse sort)
    }
    
};




class ofApp : public ofBaseApp
{
public:
	void setup();
	void exit();
	
	void update();
	void draw();
    
    //HELPER FUNCTIONS FOR FLIC IMAGE SEARCHING
    int angleToInt(float ang);  //takes angle -180,180, returns int 0-99
    float getAngle(ImgPose& img, string angleName);
    void setAngle(ImgPose& img, string angleName, float angle);
    void calcAngles(ImgPose& img); //stores angles within ImgPose based on joint positions
    //SEARCH FUNCTIONS FOR FLIC IMAGES
    void imgPoseJointSearch(vector<ImgPose>& imgPoses, ImgPose searchImg, string jointAngle, float tolerance, vector<ImgPose>& foundPoses);
    
    //HELPER FUNCTIONS FOR STORING KINECT SKELETON DATA
    void setUserPose(map<string, ofVec3f>& tempPose, ofxNiTE2::User::Ref& user);
    void setUserImg(ImgPose& tempImg, map<string, ofVec3f>& tempPose);

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
    ofxNI2::ColorStream color_image;
    ofCamera screenCam;
    ofEasyCam easyCam;
    
    //test
    ofVec3f lHandWorld, rHandWorld;
    ofVec3f lHandScreen, rHandScreen;
    //end test
    
    map<string, ofVec3f> userPose; //user pose joint data in 3d world space
    ImgPose userImg; //user pose in 2d space
    
    wng::ofxCsv flicCsv; //csv file
    
    vector<ImgPose> flicImgs;
    vector<ImgPose>::iterator imgIter, pImgIter;
    
    //map contains full flicImg vectors sorted by different angles
    map<string, vector<ImgPose> > sortImgs;
    map<string, vector<ImgPose> >::iterator mapIter;
    
    //foundImgs contains all images
    //pair <imgID, foundCount>
    vector<pair <int, int> >foundImgs;
    
    ofImage currentImage; //current FLIC image being drawn
    ofVec2f imgOrigin; //where to draw the FLIC image
    ofVec2f lWriPoints, nosePoints;
    float imgWidth, imgHeight;
    ofVec2f imgSize; //resized image size
    
    int count, speed; //helpers for speed of FLIC images
    bool bLoopImgs, bMouseSearch, bDrawLines, bPRefNear; //booleans for what to draw
    float pRefNear = 3000;
    float pRefFar = 4000;
};

/* AVAILABLE OPENNI SKELETON JOINTS:
 
 example:
 for (int i = 0; i < tracker.getNumUser(); i++){
     ofxNiTE2::User::Ref user = tracker.getUser(i);
     const ofxNiTE2::Joint &joint = user->getJoint(nite::JOINT_HEAD);
 }
 
	JOINT_HEAD,
	JOINT_NECK,
 
	JOINT_LEFT_SHOULDER,
	JOINT_RIGHT_SHOULDER,
	JOINT_LEFT_ELBOW,
	JOINT_RIGHT_ELBOW,
	JOINT_LEFT_HAND,
	JOINT_RIGHT_HAND,
 
	JOINT_TORSO,
 
	JOINT_LEFT_HIP,
	JOINT_RIGHT_HIP,
	JOINT_LEFT_KNEE,
	JOINT_RIGHT_KNEE,
	JOINT_LEFT_FOOT,
	JOINT_RIGHT_FOOT
*/