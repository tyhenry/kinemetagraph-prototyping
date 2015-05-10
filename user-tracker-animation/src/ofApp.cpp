#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetBackgroundAuto(false);
    //ofEnableBlendMode(OF_BLENDMODE_ADD);
    
    device.setup();
    
    if (tracker.setup(device))
    {
        cout << "tracker initialized" << endl;
    }
    
    //
    
    tracker.setSkeletonSmoothingFactor(0.8);
    cout << "skel smoothing factor: " << tracker.getSkeletonSmoothingFactor() << endl;
    
    screenCam = tracker.getOverlayCamera();
    
    userScale = ofGetHeight()/2;
    
    string imgPath = "ten-commandments-disc1-00059021.jpg";
    string filePath = "fullimages/" + imgPath;
    currentImage.loadImage(filePath);
    
    bDisplayUser = false;
}

void ofApp::exit()
{
    tracker.exit();
    device.exit();
}

//--------------------------------------------------------------
void ofApp::update()
{
    device.update();
    
    int numUsers = tracker.getNumUser();
    
    //alert if new user or lost a user and update nUsers
    if (numUsers < nUsers){
        cout << "dropped " << (nUsers-numUsers) << " user(s), " << numUsers << " users now" << endl;
        nUsers = numUsers;
    } else if (numUsers > nUsers){
        cout << "gained " << (numUsers-nUsers) << " user(s), " << numUsers << " users now" << endl;
        nUsers = numUsers;
    }
    
    if (numUsers > 0){
        
        ofxNiTE2::User::Ref user;
        
        for (int u=0; u<numUsers; u++){
        
            user = tracker.getUser(u); //get a user
            bDisplayUser = isUserDisplayable(user); //check if its a good one
            
            if (bDisplayUser){ //if it is, stick with it - exit loop
                break;
            }
        }
        
        if (bDisplayUser){ //if we found a good user, update the user data
        
            setUserPose(userPose, user); //save kinect data to userPose
            
            setUserImg(userImg, userPose, cam); //save userPose data to userImg
            
            calcAngles(userImg); //calculate joint angles in userImg
            
            setUserOutline(userOutline, userImg); //update the points for the particles
        }
    }
    
    
    //PARTICLES ANIMATION
    
    //ADD PARTICLES IF USER DETECTED
    if (bDisplayUser){
        for (int b=0; b<bodyPts.size(); b++){
            
            if (particles.size() < 3500){
                ofVec2f randPos(ofRandom(-10,10), ofRandom(-10,10));
                
                Particle particle(bodyPts[b] + randPos, 1, 2); //pos, init. velocity, radius
                particles.push_back(particle);
            }
        }
    }
    
    //UPDATE PARTICLES AND ERASE THE DEAD
    for (auto it=particles.begin(); it!=particles.end();){
        
        //apply noise
        it->applyNoise();
        
        //update
        ofVec2f imgOrigin(0,0); //set to screen-size for testing
        ofVec2f imgEnd(ofGetWidth(),ofGetHeight()); //set to screen-size for testing
        it->update(currentImage, imgOrigin, imgEnd);
        
        //check if dead, if so erase
        if (it->dead){
            it = particles.erase(it);
        } else {
            ++it;
        }
    }
    
}

//--------------------------------------------------------------
void ofApp::draw()
{
    
    ofSetColor(50,50,50,10);
    currentImage.draw(0,0,ofGetWidth(),ofGetHeight());
    
    
    // draw depth
    //depth_image.setFromPixels(tracker.getPixelsRef(1000, 4000));
    //depth_image.draw(0, 0);
    
    //draw in 3D
    ofPushView();
    cam.begin();
    
        if (debug && bDrawSkeleton){
            //draw the kinect skeleton
            tracker.draw();
        }
    
        screenCam.end();
        ofPopView();
    
    cam.end();

    if (debug){
        ofSetColor(255,0,255);
        for (int i=0;i<joints.size();i++){
            ofCircle(joints[i], 2); //draw joints
        }
        ofSetColor(255,255,0);
        for (int i=0;i<bodyPts.size();i++){
            ofCircle(bodyPts[i], 1); //draw
        }
    }
    
    //userOutline.draw(); //ofPolyline outline - doesn't work yet
    
    //DRAW PARTICLES
    
    for (auto it=particles.begin(); it!=particles.end(); it++){
        
        it->draw();
    }
    
    if (debug){
        ofSetColor(0,0,0,180);
        ofRect(0,0,300,80);
        ofSetColor(255, 0, 0);
        stringstream debug1;
        debug1 << "number of particles: " << particles.size();
        ofDrawBitmapString(debug1.str(), 20, 20);
        stringstream debug2;
        debug2 << "number of user trackers: " << tracker.getNumUser();
        ofDrawBitmapString(debug2.str(), 20, 40);
        stringstream debug3;
        debug3 << "framerate: " << ofGetFrameRate();
        ofDrawBitmapString(debug3.str(), 20, 60);

    }
}




/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*                                                              */
/*          HELPER FUNCTIONS FOR CALCULATING USER JOINTS        */
/*                                                              */
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/


//--------------------------------------------------------------
//used to convert floats to 2-digit ints for weightedAngle

int ofApp::angleToInt(float ang){
    ang = ofMap(ang, -180,180, 0, 99, true); //convert to 0-99f with clamp at boundaries
    int intAng = int(round(ang)); //round to int
    return intAng;
    
}


//--------------------------------------------------------------
//stores angles within ImgPose based on joint positions

void ofApp::calcAngles(ImgPose& img){
    
    //SAVE WEIGHTED AVG ANGLE
    
    ofVec2f zero(1,0); //y-axis is 0 degrees (maybe?)
    float fAngle = 0;
    int iAngle = 0;
    unsigned long long lAngle = 0;
    
    
    //1
    fAngle = zero.angle(img.lWri-img.lElb); //gets angle from lElb to lWri relative to x-axis
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 100000000000000;
    //save float angle
    img.lWriElb = fAngle;
    
    //2
    fAngle = zero.angle(img.lElb-img.lSho);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 1000000000000;
    //save float angle
    img.lElbSho = fAngle;
    
    //3
    fAngle = zero.angle(img.lSho-img.lHip);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 10000000000;
    //save float angle
    img.lShoHip = fAngle;
    
    //4
    fAngle = zero.angle(img.rWri-img.rElb);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 100000000;
    //save float angle
    img.rWriElb = fAngle;
    
    //5
    fAngle = zero.angle(img.rElb-img.rSho);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 1000000;
    //save float angle
    img.rElbSho = fAngle;
    
    //6
    fAngle = zero.angle(img.rSho-img.rHip);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 10000;
    //save float angle
    img.rShoHip = fAngle;
    
    //7
    fAngle = zero.angle(img.lSho-img.rSho);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle * 100;
    //save float angle
    img.lShoSho = fAngle;
    
    //8
    fAngle = zero.angle(img.lHip-img.rHip);
    iAngle = angleToInt(fAngle);
    lAngle += iAngle;
    //save float angle
    img.lHipHip = fAngle;
    
    //save weighted angle (value has all angles in one number)
    img.weightedAngle = lAngle;
    
}


/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*                                                              */
/*               STORE KINECT DATA HELPER FUNCTIONS             */
/*                                                              */
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

//--------------------------------------------------------------

bool ofApp::isUserDisplayable(ofxNiTE2::User::Ref& user){

    return user->getNumJoints() > 0 && user->getJoint(nite::JOINT_TORSO).getPositionConfidence() > 0.3;
}

//--------------------------------------------------------------
// returns user pose as map of joint names and ofVec3f positions
//
void ofApp::setUserPose(map<string, ofVec3f>& tempPose, ofxNiTE2::User::Ref& user){
    ofVec3f tempJoint;
    tempJoint.set(0,0,0);
    
    //grab joint data from kinect
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_SHOULDER).getGlobalPosition();
    tempPose["lSho"] = tempJoint;
    tempJoint.set(0,0,0); //reset joint position
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_ELBOW).getGlobalPosition();
    tempPose["lElb"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_HAND).getGlobalPosition();
    tempPose["lHnd"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_SHOULDER).getGlobalPosition();
    tempPose["rSho"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_ELBOW).getGlobalPosition();
    tempPose["rElb"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_HAND).getGlobalPosition();
    tempPose["rHnd"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_HIP).getGlobalPosition();
    tempPose["lHip"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_HIP).getGlobalPosition();
    tempPose["rHip"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_HEAD).getGlobalPosition();
    tempPose["head"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_KNEE).getGlobalPosition();
    tempPose["lKne"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_LEFT_FOOT).getGlobalPosition();
    tempPose["lFot"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_KNEE).getGlobalPosition();
    tempPose["rKne"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_RIGHT_FOOT).getGlobalPosition();
    tempPose["rFot"] = tempJoint;
    tempJoint.set(0,0,0);
    
    tempJoint = user->getJoint(nite::JOINT_TORSO).getGlobalPosition();
    tempPose["cTor"] = tempJoint;
    
}

//--------------------------------------------------------------
//returns user pose as 2d ImgPose
//
void ofApp::setUserImg(ImgPose& tempImg, map<string, ofVec3f>& tempPose){
    
    tempImg.lSho.set(screenCam.worldToScreen(tempPose["lSho"])); //save left shoulder values
    tempImg.lElb.set(screenCam.worldToScreen(tempPose["lElb"])); //save left elbow values
    tempImg.lWri.set(screenCam.worldToScreen(tempPose["lHnd"])); //save left wrist values
    tempImg.rSho.set(screenCam.worldToScreen(tempPose["rSho"]));
    tempImg.rElb.set(screenCam.worldToScreen(tempPose["rElb"]));
    tempImg.rWri.set(screenCam.worldToScreen(tempPose["rHnd"]));
    tempImg.lHip.set(screenCam.worldToScreen(tempPose["lHip"]));
    tempImg.rHip.set(screenCam.worldToScreen(tempPose["rHip"]));
    tempImg.nose.set(screenCam.worldToScreen(tempPose["head"]));
    tempImg.cSho.set(tempImg.lSho.getInterpolated(tempImg.rSho, 0.5));
    tempImg.cHip.set(tempImg.lHip.getInterpolated(tempImg.rHip, 0.5));
    tempImg.lKne.set(screenCam.worldToScreen(tempPose["lKne"]));
    tempImg.lFot.set(screenCam.worldToScreen(tempPose["lFot"]));
    tempImg.rKne.set(screenCam.worldToScreen(tempPose["rKne"]));
    tempImg.rFot.set(screenCam.worldToScreen(tempPose["rFot"]));
    
    //tempImg.lEye.set(screenCam.worldToScreen(userPose["lEye"]));
    //tempImg.rEye.set(screenCam.worldToScreen(userPose["rEye"]));
    //tempImg.lUto.set(screenCam.worldToScreen(userPose["lUto"]));
    //tempImg.rLto.set(screenCam.worldToScreen(userPose["rUto"]);
    
}

//--------------------------------------------------------------
//returns user pose as 2d ImgPose based on camera input
//
void ofApp::setUserImg(ImgPose& tempImg, map<string, ofVec3f>& tempPose, ofCamera& camera){
    
    tempImg.lSho.set(camera.worldToScreen(tempPose["lSho"])); //save left shoulder values
    tempImg.lElb.set(camera.worldToScreen(tempPose["lElb"])); //save left elbow values
    tempImg.lWri.set(camera.worldToScreen(tempPose["lHnd"])); //save left wrist values
    tempImg.rSho.set(camera.worldToScreen(tempPose["rSho"]));
    tempImg.rElb.set(camera.worldToScreen(tempPose["rElb"]));
    tempImg.rWri.set(camera.worldToScreen(tempPose["rHnd"]));
    tempImg.lHip.set(camera.worldToScreen(tempPose["lHip"]));
    tempImg.rHip.set(camera.worldToScreen(tempPose["rHip"]));
    tempImg.nose.set(camera.worldToScreen(tempPose["head"]));
    tempImg.cSho.set(tempImg.lSho.getInterpolated(tempImg.rSho, 0.5));
    tempImg.cHip.set(tempImg.lHip.getInterpolated(tempImg.rHip, 0.5));
    tempImg.lKne.set(camera.worldToScreen(tempPose["lKne"]));
    tempImg.lFot.set(camera.worldToScreen(tempPose["lFot"]));
    tempImg.rKne.set(camera.worldToScreen(tempPose["rKne"]));
    tempImg.rFot.set(camera.worldToScreen(tempPose["rFot"]));
    
    //tempImg.lEye.set(camera.worldToScreen(userPose["lEye"]));
    //tempImg.rEye.set(camera.worldToScreen(userPose["rEye"]));
    //tempImg.lUto.set(camera.worldToScreen(userPose["lUto"]));
    //tempImg.rLto.set(camera.worldToScreen(userPose["rUto"]);
    
}


/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*                                                              */
/*                       POLYLINE ANIMATION                     */
/*                                                              */
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/



void ofApp::setUserOutline(ofPolyline& outline, ImgPose& user){
    
    joints.clear();
    joints.push_back(user.lWri);
    joints.push_back(user.lElb);
    joints.push_back(user.lSho);
    joints.push_back(user.rWri);
    joints.push_back(user.rElb);
    joints.push_back(user.rSho);
    joints.push_back(user.lHip);
    joints.push_back(user.rHip);
    joints.push_back(user.lKne);
    joints.push_back(user.lFot);
    joints.push_back(user.rKne);
    joints.push_back(user.lFot);
    joints.push_back(user.nose);
    
    bounds.set(user.lSho,user.rHip);
    
    for (int i=0; i<joints.size(); i++){
        bounds.growToInclude(joints[i]);
    }
    
    joints.push_back(user.cHip);
    joints.push_back(user.cSho);
    joints.push_back(user.cSho.interpolate(user.cHip,0.5)); //center of torso
    
    skelAddVerts(joints,user.lWri,user.lElb,2);
    skelAddVerts(joints,user.lElb,user.lSho,2);
    skelAddVerts(joints,user.lSho,user.lHip,2);
    skelAddVerts(joints,user.lHip,user.lKne,2);
    skelAddVerts(joints,user.lKne,user.lFot,2);
    skelAddVerts(joints,user.rWri,user.rElb,2);
    skelAddVerts(joints,user.rElb,user.rSho,2);
    skelAddVerts(joints,user.rSho,user.rHip,2);
    skelAddVerts(joints,user.rHip,user.rKne,2);
    skelAddVerts(joints,user.rKne,user.rFot,2);
    skelAddVerts(joints,user.nose,user.cSho,2);
    skelAddVerts(joints,user.lSho,user.rSho,2);
    skelAddVerts(joints,user.lHip,user.rHip,2);
    
    
    bounds.scaleFromCenter(1.3);
    
    userScale = bounds.getBottom() - bounds.getTop(); //distance between top and bottom of bounds
    float scale = ofMap(userScale, 0, ofGetHeight()*2, 0.1, 2); //scale the outlines
    
    matrix.clear();
    
    float spacing = ofMap(userScale, 0, ofGetHeight()*2, 10, 20);
    
    for (int x=bounds.getLeft(); x<bounds.getRight()+spacing; x+=spacing){
        for (int y=bounds.getTop(); y<=bounds.getBottom()+10; y+=10){
            
            ofVec2f pt(x,y);
            matrix.push_back(pt);
            
        }
    }
    
    bodyPts.clear();
    
    for (int i=0;i<matrix.size();i++){
        
        //see if point works for outline
        for (int j=0;j<joints.size();j++){
            
            float dist = matrix[i].distance(joints[j]);
            if (dist < 30*scale){
                bodyPts.push_back(matrix[i]);
            }
        }
        
    }
    
    /*
     vector<ofPoint> tempPts = bodyPts;
     
     ofPolyline tempLine;
     
     for (auto it= tempPts.begin();it!=tempPts.end();it++){
     
     vector<ofPoint> tempPts = bodyPts;
     
     
     tempLine.clear();
     tempLine.addVertices(tempPts);
     
     //get nearest neighbor
     tempLine.getClosestPoint(tempLine2[i],;
     
     }
     
     
     outline.clear();
     outline.addVertices(bodyPts);
     outline.getSmoothed(1,1);
     */
    
    /*
     vector<ofPoint> tempPts;
     
     for (int i=0; i<bodyPts.size(); i++){
     
     ofVec2f tempPt = findClosest(bodyPts,bodyPts[i]);
     tempPts.push_back(tempPt);
     }
     
     outline.clear();
     outline.addVertices(tempPts);
     */
    
}


void ofApp::skelAddVerts(vector<ofVec2f>& skel, ofVec2f pt1, ofVec2f pt2, float num){
    
    ofVec2f newPt = pt1.interpolate(pt2,0.5);
    skel.push_back(newPt);
    
}


ofVec2f ofApp::findClosest(vector<ofPoint>& polyline, ofVec2f pt){
    
    ofVec2f closest(pt);
    float dist = 10000;
    
    for (int i=0; i<polyline.size(); i++){
        
        float newDist = pt.distance(polyline[i]);
        if (newDist < dist && dist > 0){
            closest.set(polyline[i]);
        }
        dist = newDist;
        
    }
    
    return closest;
    
}




//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    
    switch(key){
        case '-':
            if (userScale > 60){
                userScale-=10;
            }
            cout << "userScale: " << userScale << endl;
            break;
        case '=':
            if (userScale < ofGetHeight()-10){
                userScale+=10;
            }
            cout << "userScale: " << userScale << endl;
            break;
        case 'd':
            debug = !debug;
            break;
        case 'u':
            if (bDisplayUser){
                cam.setTarget(userPose["cTor"]);
                //target center of user[0] torso
            } else {
                cout << "no displayable users" << endl;
            }
            break;
        case 'r':
            cam.reset();
            break;
        case 's':
            bDrawSkeleton = !bDrawSkeleton;
            break;
            
    }
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
    
}