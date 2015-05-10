#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(0);
    ofSetBackgroundAuto(false);
    
    
    /*--------------------------------------------------------------*/
    /*--------------------------------------------------------------*/
    /*                                                              */
    /*SETUP CSV AND OTHER STUFF FOR FLIC IMAGE SORTING AND SEARCHING*/
    /*                                                              */
    /*--------------------------------------------------------------*/
    /*--------------------------------------------------------------*/


    // Load the CSV File.
    flicCsv.loadFile(ofToDataPath("fullFLIC_important_data.csv"));
    
    //load the csv data into flicImgs vector
    for (int r=0; r<flicCsv.numRows; r++){
        
        //READ data from CSV file
        string path = flicCsv.getString(r,1); //file path
        float lShoX = flicCsv.getFloat(r,2); //lShoX = col 2 (starting from 0)
        float lShoY = flicCsv.getFloat(r,3); //lShoY = col 3
        float lElbX = flicCsv.getFloat(r,4); //lElbX = col 4
        float lElbY = flicCsv.getFloat(r,5); //lElbY = col 5
        float lWriX = flicCsv.getFloat(r,6); //lWriX = col 6
        float lWriY = flicCsv.getFloat(r,7); //lWriY = col 7
        float rShoX = flicCsv.getFloat(r,8); //rShoX = col 8
        float rShoY = flicCsv.getFloat(r,9); //rShoY = col 9
        float rElbX = flicCsv.getFloat(r,10); //rElbX = col 10
        float rElbY = flicCsv.getFloat(r,11); //rElbY = col 11
        float rWriX = flicCsv.getFloat(r,12); //rWriX = col 12
        float rWriY = flicCsv.getFloat(r,13); //rWriY = col 13
        float lHipX = flicCsv.getFloat(r,14); //lHipX = col 14
        float lHipY = flicCsv.getFloat(r,15); //lHipY = col 15
        float rHipX = flicCsv.getFloat(r,16); //rHipX = col 16
        float rHipY = flicCsv.getFloat(r,17); //rHipY = col 17
        float lEyeX = flicCsv.getFloat(r,18);
        float lEyeY = flicCsv.getFloat(r,19);
        float rEyeX = flicCsv.getFloat(r,20);
        float rEyeY = flicCsv.getFloat(r,21);
        float noseX = flicCsv.getFloat(r,22);
        float noseY = flicCsv.getFloat(r,23);
        float lUtoX = flicCsv.getFloat(r,24);
        float lUtoY = flicCsv.getFloat(r,25);
        float rLtoX = flicCsv.getFloat(r,26);
        float rLtoY = flicCsv.getFloat(r,27);
        
        
        //STORE data into ImgPose struct
        ImgPose flicImg; //temp storage struct
        flicImg.imgPath = path; //save file path
        flicImg.lSho.set(lShoX,lShoY); //save left shoulder values
        flicImg.lElb.set(lElbX,lElbY); //save left elbow values
        flicImg.lWri.set(lWriX,lWriY); //save left wrist values
        flicImg.rSho.set(rShoX,rShoY);
        flicImg.rElb.set(rElbX,rElbY);
        flicImg.rWri.set(rWriX,rWriY);
        flicImg.lHip.set(lHipX,lHipY);
        flicImg.rHip.set(rHipX,rHipY);
        flicImg.lEye.set(lEyeX,lEyeY);
        flicImg.rEye.set(rEyeX,rEyeY);
        flicImg.nose.set(noseX,noseY);
        flicImg.lUto.set(lUtoX,lUtoY);
        flicImg.rLto.set(rLtoX,rLtoY);
        
        
        //CALCULATE and store all joint angles
        calcAngles(flicImg);
        
        //save unique ID for image (make it same as row in CSV - easy)
        flicImg.imgID = r;
        
        //STORE the current ImgPose in the flicImgs vector
        flicImgs.push_back(flicImg); //store struct in vector
        
    }
    
    //saved all values?
    //print the front and back imgIDs
    
    cout << "flicImgs[0].imgID (unsorted):    " << flicImgs.front().imgID << endl;
    cout << "flicImgs[" << flicImgs.size()-1 << "].imgID (unsorted): " << flicImgs.back().imgID << endl;
    
    
    //SORT FLICIMGS VECTOR BASED ON ID
    
    double time = ofGetElapsedTimeMillis();
    sort(flicImgs.begin(),flicImgs.end(),ImgComp("id"));
    cout << "flicImgs.imgID sort took (ms): " << ofGetElapsedTimeMillis() - time << endl;
    
    cout << "flicImgs[0].imgID (sorted):    " << flicImgs.front().imgID << endl;
    cout << "flicImgs[" << flicImgs.size()-1 << "].imgID (sorted): " << flicImgs.back().imgID << endl;
    
    
    //CREATE MAP OF SORTED VECTORS
    
    sortImgs["lWriElb"] = flicImgs;
    sort(sortImgs["lWriElb"].begin(),sortImgs["lWriElb"].end(),ImgComp("lWriElb"));
    
    sortImgs["lElbSho"] = flicImgs;
    sort(sortImgs["lElbSho"].begin(),sortImgs["lElbSho"].end(),ImgComp("lElbSho"));
    
    sortImgs["lShoHip"] = flicImgs;
    sort(sortImgs["lShoHip"].begin(),sortImgs["lShoHip"].end(),ImgComp("lShoHip"));
    
    sortImgs["rWriElb"] = flicImgs;
    sort(sortImgs["rWriElb"].begin(),sortImgs["rWriElb"].end(),ImgComp("rWriElb"));
    
    sortImgs["rElbSho"] = flicImgs;
    sort(sortImgs["rElbSho"].begin(),sortImgs["rElbSho"].end(),ImgComp("rElbSho"));
    
    sortImgs["rShoHip"] = flicImgs;
    sort(sortImgs["rShoHip"].begin(),sortImgs["rShoHip"].end(),ImgComp("rShoHip"));
    
    sortImgs["lShoSho"] = flicImgs;
    sort(sortImgs["lShoSho"].begin(),sortImgs["lShoSho"].end(),ImgComp("lShoSho"));
    
    sortImgs["lHipHip"] = flicImgs;
    sort(sortImgs["lHipHip"].begin(),sortImgs["lHipHip"].end(),ImgComp("lHipHip"));
    
    cout << "finished map constructions" << endl;
    
    
    
    //INITIALIZE FOUNDIMGS VECTOR
    
    for (int i=0; i<flicImgs.size();i++){
        
        pair<int,int> tempPair;
        
        tempPair.first = flicImgs[i].imgID; //save imgID
        tempPair.second = 0; //foundCount initially 0
        
        foundImgs.push_back(tempPair);
        
    }
    cout << "foundImgs[0].first = " << foundImgs[0].first << ", .second = " << foundImgs[0].second << endl;
    cout << "foundImgs[" << foundImgs.size()-1 << "].first = " << foundImgs.back().first << ", .second = " << foundImgs.back().second << endl;
    
    
    imgIter = flicImgs.begin(); //start display with the first FLIC image in vector
    mapIter = sortImgs.begin(); //start joint search with the first default joint
    
    
    string filePath = "fullimages/" + flicImgs.begin()->imgPath;
    currentImage.loadImage(filePath);
    
    
    speed = 10;
    bLoopImgs = false;
    bMouseSearch = false;
    
    
    //RESET flicImgs foundCounts
    time = ofGetElapsedTimeMillis();
    for (int i=0; i<flicImgs.size(); i++){
        flicImgs[i].foundCount = 0;
    }
    cout << "flicImgs.foundCount reset took (ms): " << ofGetElapsedTimeMillis() - time << endl;
    
    
    
    /*--------------------------------------------------------------*/
    /*--------------------------------------------------------------*/
    /*                                                              */
    /*                      SETUP KINECT & OPENNI                   */
    /*                                                              */
    /*--------------------------------------------------------------*/
    /*--------------------------------------------------------------*/

	device.setup();
    
    if (color_image.setup(device)) // only for kinect device
    {
        color_image.setSize(320, 240);
        color_image.setFps(60);
        color_image.start();
    }
	
	if (tracker.setup(device))
	{
		cout << "tracker inited" << endl;
	}
    
    tracker.setSkeletonSmoothingFactor(0.65);
    //0-1: higher is smoother, lower is faster
    
    
    screenCam = tracker.getOverlayCamera();
    
    
    bPRefNear = true;

    
}

//--------------------------------------------------------------

//kinect close function (doesn't work sometimes?)
void ofApp::exit()
{
	tracker.exit();
	device.exit();
}




//--------------------------------------------------------------
void ofApp::update()
{
	device.update();
    
    if (tracker.getNumUser() > 0){
       
        ofxNiTE2::User::Ref user = tracker.getUser(0); //first user
        
        setUserPose(userPose, user); //save kinect data to userPose
        
        setUserImg(userImg, userPose); //save userPose data to userImg
        
        calcAngles(userImg); //calculate joint angles in userImg
        
        
        //RESET flicImgs foundCounts
        for (int i=0; i<flicImgs.size(); i++){
            flicImgs[i].foundCount = 0;
        }
        
        
        //SEARCH FOR IMGPOSE
        
        float threshold = 6;
        auto foundCountImgs = foundImgs;
        int totalFound = 0;
    
        //search thresholds start from userImg
        ImgPose userImgLo = userImg;
        ImgPose userImgHi = userImg;
        
        //set lower thresholds
        userImgLo.lWriElb -= threshold;
        userImgLo.lElbSho -= threshold*3;
        userImgLo.lShoHip -= threshold*3;
        userImgLo.rWriElb -= threshold;
        userImgLo.rElbSho -= threshold*3;
        userImgLo.rShoHip -= threshold*3;
        userImgLo.lShoSho -= threshold*3;
        userImgLo.lHipHip -= threshold*3;
        
        //set higher thresholds
        userImgHi.lWriElb += threshold;
        userImgHi.lElbSho += threshold*3;
        userImgHi.lShoHip += threshold*3;
        userImgHi.rWriElb += threshold;
        userImgHi.rElbSho += threshold*3;
        userImgHi.rShoHip += threshold*3;
        userImgHi.lShoSho += threshold*3;
        userImgHi.lHipHip += threshold*3;
        
        
        //SEARCH EACH JOINT ANGLE IN MAP

        for (mapIter = sortImgs.begin(); mapIter != sortImgs.end(); mapIter++){
            
            string jointName = mapIter->first;
            while (jointName == "lHipHip" || jointName == "lShoSho" || jointName == "lShoHip" || jointName == "rShoHip"){
                mapIter++;
                jointName = mapIter->first;
            }
            
            //find ImgPoses within threshold
            auto loIter = lower_bound(mapIter->second.begin(), mapIter->second.end(), userImgLo, ImgComp(mapIter->first));
            auto hiIter = upper_bound(mapIter->second.begin(), mapIter->second.end(), userImgHi, ImgComp(mapIter->first));
            
            bool found = true;
            
            if (loIter == hiIter){
                if (hiIter == mapIter->second.end()){
                    cout << mapIter->first << ": loIter and hiIter are at end..." << endl;
                    found = false;
                }
                else {
                    cout << mapIter->first << ": loIter == hiIter, but not at end, incrementing hiIter" << endl;
                    hiIter++;
                }
            } else {
                cout << mapIter->first << ": loIter != hiIter" << endl;
            }
            
            //if anything was found
            if (found){
                
                for (auto tempIt = loIter; tempIt != hiIter; tempIt++){
                    
                    (foundCountImgs[tempIt->imgID].second)++; //increment foundCount of appropriate imgID
                    totalFound++;
                    
                }

            }
        }
        //phew! searched all mapped joint vectors - results are in foundCountImgs
        cout << "found " << totalFound << " total joint matches" << endl;
    
        
        //SORT FOUNDCOUNTIMGS
    
        double time = ofGetElapsedTimeMillis();
        sort(foundCountImgs.begin(),foundCountImgs.end(),countComp());
        cout << "foundCountImgs sort took (ms): " << ofGetElapsedTimeMillis() - time << endl;
        cout << "foundCountImgs[0]: imgID = " << foundCountImgs[0].first << ", foundCount = " << foundCountImgs[0].second << endl;
        cout << "foundCountImgs[" << foundCountImgs.size()-1 << "]: imgID = " << foundCountImgs.back().first << ", foundCount = " << foundCountImgs.back().second << endl;
    
        //IF THE TOP RESULT FOUND X JOINT MATCHES
        //DISPLAY IT AS CURRENT IMAGE
        
        if (foundCountImgs[0].second > 3){
            
            int foundID = foundCountImgs.front().first;
            
            *imgIter = flicImgs[foundID];
            
            string filePath = "fullimages/" + imgIter->imgPath;
            
            ofVec2f btwnEyes;
            btwnEyes.x = (imgIter->lEye.x+imgIter->rEye.x)/2;
            btwnEyes.y = (imgIter->lEye.y+imgIter->rEye.y)/2;
            
            //shift the image the distance between the center of screen and
            imgOrigin.x = ofGetWidth()/2 - (btwnEyes.x);
            imgOrigin.y = 150 - btwnEyes.y;
            
            
            
            //LOAD NEW IMAGE
            currentImage.loadImage(filePath);
            
            
            //RESIZE IMAGE TO MATCH USERIMG SIZE
            
            //abs distance between ImgPose shoulders
            ofVec2f imgShoDist (abs(imgIter->rSho.x - imgIter->lSho.x), abs(imgIter->rSho.y - imgIter->lSho.y));
            
            //abs distance between userImg shoulders
            ofVec2f userShoDist (abs(userImg.rSho.x - userImg.lSho.x), abs(userImg.rSho.y - userImg.lSho.y));
            
            float imgScale = userShoDist.x / imgShoDist.x; //scaling ratio
            imgSize.set(imgScale*currentImage.width, imgScale*currentImage.height); //set scale on image for draw
            
            
            ofVec2f imgShoCenter; //center of ImgPose shoulders in scaled image space
            imgShoCenter = (imgIter->lSho.getInterpolated(imgIter->rSho, 0.5))*imgScale;
            
            
            ofVec2f userShoCenter; //center of userImg shoulders in screen space
            userShoCenter = userImg.lSho.getInterpolated(userImg.rSho, 0.5); //center between two shoulders
            
            imgOrigin = (userShoCenter - imgShoCenter); //shift scaled image to match shoulder center
            
            
        }

    }
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofSetColor(255);
    
	// draw depth image
	//depth_image.setFromPixels(tracker.getPixelsRef(pRefNear, pRefFar));
	//depth_image.draw(0, 0, ofGetWidth(), ofGetHeight());
	
	/* draw in 2D
	ofPushView();
    screenCam.begin(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
    
        //save userPose to 2d userImg while camera is running
        //setUserImg(userImg, userPose);
    
        //draw the kinect skeleton
        tracker.draw();
    
    screenCam.end();
	ofPopView();
     */
    
    //DRAW FLIC IMAGE
    
    ofSetColor(255,50);
    
    //currentImage.draw(imgOrigin);
    currentImage.draw(imgOrigin.x,imgOrigin.y,imgSize.x,imgSize.y);

}






/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*                                                              */
/*               FLIC IMAGE SEARCH HELPER FUNCTIONS             */
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
//takes in string (i.e. mapIter->first) and returns appropriate angle
//from the ImgPose

float ofApp::getAngle(ImgPose& img, string angleName){
    if (angleName == "lWriElb"){
        return img.lWriElb;
    }
    else if (angleName == "lElbSho"){
        return img.lElbSho;
    }
    else if (angleName == "lShoHip"){
        return img.lShoHip;
    }
    else if (angleName == "rWriElb"){
        return img.lWriElb;
    }
    else if (angleName == "rElbSho"){
        return img.rElbSho;
    }
    else if (angleName == "rShoHip"){
        return img.rShoHip;
    }
    else if (angleName == "lShoSho"){
        return img.lShoSho;
    }
    else if (angleName == "lHipHip"){
        return img.lHipHip;
    }
    else if (angleName == "weighted"){
        return img.weightedAngle;
    }
    else {
        return 0;
    }
}

//--------------------------------------------------------------
//takes in ImgPose and string (i.e. mapIter->first) and angle value
// and sets it in the ImgPose

void setAngle(ImgPose& img, string angleName, float angle){
    
    if (angleName == "lWriElb"){
        img.lWriElb = angle;
    }
    else if (angleName == "lElbSho"){
        img.lElbSho = angle;
    }
    else if (angleName == "lShoHip"){
        img.lShoHip = angle;
    }
    else if (angleName == "rWriElb"){
        img.lWriElb = angle;
    }
    else if (angleName == "rElbSho"){
        img.rElbSho = angle;
    }
    else if (angleName == "rShoHip"){
        img.rShoHip = angle;
    }
    else if (angleName == "lShoSho"){
        img.lShoSho = angle;
    }
    else if (angleName == "lHipHip"){
        img.lHipHip = angle;
    }
    
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


// FLIC IMAGE SEARCH FUNCTIONS

//--------------------------------------------------------------
// copies into foundPoses a vector of ImgPoses matching the search ImgPose with an angle tolerance of "tolerance" degrees
//
void ofApp::imgPoseJointSearch(vector<ImgPose>& imgPoses, ImgPose searchImg, string jointAngle, float tolerance, vector<ImgPose>& foundPoses){

    
    //search lower_bound with tolerance
    //setAngle(searchImg, jointAngle, getAngle(searchImg,jointAngle)-tolerance);
    
    vector<ImgPose>::iterator loIter = lower_bound(imgPoses.begin(), imgPoses.end(), searchImg, ImgComp(jointAngle));
    
    //search upper_bound with tolerance
    //setAngle(searchImg, jointAngle, getAngle(searchImg,jointAngle)+tolerance*2);
    vector<ImgPose>::iterator hiIter = upper_bound(imgPoses.begin(), imgPoses.end(), searchImg, ImgComp(jointAngle));
    
    copy(loIter, hiIter, back_inserter(foundPoses));
    
}




/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*                                                              */
/*               STORE KINECT DATA HELPER FUNCTIONS             */
/*                                                              */
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

//--------------------------------------------------------------
// returns user pose as map of joint names and ofVec3f positions
//
void ofApp::setUserPose(map<string, ofVec3f>& tempPose, ofxNiTE2::User::Ref& user){
    ofVec3f tempJoint;
    tempJoint.set(0,0,0);
    
    //if (tracker.getNumUser() > 0){
        
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
        
//    } else {
//        //set all joints to 0
//        tempPose["lSho"] = tempJoint;
//        tempPose["lElb"] = tempJoint;
//        tempPose["lHnd"] = tempJoint;
//        tempPose["rSho"] = tempJoint;
//        tempPose["rElb"] = tempJoint;
//        tempPose["rHnd"] = tempJoint;
//        tempPose["lHip"] = tempJoint;
//        tempPose["rHip"] = tempJoint;
//        tempPose["head"] = tempJoint;
//    }
    
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

    //tempImg.lEye.set(screenCam.worldToScreen(userPose["lEye"]));
    //tempImg.rEye.set(screenCam.worldToScreen(userPose["rEye"]));
    //tempImg.lUto.set(screenCam.worldToScreen(userPose["lUto"]));
    //tempImg.rLto.set(screenCam.worldToScreen(userPose["rUto"]);
    
}



//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    switch (key){
        case 'l':
            bLoopImgs = !bLoopImgs;
            break;
            
        case 'm':
            bMouseSearch = !bMouseSearch;
            break;
            
        case 's':
            bDrawLines = !bDrawLines;
            break;
            
        case '=':
            speed++;
            cout << "speed: " << speed << endl;
            break;
            
        case '-':
            if (speed > 1){
                speed--;
            }
            cout << "speed: " << speed << endl;
            break;
            
        case ',':
            if (bPRefNear){
                pRefNear -= 100;
                cout << "pRefNear = " << pRefNear << endl;
            } else {
                if (pRefFar-pRefNear > 100)
                    pRefFar -= 100;
                cout << "pRefFar = " << pRefFar << endl;
            }
            break;
        case '.':
            if (bPRefNear){
                if (pRefFar-pRefNear > 100)
                    pRefNear += 100;
                cout << "pRefNear = " << pRefNear << endl;
            } else {
                pRefFar += 100;
                cout << "pRefFar = " << pRefFar << endl;
            }
            break;
        case 'p':
            bPRefNear = !bPRefNear;
            if (pRefNear){
                cout << "adjusting pRefNear" << endl;
            } else {
                cout << "adjusting pRefFar" << endl;
            }
            break;
        default:
            break;
    }
}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}


