#pragma once

#include "ofMain.h"
#include "ofxICP.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

    vector<ofPoint> initial;
    vector<ofPoint> target;
    vector<ofPoint> output;
    ofMatrix4x4 transformation;
    double error;
    int iterations;
    
    ofxIcp icp;
    
    ofEasyCam easycam;
};
