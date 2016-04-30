#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    initial.push_back(ofPoint(100,100,100));
    initial.push_back(ofPoint(120,100,100));
    initial.push_back(ofPoint(100,120,100));
    initial.push_back(ofPoint(100,100,120));
    initial.push_back(ofPoint(120,120,100));
    initial.push_back(ofPoint(100,120,120));
    initial.push_back(ofPoint(120,100,120));
    initial.push_back(ofPoint(150,100,100));
    initial.push_back(ofPoint(100,150,150));
    
    target = initial;
    for(int i = 0; i < target.size(); i++){
        target[i].x += 3;
    }
    
    icp.setClosestPointPairs(9);
    icp.setErrorThreshold(0.01);
    icp.setMaxIterations(10);
    
    icp.compute(initial, target, output, transformation, error, iterations);
    
    cout << transformation << endl;
    
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    easycam.enableMouseInput();
    easycam.begin();
    
    ofSetColor(255, 0, 0);
    for(int i = 0; i < initial.size(); i++){
        ofDrawSphere(initial[i],2);
    }
    ofSetColor(0, 255, 0);
    for(int i = 0; i < output.size(); i++){
        ofDrawSphere(output[i],2);
    }
    ofSetColor(0, 0, 255);
    for(int i = 0; i < target.size(); i++){
        ofDrawSphere(target[i],1);
    }
   
    easycam.end();
}
