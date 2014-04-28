#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	ofEnableDepthTest();
	
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();
	
	ofSetFrameRate(30);
	
	cloudIndex = 0;
}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();
}

inline ofxPCL::PointXYZRGBCloud getPointCloudFromKinect(ofxKinect &kinect, int step = 2) {
	ofxPCL::PointXYZRGBCloud cloud(new ofxPCL::PointXYZRGBCloud::element_type);
	
	int count = 0;
	int w = kinect.getWidth();
	int h = kinect.getHeight();
	
	cloud->width = ceil((float)w / (float)step) * ceil((float)h / (float)step);
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				ofVec3f v = kinect.getWorldCoordinateAt(x, y);
				cloud->points[count].x = v.x;
				cloud->points[count].y = v.y;
				cloud->points[count].z = v.z;
				ofColor col = kinect.getColorAt(x, y);
				uint32_t rgb = ((uint32_t)col.r << 16 | (uint32_t)col.g << 8 | (uint32_t)col.b);
				cloud->points[count].rgb = *reinterpret_cast<float*>(&rgb);
				count++;
			}
		}
	}
	cloud->points.resize(count);
	cloud->width = count;
	
	return cloud;
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(10);
	
	easyCam.begin();
	
	ofxPCL::PointXYZRGBCloud cloud = getPointCloudFromKinect(kinect, 4);
	
	// Euclidean cluster extraction
	clouds = ofxPCL::clusterExtraction(cloud, 30.0f, 100, 5);
	
	ofMesh mesh;
	
	ofPushMatrix();
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000);
	
	vector<ofVec3f> curCentroids;
	
	for( int i = 0; i < clouds.size(); i++ ) {
		mesh = ofxPCL::toOF(clouds.at(i));
		
		ofVec3f centroid = mesh.getCentroid();
		
		curCentroids.push_back(centroid);
		
		ofColor color;
		if( i == cloudIndex ) {
			color.setHsb(255 * i / 10, 255, 255);
			ofPushMatrix();
			ofTranslate(centroid);
			ofDrawAxis(100);
			ofPopMatrix();
			
			ofPushMatrix();
			for( int j = prevCentroids.size() - 1; j >= 0 ; j-- ) {
				if( prevCentroids.at(j).size() == 0 ) continue;
				
				ofVec3f closestCentroid = prevCentroids.at(j).at(0);
				for( int i = 1; i < prevCentroids.at(j).size(); i++ ) {
					if(centroid.distanceSquared(closestCentroid) > centroid.distanceSquared(prevCentroids.at(j).at(i))) {
						closestCentroid = prevCentroids.at(j).at(i);
					}
				}
				ofTranslate(0, 0, 500);
				ofPushMatrix();
				ofTranslate(closestCentroid);
				ofDrawAxis(100);
				ofPopMatrix();
				
			}
			ofPopMatrix();
		} else {
			color = ofColor::gray;
		}
		ofSetColor(color);
		mesh.clearColors();
		mesh.drawVertices();
		
	}
	
	if( prevCentroids.size() > queueMax )
		prevCentroids.erase(prevCentroids.begin());
	prevCentroids.push_back(curCentroids);
	
	ofPopMatrix();
	
	easyCam.end();
	
	ofSetColor(255);
	ofDrawBitmapString("FPS: " + ofToString(ofGetFrameRate()), 20, 50);
	ofDrawBitmapString("Clusters: " + ofToString(clouds.size()), 20, 75);
	ofDrawBitmapString("Current Cloud: " + ofToString(cloudIndex), 20, 100);
}

//--------------------------------------------------------------
void ofApp::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if( key == ' ' ) {
		toggleWireframe = !toggleWireframe;
	}
	
	if( key == OF_KEY_UP ) {
		cloudIndex++;
	}
	
	if( key == OF_KEY_DOWN ) {
		cloudIndex--;
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
