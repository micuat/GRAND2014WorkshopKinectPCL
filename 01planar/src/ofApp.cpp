#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	ofEnableDepthTest();
	
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();
	
	ofSetFrameRate(30);
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
	
	// planar segmentation
	vector<ofxPCL::PointXYZRGBCloud> clouds = ofxPCL::segmentation(cloud, pcl::SACMODEL_PLANE, mouseX / 20.0f, 50, 10);
	
	// Euclidean cluster extraction
	//vector<ofxPCL::PointXYZRGBCloud> clouds = ofxPCL::clusterExtraction(cloud, mouseX / 20.0f, 50, 10);
	
	ofMesh mesh;
	
	ofPushMatrix();
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000);
	
	for( int i = 0; i < clouds.size(); i++ ) {
		mesh = ofxPCL::toOF(clouds.at(i));
		
		ofColor color;
		color.setHsb(255 * i / 10, 255, 255);
		ofSetColor(color);
		mesh.clearColors();
		mesh.drawVertices();
		
		// draw centroid
		ofPushMatrix();
		ofTranslate(mesh.getCentroid());
		ofDrawAxis(100);
		ofPopMatrix();
	}
	
	ofPopMatrix();
	
	easyCam.end();
	
	ofSetColor(255);
	ofDrawBitmapString("FPS: " + ofToString(ofGetFrameRate()), 20, 50);
	ofDrawBitmapString("Clusters: " + ofToString(clouds.size()), 20, 75);
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
