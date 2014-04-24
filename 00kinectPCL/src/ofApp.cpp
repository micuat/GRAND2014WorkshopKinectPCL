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
	cloud->is_dense = false;
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
	
	ofxPCL::PointXYZRGBCloud cloud = getPointCloudFromKinect(kinect, 8);

	ofxPCL::PointXYZRGBNormalCloud cloud_with_normals(new ofxPCL::PointXYZRGBNormalCloud::element_type);
	ofxPCL::normalEstimation(cloud, cloud_with_normals);
	ofMesh mesh = ofxPCL::triangulate(cloud_with_normals, 500);
	
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	
	if( toggleWireframe ) {
		mesh.drawWireframe();
	} else {
		mesh.drawFaces();
	}
	
	// another solution
	//ofxPCL::organizedFastMesh(kinect.getPixelsRef(), kinect.getRawDepthPixelsRef(), 8, 1).drawWireframe();
	
	ofPopMatrix();
	
	easyCam.end();

}

//--------------------------------------------------------------
void ofApp::exit(){
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
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
