#ifndef OFXTRACKERH
#define OFXTRACKERH

#include "ofxCvGrayscaleImage.h"

class ofxCvContourFinder;
class ofxTrackerTimeSlice;
class ofxTrackerTrajectory;
class ofxTrackerBlob;

class ofxTracker {
public:
	ofxTracker(ofxCvContourFinder* pContourFinder, float fX, float fY);
	~ofxTracker();
	
	void addImage(ofxCvGrayscaleImage oImage); 
	void draw(); // only for debugging purposes.
	void reposition();
	void getBlobsFromHistoryWithFlag(int nFlag,vector<ofxTrackerBlob*>&rFoundBlobs);
	
	float x;
	float y;
	ofxCvContourFinder* contour_finder;
	list<ofxTrackerTimeSlice*> slices;
	ofxTrackerTrajectory* trajectory;
	
private:
	int frame_count;

};
#endif