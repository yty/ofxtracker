#ifndef TRTIMESLICEH
#define TRTIMESLICEH

#include "ofxCvGrayscaleImage.h"
#include "ofxCvBlob.h"
#include "ofxTrackerBlob.h"
//class ofxTrackerBlob;

class ofxTrackerBlobMatch;

inline bool tracked_blobs_compare_position(const ofxTrackerBlob* pOne, const ofxTrackerBlob* pTwo){ 
	return pOne->cv_blob.centroid.x > pTwo->cv_blob.centroid.x; 
}

inline bool sort_on_id(const ofxTrackerBlob* pOne, const ofxTrackerBlob* pTwo){ 
	return pOne->getID() < pTwo->getID(); 
}



// Tr-acker-TimeSlice
class ofxTrackerTimeSlice {
public:
	ofxTrackerTimeSlice(
		ofxCvGrayscaleImage oInputImage
		,vector<ofxCvBlob> oBlobs
		,int nFrameNum
	);
	~ofxTrackerTimeSlice();
	
	void draw();
	void drawPrev(float fX, float fY);
	void compareTimeSlice(ofxTrackerTimeSlice* pSlice);
	void deleteMatches();
	vector<ofxTrackerBlobMatch*>getMatches(); // get all match values (used to assign IDs)
	vector<ofxTrackerBlob*>getUnmatchedBlobs();
	vector<ofxTrackerBlob*>getTrackedBlobsSortedOnID();
	void getBlobsWithFlag(int nFlag, vector<ofxTrackerBlob*>&rFoundBlobs);
	
	int width;
	int height;
	int frame_num;
	float x;
	float y;
	
	ofxTrackerTimeSlice* next;
	ofxTrackerTimeSlice* prev;
	ofxCvGrayscaleImage image;
	
	vector<ofxTrackerBlob*> tracked_blobs;
	vector<ofxCvBlob> cv_blobs;
	vector<ofxTrackerBlobMatch*> all_matches;

private:	
};
#endif



