#ifndef OFXTRACKERBLOBH
#define OFXTRACKERBLOBH

#include "ofxCvBlob.h"
#include "ofxVec2f.h"

class ofxTrackerTimeSlice;
class ofxTrackerBlobMatch;

enum FLAG_TYPE {
		FLAG_OUT_OF_FRAME = 1
};

class ofxTrackerBlob {
public:
	ofxTrackerBlob(ofxTrackerTimeSlice* pSlice, ofxCvBlob oBlob);
	~ofxTrackerBlob();
	void draw();	
	void drawMatches(float fChange = 10.0f);
	void deleteMatches();
	float calculateK1(ofxTrackerBlob* pBlob);
	float calculateK2(ofxTrackerBlob* pBlob);
	void calculateVelocity(ofxTrackerBlob* pBlob);
	float getAbsX();
	float getAbsY();
	bool hasID();
	bool isMatched();
	void setMatched(bool bIsMatched);
	void setID(int nID);
	int getID() const;
	void setMatchingBlob(ofxTrackerBlob* pBlob); 
	ofxTrackerBlobMatch* getBestAvailableMatch();
	vector<ofxTrackerBlobMatch*> matches;	

	int flag;
	
	ofxCvBlob cv_blob;
	ofxTrackerTimeSlice* slice;
private:
	ofxTrackerBlob* matching_blob;
	
	ofxVec2f velocity;
	ofxVec2f position;
	ofxVec2f predicted_position;

	float max_dist;
	int id;	
	bool is_matched;
	bool has_id;
	
};


#endif