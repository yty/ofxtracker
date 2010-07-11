#ifndef OFXTRACKERTRAJECTORYH
#define OFXTRACKERTRAJECTORYH

/**
 * Some interesting information
 * - The invisible hand algorithm: Solving the assignment problem with statistical physics
 *
 */

/**
 * This class is responsible for detecting the tracjectories of the blobs
 * and for assigning the correct IDs to the correct blobs. 
 *
 */
 
#include "ofMain.h"
#include "ofxTrackerBlob.h"

class ofxTracker;


// when using "const ofxTrackerBlob* pOne" gives this error:
// passing 'const ofxTrackerBlob' as 'this' argument of 'int ofxTrackerBlob::getID()' discards qualifiers
// so you cannot use it!
inline bool set_difference_on_id(ofxTrackerBlob* pOne, ofxTrackerBlob* pTwo){ 
	return pOne->getID() < pTwo->getID(); 
}



struct ofxTrackerHistory {
	ofxTrackerHistory(vector<ofxTrackerBlob*> oBlobs):blobs(oBlobs){}
	vector<ofxTrackerBlob*> blobs;
	vector<ofxTrackerBlob*> lost_blobs; // blobs which have been seen in the previous frame, but not in the current one.
}; 
 

class ofxTrackerTrajectory {
public:
	ofxTrackerTrajectory(ofxTracker* pTracker);
	void findTrajectories();
	int getNextID();	
	static int next_id;
private:
	ofxTracker* tracker;
	vector<int>used_ids;
	vector<ofxTrackerHistory*> history;
};

#endif