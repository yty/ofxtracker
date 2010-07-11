#ifndef OFXTRACKERBLOBMATCHH
#define OFXTRACKERBLOBMATCHH

#include "ofxTrackerBlob.h"

struct ofxTrackerBlobMatch {
	ofxTrackerBlob* blob;
	
	
	// keeps track of match result between owner and blob. 
	// owner is on frame "N", blob is on frame "N - 1" 
	ofxTrackerBlob* owner; 
	
	float k;

	ofxTrackerBlobMatch(ofxTrackerBlob* pBlob, ofxTrackerBlob* pOwner, float nK)
		:blob(pBlob)
		,owner(pOwner)
		,k(nK)
	{
	}

	// comparator.
	bool operator < (const ofxTrackerBlobMatch& rOther) const {
		return k < rOther.k;
	}
};

inline bool blob_match_compare_k(const ofxTrackerBlobMatch* pOne, const ofxTrackerBlobMatch* pTwo){ 
	return pOne->k > pTwo->k; 
}


#endif