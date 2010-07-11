#include "ofxTrackerTimeSlice.h"
//#include "ofxTrackerBlob.h"
#include "ofxTrackerBlobMatch.h"

// create this new time slice.
ofxTrackerTimeSlice::ofxTrackerTimeSlice(
		ofxCvGrayscaleImage oInputImage
		,vector<ofxCvBlob> oBlobs
		,int nFrameNum
)
:image(oInputImage)
,cv_blobs(oBlobs)
,frame_num(nFrameNum)
,x(0)
,y(0)
,next(NULL)
,prev(NULL)
{
	width = image.width;
	height = image.height;
	
	// use our ofxTrackerBlob
	vector<ofxCvBlob>::const_iterator it = cv_blobs.begin();
	while(it != cv_blobs.end()) {
		ofxTrackerBlob* tracked_blob = new ofxTrackerBlob(this,(*it));
		tracked_blobs.push_back(tracked_blob);
		++it;
	}
	
	// sort tracked blobs on position (x position);
	std::sort(tracked_blobs.begin(), tracked_blobs.end(), tracked_blobs_compare_position);
}

ofxTrackerTimeSlice::~ofxTrackerTimeSlice() {
	vector<ofxTrackerBlob*>::iterator it = tracked_blobs.begin();
	while(it != tracked_blobs.end()) {
		delete (*it);
		++it;
	}
	tracked_blobs.clear();
	all_matches.clear();
	
	if(next != NULL) {
		next->deleteMatches();
	}
	
}
	
// draw the blobs and matches.
void ofxTrackerTimeSlice::draw() {
	// draw black/white image.
	glColor3f(1.0f, 0.0f, 0.0f);
	image.draw(x, y);
	
	// draw slice number
	glColor3f(1.0f, 1.0f, 1.0f);
	ofDrawBitmapString(ofToString(frame_num), x+10, y+20);

	// draw blobs contours.
	glPushMatrix();
		glColor4f(0.0f,1.0f, 1.0f, 1.0f);
		glTranslatef(x,y,0);
		vector<ofxCvBlob>::iterator it = cv_blobs.begin();
		while(it != cv_blobs.end()) {
			(*it).draw();
			++it;
		}
	glPopMatrix();

	// draw detection/tracking information.
	int  i = 0;
	vector<ofxTrackerBlob*>::const_iterator it_track = tracked_blobs.begin();
	while(it_track != tracked_blobs.end()) {
		glColor4f(1.0f, 0.0f, 0.6f, 1.0f);
		(*it_track)->draw();
		
		if(i == 0) 
			glColor4f(1.0f, 0.0f, 0.6f,0.4f);
		else if( i == 1) 
			glColor4f(1.0f, 1.0f, 0.0f,0.4f);
		else if (i == 2) 
			glColor4f(0.0f, 1.0f, 0.0f,0.4f);
		else 
			glColor4f(0.0f, 0.0f, 1.0f,0.4f);
		(*it_track)->drawMatches(i * 10.0f);
		++it_track;
		++i;
	}
	/*
	if(tracked_blobs.size() > 0) 
		tracked_blobs.front()->drawMatches();
	*/

}

void ofxTrackerTimeSlice::drawPrev(float fX, float fY) {

//	ofCircle(fX, fY, 40);
	glPushMatrix();
		glColor4f(1.0f,0.0f, 0.6f, 0.6f);
		glTranslatef(fX,fY,0);
		vector<ofxCvBlob>::iterator it = cv_blobs.begin();
		while(it != cv_blobs.end()) {
			(*it).draw();
			++it;
		}
	glPopMatrix();
}


// find blobs in the given slice and calculate the matching value.
void ofxTrackerTimeSlice::compareTimeSlice(ofxTrackerTimeSlice* pSlice) {
	char type = 'a';
	all_matches.clear();
	
	// test A) loop through all new blobs and match them against the previous ones.
	if (type == 'a') {
		vector<ofxTrackerBlob*>::iterator curr_it = tracked_blobs.begin();
		while(curr_it != tracked_blobs.end()) {
			vector<ofxTrackerBlob*>::iterator prev_it = pSlice->tracked_blobs.begin();
			vector<ofxTrackerBlobMatch*> matches;
			cout << "=================== area: " << (*curr_it)->cv_blob.area << " ============================================================================\n";
			while(prev_it != pSlice->tracked_blobs.end()) {
				float k1 = (*curr_it)->calculateK1((*prev_it));
				ofxTrackerBlobMatch* match = new ofxTrackerBlobMatch((*prev_it), (*curr_it), k1);
				matches.push_back(match);
				all_matches.push_back(match); 
				prev_it++;
			}
			std::sort(matches.begin(), matches.end(), blob_match_compare_k);
			cout << "\n";
			// calculate the velocity from the best match.
			if(matches.size() > 0) {
				//(*curr_it)->calculateVelocity(matches.at(0)->blob);		
			}
			(*curr_it)->matches = matches;
			curr_it++;
		}
		// now we need to sort all the matches.
		std::sort(all_matches.begin(), all_matches.end(), blob_match_compare_k);
		/*
		cout << "number of matches: " << all_matches.size() << std::endl;
		for(int i = 0; i < all_matches.size(); ++i) {
			cout << all_matches.at(i)->k1 << "\n";
		}
		*/
	}
	else if (type == 'b') {
	/*
		// test B) loop through previous blobs and check if they exist on the new one
		vector<ofxTrackerBlob*>::iterator curr_it = tracked_blobs.begin();
		while(curr_it != tracked_blobs.end()) {
			vector<ofxTrackerBlob*>::iterator prev_it = pSlice->tracked_blobs.begin();
			vector<ofxTrackerBlobMatch*> matches;
			cout << "=================== area: " << (*curr_it)->cv_blob.area << " =========================================\n";
			while(prev_it != pSlice->tracked_blobs.end()) {
				float k1 = (*curr_it)->calculateK1((*prev_it));
				ofxTrackerBlobMatch* match = new ofxTrackerBlobMatch((*prev_it), k1);
				matches.push_back(match);
				prev_it++;
			}
			std::sort(matches.begin(), matches.end(), blob_match_compare_k1);
			cout << "\n";
			// calculate the velocity from the best match.
			if(matches.size() > 0) {
				//(*curr_it)->calculateVelocity(matches.at(0)->blob);		
			}
			(*curr_it)->matches = matches;
			curr_it++;
		}
		*/
	}
}

/** 
 * ofxTracker uses a multiple frame sequences, so you can impelement 
 * blob trajectory matching based on multiple frames. When the "first" or
 * oldest frame is removed, we need to break all pointers to the blobs
 * which are removed.
 *
 */
void ofxTrackerTimeSlice::deleteMatches() {
	vector<ofxTrackerBlob*>::const_iterator it = tracked_blobs.begin();
	while(it != tracked_blobs.end()) {
		(*it)->deleteMatches();
		++it;
	}
	//tracked_blobs.clear();
	prev = NULL;
}


// get blobs from this timeframe which do not have a match yet.
vector<ofxTrackerBlob*> ofxTrackerTimeSlice::getUnmatchedBlobs() {
	vector<ofxTrackerBlob*> unmatched;
	vector<ofxTrackerBlob*>::iterator it = tracked_blobs.begin();
	while(it != tracked_blobs.end()) {
		if(!(*it)->hasID()) {
			unmatched.push_back((*it));
		}
		++it;
	}
	return unmatched;
}


void ofxTrackerTimeSlice::getBlobsWithFlag(int nFlag, vector<ofxTrackerBlob*>&rFoundBlobs) {
	vector<ofxTrackerBlob*>::const_iterator blob_it = tracked_blobs.begin();
	while(blob_it != tracked_blobs.end()) {
		if( ((*blob_it)->flag & nFlag) == nFlag) 
			rFoundBlobs.push_back((*blob_it));
		++blob_it;
	}
}

/**
 * When we compare each of the blobs in this time slice with the blobs
 * in a previous time slice we keep track of all the matches we made. When 
 * we have 3 blobs on this time slice and 2 in the previous time slice, which 
 * is passed to this.compareTimeSlice(), we have 6 different ofxTrackerBlobMatch
 * objects. 
 *
 * These matches are used when we are labelling the blobs. We start by giving
 * an ID to the blob which has the highest confidence value.
 */
vector<ofxTrackerBlobMatch*> ofxTrackerTimeSlice::getMatches() {
	return all_matches;
}

vector<ofxTrackerBlob*> ofxTrackerTimeSlice::getTrackedBlobsSortedOnID() {
	vector<ofxTrackerBlob*> to_sort = tracked_blobs;
	std::sort(to_sort.begin(), to_sort.end(), sort_on_id);
	return to_sort;
}