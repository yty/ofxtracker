#include "ofxTracker.h"
#include "ofxCvContourFinder.h"
#include "ofxTrackerTimeSlice.h"
#include "ofxTrackerTrajectory.h"

ofxTracker::ofxTracker(
	ofxCvContourFinder* pContourFinder
	,float fX
	,float fY
)
:contour_finder(pContourFinder)
,x(fX)
,y(fY)
,frame_count(0)
{
	trajectory = new ofxTrackerTrajectory(this);
}

ofxTracker::~ofxTracker() {
	delete trajectory;
}

// add a new image, and detect blobs
void ofxTracker::addImage(ofxCvGrayscaleImage oImage) {
	// find blobs
	int min_area = oImage.width * .01f;
	int max_area = oImage.width*oImage.height * .5;
	int num_balls = 3;
	bool find_holes = false;
	bool use_approx = false;

	contour_finder->findContours(
		oImage
		,min_area
		,max_area
		,num_balls
		,find_holes
		,use_approx
	);
		
	// create new time slice
	ofxTrackerTimeSlice* slice = new ofxTrackerTimeSlice(
		oImage
		,contour_finder->blobs
		,frame_count++
	);
	slices.push_back(slice);
	
	
	// we keep track of "X" number of slices.
	if(slices.size()>12) {
		list<ofxTrackerTimeSlice*>::iterator it = slices.begin();
		delete slices.front();
		slices.pop_front();
	}
	
	// compare last added slices with previous slice.
	if(slices.size() >= 2) {
		list<ofxTrackerTimeSlice*>::iterator prev = --(--(slices.end()));
		slices.back()->prev = (*prev);
		(*prev)->next = slices.back();
		slices.back()->compareTimeSlice(*prev);
	}
	trajectory->findTrajectories();
	reposition();
}

// show the blob detection and tracking information.
void ofxTracker::draw() {
	list<ofxTrackerTimeSlice*>::const_iterator it = slices.begin();
	list<ofxTrackerTimeSlice*>::const_iterator prev = NULL;
	float slice_x = x;
	float slice_y = y;
	while(it != slices.end()) {
		(*it)->draw();
		if(prev != NULL) {
			(*prev)->drawPrev((*it)->x, (*it)->y);
		}
		slice_x += (*it)->width + 5.0f;
		prev = it;
		++it;
	}
}


void ofxTracker::reposition() {
	list<ofxTrackerTimeSlice*>::reverse_iterator it = slices.rbegin();
	int i = 0;
	while(it != slices.rend()) {
		(*it)->x = ((*it)->width * i) + 5*i;
		(*it)->y = y;
		++it;
		++i;
	}
}

void ofxTracker::getBlobsFromHistoryWithFlag(int nFlag,vector<ofxTrackerBlob*>&rFoundBlobs) {
	list<ofxTrackerTimeSlice*>::reverse_iterator slice_it = slices.rbegin();
	while(slice_it != slices.rend()) {
		(*slice_it)->getBlobsWithFlag(nFlag, rFoundBlobs);
		++slice_it;
	}
}
