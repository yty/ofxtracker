#include "ofxTrackerBlob.h"
#include "ofxTrackerTimeSlice.h"
#include "ofxTrackerBlobMatch.h"

ofxTrackerBlob::ofxTrackerBlob(
	 ofxTrackerTimeSlice* pSlice
	,ofxCvBlob oBlob
) 
:slice(pSlice)
,cv_blob(oBlob)
,id(-1)
,is_matched(false)
,has_id(false)
,matching_blob(NULL)
,flag(0)
,position(oBlob.centroid.x, oBlob.centroid.y)
{
	max_dist = (slice->width + slice->height) / 2;
	predicted_position = position;
}

ofxTrackerBlob::~ofxTrackerBlob() {
	vector<ofxTrackerBlobMatch*>::iterator it = matches.begin();
	while(it != matches.end()) {
		delete (*it);
		++it;
	}
}

float ofxTrackerBlob::calculateK1(ofxTrackerBlob* pOther) {
	ofxVec2f new_direction = position - pOther->position;
	ofxVec2f other_direction = pOther->velocity;
	float accel = other_direction.length() - new_direction.length(); 
	new_direction.normalize();
	other_direction.normalize();
	
	// how does our current direction match up when we match 100% with the 
	// given pOther blob? When the values are near 1.0f then we are going
	// into the same direction.
	float dot = new_direction.dot(other_direction);
	float dot_measure = (dot >= 0.97) ? 1 : 0;

	//  Size dissimilarity:  | (a - b) / (a + b) |
	float s = fabs((cv_blob.area - pOther->cv_blob.area) / (cv_blob.area + pOther->cv_blob.area));
	float area_measure = 1-s;
	
	// Predicted similarity.
	ofxVec2f predicted_dir = cv_blob.centroid - pOther->predicted_position;
	float predicted_dist = predicted_dir.length();
	float predicted_measure = 1-(predicted_dist/max_dist);

	// Distance dissimilarity
	float dx = cv_blob.centroid.x - pOther->cv_blob.centroid.x;
	float dy = cv_blob.centroid.y - pOther->cv_blob.centroid.y;
	float d = sqrt(dx*dx + dy*dy)/max_dist;
	float dist_measure = 1.0f - d;
	
	// Add all base confidences
	float confidence = area_measure;
	confidence += dist_measure;
	confidence += predicted_measure;
	
	// When we are quite confident we have a match (2.5 is a value which we 
	// get when two blobs are quite similar) and the change in velocity
	// between our previous blob velocity and the one we would use when we 
	// have a 100% match with the given blob (see new_direction) we add the
	// dot measure which is 1 when the the direction vectors are parallel.
	//if(confidence >= 2.5 && fabs(accel) <= 10.0f)
	if(confidence >= 2.25 && fabs(accel) <= 20.0f)
		confidence += dot_measure;
	else if (confidence < 1.5f)
		confidence = 0; // at least we must have a bit of confidence
	cout	<< "(k1) predicted_measure: "	<< setw(10)	<< predicted_measure 
			<< " dist_measure: "		<< setw(10)	<< dist_measure 
			<< " area_measure: "		<< setw(10)	<< area_measure 
			<< " other->area: "			<< setw(5)	<< pOther->cv_blob.area 
			<< " confidence: "			<< setw(10)	<< confidence
			<< " dot: "					<< setw(5)	<< dot
			<< " accel: "				<< setw(5)	<< accel
			<< " other->id: "			<< pOther->getID() << std::endl;
//	cout << "Confidence: " << confidence << std::endl;	
	return confidence;
}

/**
 * K2 is calculated in special cases, i.e. when we found a new blob, and 
 * we have another blob from the history which went out of sight. See
 * ofxTrackerTrajectory.
 *
 */
float ofxTrackerBlob::calculateK2(ofxTrackerBlob* pOther) {
	//  Size dissimilarity:  | (a - b) / (a + b) |
	float s = fabs((cv_blob.area - pOther->cv_blob.area) / (cv_blob.area + pOther->cv_blob.area));
	float area_measure = 1-s;
	
	// Distance dissimilarity
	float dx = cv_blob.centroid.x - pOther->cv_blob.centroid.x;
	float dy = cv_blob.centroid.y - pOther->cv_blob.centroid.y;
	float d = sqrt(dx*dx + dy*dy)/max_dist;
	float dist_measure = 1.0f - d;
	
	float confidence = area_measure;
	confidence += dist_measure;
	
	
	cout	<< "(k2) dist_measure: "	<< setw(10)	<< dist_measure 
			<< " area_measure: "		<< setw(10)	<< area_measure 
			<< " other->area: "			<< setw(5)	<< pOther->cv_blob.area 
			<< " confidence: "			<< setw(10)	<< confidence
			<< " other->id: "			<< setw(5) << pOther->getID() 
			<< " other->frame: "		<< setw(5) << pOther->slice->frame_num
			<< std::endl;
	return confidence;
}

void ofxTrackerBlob::calculateVelocity(ofxTrackerBlob* pBlob) {
	if(pBlob == NULL)
		return;
	velocity = cv_blob.centroid - pBlob->cv_blob.centroid ;
	predicted_position = position + velocity;
}

void ofxTrackerBlob::draw() {
	glColor3f(1.0f, 1.0f, 0.0f);
	ofDrawBitmapString(ofToString(id), getAbsX()-15, getAbsY()+15);
}

void ofxTrackerBlob::drawMatches(float fChange) {
	// Draw numeric matching values.
	vector<ofxTrackerBlobMatch*>::const_iterator it = matches.begin();
	while(it != matches.end()) {
		if((*it) == NULL) {
			++it;
			continue;
		}
		ofDrawBitmapString(ofToString((*it)->k,2), (*it)->blob->getAbsX()-10, (*it)->blob->getAbsY()-10 -fChange);
		++it;
	}
	
	// draw color for match values 
	ofFill();
	ofCircle(getAbsX(), getAbsY() + 20, 3);
	
	// draw prediction velocity
	glLineWidth(2.5f);
	glBegin(GL_LINES);
		glColor4f(0.0f, 1.0f, 1.0f, 1.0f);		
		glVertex2f(getAbsX(), getAbsY());
		glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
		glVertex2f(velocity.x + getAbsX(), velocity.y + getAbsY());
	glEnd();
	glLineWidth(2.0f);	
		
	// draw a circle at the center position of the previous position.
	glColor3f(0.1f, 0.4f, 0.8f);
	ofFill();
	ofCircle(velocity.x + getAbsX(), velocity.y + getAbsY(),2);
	
	// draw predicted position
	glPushMatrix();
		glTranslatef(slice->x, slice->y,0);
		glColor3f(1.0f, 0.0f, 0.6f);
		ofCircle(predicted_position.x, predicted_position.y, 2);
	glPopMatrix();
}

ofxTrackerBlobMatch* ofxTrackerBlob::getBestAvailableMatch() {
	vector<ofxTrackerBlobMatch*>::const_iterator it = matches.begin();
	while(it != matches.end()) {
		if(!(*it)->blob->isMatched()) {
			return (*it);
		}
		++it;
	}
	return NULL;
}

void ofxTrackerBlob::deleteMatches() {
	// matches as the blobs do not exist anymore.
	matches.clear();
	matching_blob = NULL;
}

bool ofxTrackerBlob::hasID() {
	return has_id;
}

int ofxTrackerBlob::getID() const {
	return id;
}

// are we matched with another blob? The first frame is not matched directly but the blobs get an id
bool ofxTrackerBlob::isMatched() {
	return is_matched;
}

void ofxTrackerBlob::setMatched(bool bIsMatched) {
	is_matched = bIsMatched;
	flag = 0; // when matched, we are back into the view!
}

void ofxTrackerBlob::setMatchingBlob(ofxTrackerBlob* pMatched) {
	matching_blob = pMatched;
}


void ofxTrackerBlob::setID(int nID) {
	id = nID;
	has_id = true;
}

float ofxTrackerBlob::getAbsX() {
	return slice->x + cv_blob.centroid.x;
}
	

float ofxTrackerBlob::getAbsY() {
	return slice->y + cv_blob.centroid.y;
}