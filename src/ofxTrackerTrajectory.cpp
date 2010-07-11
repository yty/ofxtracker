#include "ofxTrackerTrajectory.h"
#include "ofxTracker.h"
#include "ofxTrackerBlob.h"
#include "ofxTrackerBlobMatch.h"
#include "ofxTrackerTimeSlice.h"

#include <algorithm>


int ofxTrackerTrajectory::next_id = 0;

ofxTrackerTrajectory::ofxTrackerTrajectory(ofxTracker* pTracker)
:tracker(pTracker) 
{
}

// Try to find trajectories and matching blobs
void ofxTrackerTrajectory::findTrajectories() {
	list<ofxTrackerTimeSlice*>::reverse_iterator it = tracker->slices.rbegin();
	vector<ofxTrackerBlob*>::iterator blob_it = (*it)->tracked_blobs.begin();
	int match_count = 0;
	int num_blobs = (*it)->tracked_blobs.size();
	vector<int>assigned_ids;
	cout << "\nFRAME " << (*it)->frame_num << std::endl;
	
	// first frame (this can be optimized a bit)
	if((*it)->prev == NULL) {
		while(blob_it != (*it)->tracked_blobs.end()) {
			if(!(*blob_it)->isMatched())  {
				(*blob_it)->setID(getNextID());
				match_count++;
			}		
			blob_it++;
		}
	}
	else {
		// start with the best matches, and give them an ID first.
		vector<ofxTrackerBlobMatch*>all_matches = (*it)->getMatches();
		vector<ofxTrackerBlobMatch*>::iterator match_it = all_matches.begin();
		vector<ofxTrackerBlob*> unmatched_blobs;

		
		while(match_it != all_matches.end()) {
			ofxTrackerBlobMatch* match = (*match_it);
			if(!match->blob->isMatched() && !match->owner->hasID() && match->k != 0) {
				match->blob->setMatched(true);
				match->owner->setMatchingBlob(match->blob);
				match->owner->setID(match->blob->getID());
				match->owner->calculateVelocity(match->blob);
				assigned_ids.push_back(match->owner->getID());
				cout << ">> give id: " << match->owner->getID() << std::endl;
				match_count++;
			}
			else {
				// somehow the unmatched_blob contains more than the number of available blobs;
				// we need to change this!!
				if(unmatched_blobs.size() == 0) {
					unmatched_blobs.push_back(match->owner);
				}
				else {
					bool already_stored = false;
					for(int i = 0; i < unmatched_blobs.size(); ++i) {
						if(unmatched_blobs.at(i) == match->owner) {
							already_stored = true;
							break;
						}
					}
					if(!already_stored) 
						unmatched_blobs.push_back(match->owner);
				}
			}
			match_it++;
		}
		// cleanup the unmatched blobs
		vector<ofxTrackerBlob*>::iterator clean_it = unmatched_blobs.begin();
		while(clean_it != unmatched_blobs.end()) {
			if( (*clean_it)->hasID()) {
				cout << "ERASE\n";
				clean_it = unmatched_blobs.erase(clean_it);
			}
			else {
				++clean_it;
			}
		}
		
		// didn't we match all blobs?
		if(match_count < num_blobs) { // there are unlabelled blobs
			cout << ">> num blobs:" 
				 << num_blobs
				 << " match count: " 
				 << match_count 
				 << "\n unmatched_blobs.size(): " << unmatched_blobs.size()
				 << std::endl;
				 
			// get unlabelled blobs, find blobs in history which left the frame
			vector<ofxTrackerBlob*>::const_iterator unmatched_blob_it = unmatched_blobs.begin();
			vector<ofxTrackerBlob*>flagged_blobs;
		
			while(unmatched_blob_it != unmatched_blobs.end()) {
				cout	<< "Unmatched (?): "
						<< (*unmatched_blob_it)->hasID() << ", ID: "	
						<< (*unmatched_blob_it)->getID() << ", p: "
						<< (*unmatched_blob_it) << std::endl;
				unmatched_blob_it++;
			}
					
			tracker->getBlobsFromHistoryWithFlag(FLAG_OUT_OF_FRAME, flagged_blobs);
			vector<ofxTrackerBlob*>::const_iterator out_of_frame_it = flagged_blobs.begin();
			while(out_of_frame_it != flagged_blobs.end()) {
				(*out_of_frame_it)->setMatched(false);
				++out_of_frame_it;
			}
			cout << ">> flagged blobs: " << flagged_blobs.size() << std::endl;
			if(flagged_blobs.size() > 0) {
				unmatched_blob_it = unmatched_blobs.begin();
				while(unmatched_blob_it != unmatched_blobs.end()) {
					out_of_frame_it = flagged_blobs.begin();
					vector<ofxTrackerBlobMatch*>k2_matches;
					while(out_of_frame_it != flagged_blobs.end()) {
						cout << "Is matched: " << (*out_of_frame_it)->isMatched() << ", ID: " << (*out_of_frame_it)->getID() << std::endl;
						if(!(*out_of_frame_it)->isMatched()) {
							float confidence = (*unmatched_blob_it)->calculateK2((*out_of_frame_it));
							if(confidence > 0) {
								ofxTrackerBlobMatch* out_of_frame_match = new ofxTrackerBlobMatch(
									(*out_of_frame_it)
									,(*unmatched_blob_it)
									,confidence
								);
								k2_matches.push_back(out_of_frame_match);
							}
						}
						++out_of_frame_it;
					}
					
					// sort on out of frame matches.
					if(k2_matches.size() > 0) {
						std::sort(k2_matches.begin(), k2_matches.end(), blob_match_compare_k);
						cout << ">> blob came back into the screen, give it this id:" << k2_matches.at(0)->blob->getID();
						k2_matches.at(0)->blob->setMatched(true);
						k2_matches.at(0)->owner->setMatchingBlob(k2_matches.at(0)->blob);
						k2_matches.at(0)->owner->setID(k2_matches.at(0)->blob->getID());
						match_count++;
						assigned_ids.push_back(k2_matches.at(0)->owner->getID());
					}
					++unmatched_blob_it;
				}
			}
			else {
				// There are no flagged (OUT_OF_FRAME) blobs in the history
				// but we do have a new blob. Now we need to figure out which 
				// ID is unnused and assign that to the new blob. Or, when
				// all IDs are used, we need to create a new one.
				vector<int>::iterator assigned_it = assigned_ids.begin();
				vector<int>free_ids;
				std::sort(assigned_ids.begin(), assigned_ids.end());
				back_insert_iterator<vector<int> > id_back_iter(free_ids);
				set_difference(used_ids.begin(),used_ids.end(),assigned_ids.begin(),assigned_ids.end(),id_back_iter);
				vector<int>::iterator free_ids_it = free_ids.begin();
				unmatched_blob_it = unmatched_blobs.begin();
				while(unmatched_blob_it != unmatched_blobs.end()) {
					if(free_ids_it != free_ids.end() && !(*unmatched_blob_it)->hasID()) {
						(*unmatched_blob_it)->setID((*free_ids_it));
						(*unmatched_blob_it)->setMatched(true);
						cout << ">> Use free ID: " << (*free_ids_it) << std::endl;
						assigned_ids.push_back((*free_ids_it));
						++free_ids_it;

						match_count++;
					}
					unmatched_blob_it++;
				}
				//return;
			}
		}
	}


	// check if there are new blobs, or some blobs left the frame.
	if(it != tracker->slices.rend()) {
		ofxTrackerTimeSlice* curr_slice = (*it);
		++it;
		if(it == tracker->slices.rend()) {
			//cout << "We are at the end\n";
			return;
		}
		ofxTrackerTimeSlice* prev_slice = (*it);
		cout	<< ">> frame: " << curr_slice->frame_num  << " <> "
				<< prev_slice->frame_num 
				<< " blobs: " << curr_slice->tracked_blobs.size() 
				<< " <> " << prev_slice->tracked_blobs.size() 
				<< std::endl;
				
		vector<ofxTrackerBlob*> slice_difference; 	
		

		// @todo: I'm not sure if I need to sort the tracked_blobs on ID first,
		// have to reread set_difference() as someone posted on stackoverflow
		// I should sort first.
		// CONFIRMED: I do need to sort on ID here!
		back_insert_iterator<vector<ofxTrackerBlob*> > back_iter(slice_difference);
		vector<ofxTrackerBlob*> prev_slice_blobs = prev_slice->getTrackedBlobsSortedOnID();
		vector<ofxTrackerBlob*> curr_slice_blobs = curr_slice->getTrackedBlobsSortedOnID();
	
		set_difference(
			 prev_slice_blobs.begin()
			,prev_slice_blobs.end()
			,curr_slice_blobs.begin()
			,curr_slice_blobs.end()
			,back_iter
			,set_difference_on_id
		);
		
		// --------
		for(int i = 0; i < prev_slice_blobs.size(); ++i) {
			cout << "+++ in prev: "<< prev_slice_blobs.at(i)->getID() << std::endl;
		}
		cout << "======" << std::endl;
		for(int i = 0; i < curr_slice_blobs.size(); ++i) {
			cout << "+++ in curr: "<< curr_slice_blobs.at(i)->getID() << std::endl;
		}
		cout << "======" << std::endl;
		for(int i = 0; i < slice_difference.size(); ++i) {
			cout << "+++ difference: "<< slice_difference.at(i)->getID() << std::endl;
		}
		cout << "======" << std::endl << std::endl;

		// --------

		vector<ofxTrackerBlob*>::const_iterator not_matched_it = slice_difference.begin();
		while(not_matched_it != slice_difference.end()) {
			cout	<< ">> blob: " << (*not_matched_it)->getID() 
					<< " on frame:" << (*not_matched_it)->slice->frame_num
					<< " flagged for out of frame." << std::endl;
					
			(*not_matched_it)->flag |= FLAG_OUT_OF_FRAME;
			++not_matched_it;
		}

							
		// new blob in the current frame
		if (prev_slice->tracked_blobs.size() < curr_slice->tracked_blobs.size()) {
			// this is a more interesting "problem". We will got back in time and
			// check if there is a blob which wsa flagged as FLAG_OUT_OF_FRAME.
			// then we calculate the k2 confidence value and check if we can 
			// re-label the blob when it looks like it's the same blob.
			back_insert_iterator<vector<ofxTrackerBlob*> > back_iter(slice_difference);
			set_difference(
				curr_slice->tracked_blobs.begin()
				,curr_slice->tracked_blobs.end()
				,prev_slice->tracked_blobs.begin()
				,prev_slice->tracked_blobs.end()
				,back_iter
				,set_difference_on_id
			);
			
			vector<ofxTrackerBlob*>flagged_blobs;
			tracker->getBlobsFromHistoryWithFlag(FLAG_OUT_OF_FRAME, flagged_blobs);
			cout	<< ">> found: " << flagged_blobs.size() << " number of flagged blobs"
					<< " and slice_difference: " << slice_difference.size() << std::endl;
			
			// for each new blob, calculate k2 on the flagged blobs
			vector<ofxTrackerBlob*>::const_iterator new_blobs_it = slice_difference.begin();
			vector<ofxTrackerBlob*>::const_iterator flagged_blobs_it = flagged_blobs.begin();
		
			while(flagged_blobs_it != flagged_blobs.end()) {
				(*flagged_blobs_it)->setMatched(false);
				++flagged_blobs_it;
			}
			
			
			while(new_blobs_it != slice_difference.end()) {
				flagged_blobs_it = flagged_blobs.begin();
				vector<ofxTrackerBlobMatch*>k2_matches;
				while(flagged_blobs_it != flagged_blobs.end()) {
					if(!(*flagged_blobs_it)->isMatched()) {
						float confidence = (*new_blobs_it)->calculateK2((*flagged_blobs_it));
						cout	<< ">> got confidence: " << confidence 
								<< " with id: " << (*flagged_blobs_it)->getID() 
								<< std::endl;
						if(confidence > 0) {
							(*flagged_blobs_it)->flag = 0;		
							ofxTrackerBlobMatch* k2_match = new ofxTrackerBlobMatch(
								(*flagged_blobs_it)
								,(*new_blobs_it)
								,confidence
							);
							k2_matches.push_back(k2_match);
						}
						
					}
					++flagged_blobs_it;
				}
				// sort on confidences and check the best matched value.
				if(k2_matches.size() > 0) {
					std::sort(k2_matches.begin(), k2_matches.end(), blob_match_compare_k);
					k2_matches.at(0)->blob->setMatched(true);
					k2_matches.at(0)->owner->setMatchingBlob(k2_matches.at(0)->blob);
					k2_matches.at(0)->owner->setID(k2_matches.at(0)->blob->getID());
					match_count++;
				}
				++new_blobs_it;
			}
		}
	}
	
	// when all matching failed, we need to assign new (available ids)
	if(match_count < num_blobs) {
		vector<int>::iterator assigned_it = assigned_ids.begin();
		vector<int>free_ids;
		std::sort(assigned_ids.begin(), assigned_ids.end());
		back_insert_iterator<vector<int> > id_back_iter(free_ids);
		set_difference(used_ids.begin(),used_ids.end(),assigned_ids.begin(),assigned_ids.end(),id_back_iter);

		vector<int>::iterator free_ids_it = free_ids.begin();
		blob_it = tracker->slices.back()->tracked_blobs.begin();
		while(blob_it != tracker->slices.back()->tracked_blobs.end()) {
			if(free_ids_it != free_ids.end()) {
				(*blob_it)->setID((*free_ids_it));
				++free_ids_it;
				match_count++;
			}
			else {
				// new id!
				(*blob_it)->setID(getNextID());
				match_count++;
			}
			++blob_it;
		} 
		// @todo check if we could reuse all the ids or we need to generate some new ids.
	}
	
	cout << "\n----------------- match count: " << match_count << " num_blobs: " << num_blobs << "----------------------------------------\n\n";
}
int ofxTrackerTrajectory::getNextID() {
	cout << "-- store ID: " << next_id << std::endl;
	used_ids.push_back(next_id);
	return next_id++;
}