#include "data_manager/data_manager.h"
#include <algorithm>

namespace glb_auto_perception_sensorfusion {

std::unique_ptr<SortedSensorFramePoolManager> SortedSensorFramePoolManager::singleton_;
static std::once_flag once_control;
static uint64_t BUFFERED_POOL_DURATION_LENGTH = 500000000;  // Set the maximum duration pool to be 500 ms

bool compareTimestamp(std::shared_ptr<SensorObjects>  p1, std::shared_ptr<SensorObjects>  p2) { return (p1->timestamp > p2->timestamp); }

void SortedSensorFramePoolManager::create() { singleton_.reset(new SortedSensorFramePoolManager()); }

SortedSensorFramePoolManager* SortedSensorFramePoolManager::getInstance() {
    std::call_once(once_control, SortedSensorFramePoolManager::create);
    return singleton_.get();
}

bool SortedSensorFramePoolManager::addNewFrame(std::shared_ptr<SensorObjects>  sfPtr) {
    if (sfPtr == nullptr) {
        AERROR << "Can't add a nullptr sensorobjects";
    }
    std::lock_guard<std::mutex> guard(mutex_);
    if (originalSensorFramePtrList_.size() == MAX_POOL_SIZE) {
        //originalSensorFramePtrList_.erase(originalSensorFramePtrList_.begin());
        originalSensorFramePtrList_.erase(originalSensorFramePtrList_.end()-1);
    }
    originalSensorFramePtrList_.push_back(sfPtr);
    sortingAll();
    AINFO << "addNewFrame: size is: " << originalSensorFramePtrList_.size();

    return true;
}

void SortedSensorFramePoolManager::SortedSensorFramePoolManager::clearAllFramesFromSnapShot() {
    std::lock_guard<std::mutex> guard(mutex_);
    snapShotSensorFramePtrList_.clear();
}

/*
 * This function copies those frames that is a few cycle behind the latest localization timestamp
 * and leave the newer frames in the pool for the next time process
 */
void SortedSensorFramePoolManager::copyAndMoveFramesPtrList(uint64_t timestamp) {
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<std::shared_ptr<SensorObjects> >::iterator separator;
    //      AINFO << "copyAndMoveFramesPtrList timestamp = " << timestamp;
    //      for (std::vector<std::shared_ptr<SensorObjects> >::iterator it = originalstd::shared_ptr<SensorObjects> List_.begin();
    //           it != originalstd::shared_ptr<SensorObjects> List_.end(); it++) {
    //        AINFO << "copyAndMoveFramesPtrList originalList: " << (*it)->timestamp;
    //      }

    std::shared_ptr<SensorObjects>  framePtr = nullptr;
    if (originalSensorFramePtrList_.size() == 0) {
        AINFO << "copyAndMoveFramesPtrList: size is 0";
        return;
    }
    if (timestamp > originalSensorFramePtrList_.front()->timestamp) {
        AINFO << "copyAndMoveFramesPtrList: time is newer than the newest frame, " << timestamp << ":"
                    << originalSensorFramePtrList_.front()->timestamp;
        return;
    } else if (timestamp < originalSensorFramePtrList_.back()->timestamp) {
        AINFO << "copyAndMoveFramesPtrList: time is older than the oldest frame, " << timestamp << ":"
                    << originalSensorFramePtrList_.back()->timestamp;
        return;
    }

    for (separator = originalSensorFramePtrList_.end() - 1; separator != originalSensorFramePtrList_.begin() - 1;
         separator--) {
        framePtr = *separator;
        if (timestamp < framePtr->timestamp) {
            break;
        }
    }

    // if (separator == originalSensorFramePtrList_.begin()) {
    //  return;
    //}

    if (framePtr == nullptr) {
        return;
    }

    uint64_t separateTime = framePtr->timestamp;
    snapShotSensorFramePtrList_ = std::vector<std::shared_ptr<SensorObjects> >(separator + 1, originalSensorFramePtrList_.end());
    originalSensorFramePtrList_.resize(originalSensorFramePtrList_.size() - snapShotSensorFramePtrList_.size());

    AINFO << "separate time: " << separateTime;
    AINFO << "snapShotList: front->" << snapShotSensorFramePtrList_.front()->timestamp << ", back->"
                << snapShotSensorFramePtrList_.back()->timestamp;
    if (snapShotSensorFramePtrList_.size() != 0) {
        for (std::vector<std::shared_ptr<SensorObjects> >::iterator it = snapShotSensorFramePtrList_.begin();
             it != snapShotSensorFramePtrList_.end();) {
            std::shared_ptr<SensorObjects>  temp = *it;
            AINFO << "snapShotSensorFramePtrList: timestamp: " << temp->timestamp;
            if (temp->timestamp < separateTime - BUFFERED_POOL_DURATION_LENGTH) {
                snapShotSensorFramePtrList_.erase(it);
                AINFO << "copyAndMoveFramesPtrList: removed one frame";
            } else {
                it++;
            }
        }
        AINFO << "copyAndMoveFramesPtrList final originalSensorFramePtrList_.size = "
                    << originalSensorFramePtrList_.size();
        AINFO << "copyAndMoveFramesPtrList final snapShotSensorFramePtrList_.size = "
                    << snapShotSensorFramePtrList_.size();
    }
    //  AINFO << "Final orignalSensorFramePtrList size = " << originalSensorFramePtrList_.size();
}

std::shared_ptr<SensorObjects> SortedSensorFramePoolManager::getNewestFrame(){
    std::lock_guard<std::mutex> guard(mutex_);
    if (originalSensorFramePtrList_.size() == 0) {
        AINFO << "copyAndMoveFramesPtrList: size is 0";
        return nullptr;
    }else
    {
        return originalSensorFramePtrList_.at(0);
        AINFO << "Get Newest Frame's size is: " << originalSensorFramePtrList_.at(0)->objects.size();
    }
    
}

std::vector<std::shared_ptr<SensorObjects> > SortedSensorFramePoolManager::getAllFramesPtrListSnapshot() {
    return snapShotSensorFramePtrList_;
}

std::shared_ptr<SensorObjects>  SortedSensorFramePoolManager::getClosestFrame(uint64_t &timeStamp) {
    std::shared_ptr<SensorObjects>  ptr = nullptr;

    if (snapShotSensorFramePtrList_.size() != 0) {
        for (std::vector<std::shared_ptr<SensorObjects> >::iterator it = snapShotSensorFramePtrList_.begin();
             it != snapShotSensorFramePtrList_.end(); it++) {
            ptr = *it;
            if (ptr->timestamp > timeStamp) {
                break;
            }
        }
    }
    return ptr;
}

void SortedSensorFramePoolManager::sortingAll() {
    sort(originalSensorFramePtrList_.begin(), originalSensorFramePtrList_.end(), compareTimestamp);
} 
}