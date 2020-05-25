#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include "common/object.h"
#include "util/log.h"
#include <memory>
#include <mutex>

namespace glb_auto_perception_sensorfusion
{
class SortedSensorFramePoolManager {
   private:
    std::mutex mutex_;
    static const int MAX_POOL_SIZE = 100;
    std::vector<std::shared_ptr<SensorObjects> > originalSensorFramePtrList_;
    std::vector<std::shared_ptr<SensorObjects> > snapShotSensorFramePtrList_;
    
    static std::unique_ptr<SortedSensorFramePoolManager> singleton_;

   public:
    static SortedSensorFramePoolManager* getInstance();

    bool addNewFrame(std::shared_ptr<SensorObjects>  frame);

    std::shared_ptr<SensorObjects>  getClosestFrame(uint64_t &);

    void clearAllFramesFromSnapShot();

    std::vector<std::shared_ptr<SensorObjects> > copyAndClearAllFramesPtrList();
    void copyAndMoveFramesPtrList(uint64_t timestamp);
    std::shared_ptr<SensorObjects> getNewestFrame(); //get newest frame;


    std::vector<std::shared_ptr<SensorObjects> > getAllFramesPtrListSnapshot();

    void sortingAll(std::vector<std::shared_ptr<SensorObjects> > &);

    void sortingAll();

   private:
    SortedSensorFramePoolManager() {}  
    SortedSensorFramePoolManager(SortedSensorFramePoolManager const &);  
    void operator=(SortedSensorFramePoolManager const &);                

    static void create();

};

}
#endif // DATA_MANAGER_H_