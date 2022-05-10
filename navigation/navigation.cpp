#include <vector>

#include "Sample_SoloMesh.h"
#include "Sample_TempObstacles.h"
#include "DetourNavMeshQuery.h"
//#include "NavMeshTesterTool.h"
#include "InputGeom.h"


#include <map>
#include <vector>
#include "stdio.h"
#include "stdlib.h"

#ifndef NavigationAPI
#ifdef _WIN32
#	define NavigationAPI __declspec(dllexport)
#else
#	define NavigationAPI
#endif
#endif

#define SAFE_DEL_PTR(p) if (p) delete p;\
p = nullptr

extern "C" {

    
    typedef struct {
        float x, y, z;
    }navigation_position_t;


    
    typedef struct navigation_sdk{
        Sample *sample;
        std::map<int,std::vector<navigation_position_t> > resultCache;
        int cacheSeq;
    } *navigation_sdk_t;


    
    NavigationAPI navigation_sdk_t navigation_sdk_create(int meshType) {
        navigation_sdk_t sdk = new navigation_sdk();
        if (meshType == 1) {
            sdk->sample = new Sample_SoloMesh();
        } else  {
            sdk->sample = new Sample_TempObstacles();
        }
        sdk->cacheSeq = 1000;
        srand(time(NULL));
        return sdk;
    }


    
    NavigationAPI int LoadMap(char *path, navigation_sdk_t sdk) {
        if (!sdk) return -1;

        Sample* sample = sdk->sample;
        InputGeom* geom = sample->getInputGeom();
        if (geom != nullptr) {
            SAFE_DEL_PTR(geom);
        }

        BuildContext* ctx = sample->getContext();
        if (ctx != nullptr) {
            SAFE_DEL_PTR(ctx);
        }

        geom = new InputGeom();
        ctx = new BuildContext();
        
        if (!geom || !ctx) return -1;

        ctx->resetLog();
        if (!geom->load(ctx, path)) {
            SAFE_DEL_PTR(geom);
            SAFE_DEL_PTR(ctx);
            return -1;
        }

        sample->setContext(ctx);
        sample->handleMeshChanged(geom);
        sample->handleSettings();
        sample->handleBuild();
        return 1;
    }


    
    NavigationAPI void UnloadMap(navigation_sdk_t sdk) {
        if (!sdk) return;
        Sample *sample = sdk->sample;

        InputGeom* geom = sample->getInputGeom();
        if (geom != nullptr) {
            SAFE_DEL_PTR(geom);
        }
        
        BuildContext* ctx = sample->getContext();
        if (ctx != nullptr) {
            SAFE_DEL_PTR(ctx);
        }

        SAFE_DEL_PTR(sample);
    }

    
    NavigationAPI int FindPath(navigation_position_t start, navigation_position_t end, navigation_sdk_t sdk) {
        if (!sdk) return -1;
        Sample *sample = sdk->sample;
        float startPos[3];
        startPos[0] = start.x;
        startPos[1] = start.y;
        startPos[2] = start.z;

        float endPos[3];
        endPos[0] = end.x;
        endPos[1] = end.y;
        endPos[2] = end.z;

        float ingore = 0.0f;
        
        //printf("%s\n", "FindPath");

        sample->handleClick(&ingore, startPos, true);
        sample->handleClick(&ingore, endPos, false);
        return sample->getResultNum();
    }

    
    NavigationAPI int GetValue(int index, navigation_sdk_t sdk, navigation_position_t* pos)
    {
        if (!sdk || !pos) return -1;

        Sample *sample = sdk->sample;
        float out[3] = { 0 };
        bool ret = sample->getResultByIndex(index, out);
        if (ret == false) return -1;
        pos->x = out[0];
        pos->y = out[1];
        pos->z = out[2];
        return 1;
    }


    NavigationAPI int FindPath2(navigation_position_t start, navigation_position_t end, navigation_sdk_t sdk, navigation_position_t **ret, int *retLen)
    {
        if (!sdk) return -1;
        printf("%s\n", "FindPath2");

        Sample *sample = sdk->sample;
        //Sample *sample = reinterpret_cast<Sample*>(sdk);
        float startPos[3];
        startPos[0] = start.x;
        startPos[1] = start.y;
        startPos[2] = start.z;
    
        float endPos[3];
        endPos[0] = end.x;
        endPos[1] = end.y;
        endPos[2] = end.z;
    
        float ingore = 0.0f;

        int resultNum  = 0;
        {
//            sample->getNavMeshQuery()->init(sample->getNavMesh(), 65535);
//            NavMeshTesterTool *tt = new NavMeshTesterTool();
//            tt->init(sample);
//
//
//            tt->handleClick(&ingore, startPos, true);
//            tt->handleClick(&ingore, endPos, false);
//
//            resultNum = tt->getResultNum();
            sample->handleClick(&ingore, startPos, true);
            sample->handleClick(&ingore, endPos, false);
            resultNum = sample->getResultNum();
            
            if (resultNum > 0) {
                int cacheNum = /*rand()  * 10000 +*/ sdk->cacheSeq++;
                navigation_position_t pos;
                
                float out[3] = { 0 };
                for(int index = 0; index < resultNum; index++) {
                    bool ret = sample->getResultByIndex(index, out);
                    if(ret) {
                        pos.x = out[0];
                        pos.y = out[1];
                        pos.z = out[2];
                        sdk->resultCache[cacheNum].push_back(pos);
                    }
                }
                resultNum = cacheNum;
                *ret = sdk->resultCache[cacheNum].data();
                *retLen = sdk->resultCache[cacheNum].size();
            }
        }


        return resultNum;
    }

    
    NavigationAPI int  FreePathCache(navigation_sdk_t sdk, int cacheSeq)
    {
        if (!sdk) return -1;
        sdk->resultCache.erase(cacheSeq);
        printf("%s\n", "FreePathCache");
        return 0;
    }

    typedef unsigned int uint;
    NavigationAPI int  AddObstacle(navigation_sdk_t sdk, navigation_position_t pos,const float radius,const float height,uint* result)
    {
        if (!sdk) return -1;
        printf("%s\n", "AddObstacle");

        Sample *sample = sdk->sample;
        Sample_TempObstacles *tempObstacleSample = dynamic_cast<Sample_TempObstacles*>(sample);

        tempObstacleSample->saveNavMesh("./mesh_add_before.bin", tempObstacleSample->getNavMesh());
        
        float inputPos[3];
        inputPos[0] = pos.x;
        inputPos[1] = pos.y;
        inputPos[2] = pos.z;

        //printf("================== %f %f %f\n",inputPos[0], inputPos[1],inputPos[2] );
        
        
        tempObstacleSample->addTempObstacle(inputPos,radius,height,result);
        tempObstacleSample->handleUpdate(inputPos[0]);

        tempObstacleSample->saveNavMesh("./mesh_add_after.bin", tempObstacleSample->getNavMesh());

        
        //printf("================== %d\n",*result);
        return 0;
    }

    
    NavigationAPI int  RemoveObstacle(navigation_sdk_t sdk, uint refNo)
    {
        if (!sdk) return -1;
        printf("%s\n", "RemoveObstacle");
        Sample *sample = sdk->sample;
        Sample_TempObstacles *tempObstacleSample = dynamic_cast<Sample_TempObstacles*>(sample);

        if(tempObstacleSample->removeTempObstacleByRef(refNo) == 0)
        {
            float x = 0;
            tempObstacleSample->handleUpdate(x);
            return 0;
        }

        return -1;
    }

}
