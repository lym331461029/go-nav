#ifndef _navigation_h_
#define _navigation_h_

#ifndef NavigationAPI
#ifdef _WIN32
#	define NavigationAPI __declspec(dllimport)
#else
#   define NavigationAPI
#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y, z;
}navigation_position_t;

typedef struct navigation_sdk{
//        Sample *sample;
//        std::map<int,std::vector<navigation_position_t> > resultCache;
//        int cacheSeq;
} *navigation_sdk_t;

NavigationAPI navigation_sdk_t navigation_sdk_create(int meshType);
//NavigationAPI navigation_sdk_t navigation_sdk_create();
NavigationAPI int  LoadMap(char *path, navigation_sdk_t sdk);
NavigationAPI void UnloadMap(navigation_sdk_t sdk);
NavigationAPI int  FindPath(navigation_position_t start, navigation_position_t end, navigation_sdk_t sdk);
NavigationAPI int  GetValue(int index, navigation_sdk_t sdk, navigation_position_t* pos);
NavigationAPI int FindPath2(navigation_position_t start, navigation_position_t end, navigation_sdk_t sdk, navigation_position_t **ret, int *retLen);
NavigationAPI int  FreePathCache(navigation_sdk_t sdk, int cacheSeq);

typedef unsigned int uint;
NavigationAPI int  AddObstacle(navigation_sdk_t sdk, navigation_position_t pos,const float radius,const float height,uint* result);
NavigationAPI int  RemoveObstacle(navigation_sdk_t sdk, uint refNo);
#ifdef __cplusplus
}
#endif

#endif