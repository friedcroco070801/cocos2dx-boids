#ifndef __EXAMPLE_SCENE_H__
#define __EXAMPLE_SCENE_H__

#include "cocos2d.h"

class ExampleScene : public cocos2d::Scene {
public:
    static ExampleScene* create();
    bool init();
    virtual ~ExampleScene(){};
};

#endif // !__EXAMPLE_SCENE_H__