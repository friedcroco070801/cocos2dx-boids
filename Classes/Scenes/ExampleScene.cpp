#include "ExampleScene.h"
USING_NS_CC;

ExampleScene* ExampleScene::create() {
    ExampleScene *ret = new (std::nothrow) ExampleScene;
    if (ret && ret->init())
    {
        ret->autorelease();
        return ret;
    }
    else
    {
        CC_SAFE_DELETE(ret);
        return nullptr;
    }
}

bool ExampleScene::init() {

    return true;
}