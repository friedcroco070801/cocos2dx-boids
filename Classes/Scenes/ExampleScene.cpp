#include "ExampleScene.h"
#include <functional>
USING_NS_CC;
using namespace std;
using namespace boids;

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
    auto visibleSize = Director::getInstance()->getVisibleSize();
    auto origin = Director::getInstance()->getVisibleOrigin();

    auto ground = Sprite::create("sprites/seafloor.jpg");
    this->addChild(ground, -1);
    ground->setPosition(visibleSize.width / 2 + origin.x, visibleSize.height / 2 + origin.y);
    ground->setColor(Color3B(150, 150, 255));

    auto fore = Sprite::create("sprites/black.png");
    this->addChild(fore, -1);
    fore->setPosition(visibleSize.width / 2 + origin.x, visibleSize.height / 2 + origin.y);
    fore->setOpacity(160);

    function<Node*()> singleBoid = []() -> Node* {
        auto sprite = Sprite::create("sprites/fish.png", Rect(0.0f, 0.0f, 32.0f, 32.0f));
        Vector<SpriteFrame*> frames;
        for (int i = 0; i < 4; i++) {
            auto frame = SpriteFrame::create("sprites/fish.png", Rect(32.0f * i, 0.0f, 32.0f, 32.0f));
            frames.pushBack(frame);
        }
        auto animation = Animation::createWithSpriteFrames(frames, 0.05f + 0.075f * CCRANDOM_0_1());
        auto animate = RepeatForever::create(Animate::create(animation));
        sprite->runAction(animate);
        sprite->setScale(0.8f);
        return sprite;
    };
    boids = Boids::create(Rect(origin.x + 20.0f, origin.y + 20.0f, visibleSize.width - 40.0f, visibleSize.height - 40.0f), 150, singleBoid, 6.0f, 100.0f, 20.0f);
    this->addChild(boids, 1);

    auto delay = DelayTime::create(0.01f);
    auto update = CallFunc::create([this](){
        return [&](){
            this->boids->update();
        };
    }());
    auto seq = Sequence::create(delay, update, nullptr);
    this->runAction(RepeatForever::create(seq));

    return true;
}