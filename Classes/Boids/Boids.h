#ifndef __BOIDS_H__
#define __BOIDS_H__

#include "cocos2d.h"
#include <list>
#include <functional>

namespace boids {

class Boids;

class Boid : public cocos2d::Node {
    friend class Boids;
    cocos2d::Node* node;
    float limitSpeed;
    float x, y, dx, dy;
public:
    cocos2d::Node*& getNode() {return node;}
    static Boid* create(cocos2d::Node* node, float limitSpeed, float x, float y, float dx, float dy);
    bool init(cocos2d::Node* node, float limitSpeed, float x, float y, float dx, float dy);
    void update();
    float getDistanceTo(Boid* other);
};

class Boids : public cocos2d::Node {
    std::list<Boid*> boids;
    bool isRunning;
    cocos2d::Rect boundary;
    float limitSpeed, radius, mininumDist;
    cocos2d::Vec2 updateSpeedCoherence(Boid* target);
    cocos2d::Vec2 updateSpeedSeperation(Boid* target);
    cocos2d::Vec2 updateSpeedAlignment(Boid* target);
public:
    static Boids* create(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, std::function<float()> x, std::function<float()> y, std::function<float()> dx, std::function<float()> dy);
    static Boids* create(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist);
    bool init(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, std::function<float()> x, std::function<float()> y, std::function<float()> dx, std::function<float()> dy);
    void setParent(cocos2d::Node* parent);
    void removeFromParent();
    void update();
};

};
#endif // !__BOIDS_H__