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
    void limitSpeedToMax();
public:
    cocos2d::Node*& getNode() {return node;}
    static Boid* create(cocos2d::Node* node, float limitSpeed, float x, float y, float dx, float dy);
    bool init(cocos2d::Node* node, float limitSpeed, float x, float y, float dx, float dy);
    void update();
    float getDistanceTo(Boid* other);
};

class Boids : public cocos2d::Node {
    std::vector<Boid*> boids;
    bool isRunning;
    cocos2d::Rect boundary;
    float limitSpeed, radius, mininumDist;
    float coherenceCoef, seperationCoef, alignmentCoef;
    float boundaryFactor;
    cocos2d::Vec2 updateSpeedCoherence(Boid* target);
    cocos2d::Vec2 updateSpeedSeperation(Boid* target);
    cocos2d::Vec2 updateSpeedAlignment(Boid* target);
    cocos2d::Vec2 updateSpeedBoundary(Boid* target);
public:
    static Boids* create(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, std::function<float()> x, std::function<float()> y, std::function<float()> dx, std::function<float()> dy);
    static Boids* create(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist);
    Boids(cocos2d::Rect boundary, int population, std::function<cocos2d::Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, std::function<float()> x, std::function<float()> y, std::function<float()> dx, std::function<float()> dy);
    void setParentForBoids(cocos2d::Node* parent);
    void removeFromParent();
    void update();
};

};
#endif // !__BOIDS_H__