#include "Boids.h"
#include <cmath>
USING_NS_CC;
using namespace std;

namespace boids {

// Boid class
Boid* Boid::create(Node* node, float limitSpeed, float x, float y, float dx, float dy) {
    Boid * ret = new (std::nothrow) Boid;
    if (ret && ret->init(node, limitSpeed, x, y, dx, dy))
    {
        ret->autorelease();
    }
    else
    {
        CC_SAFE_DELETE(ret);
    }
    return ret;
}

bool Boid::init(Node* node, float limitSpeed, float x, float y, float dx, float dy) {
    this->node = node;
    this->limitSpeed = limitSpeed;
    this->x = x;
    this->y = y;
    this->dx = x;
    this->dy = y;
    this->node->setPosition(x, y);

    return true;
}

float Boid::getDistanceTo(Boid* other) {
    return sqrtf((x - other->x) * (x - other->x) + (y - other->y) * (y - other->y));
}

void Boid::update() {

}

// Boids class

Boids* Boids::create(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, function<float()> x, function<float()> y, function<float()> dx, function<float()> dy) {
    Boids * ret = new (std::nothrow) Boids;
    if (ret && ret->init(boundary, population, singleBoid, limitSpeed, radius, mininumDist, x, y, dx, dy))
    {
        ret->autorelease();
    }
    else
    {
        CC_SAFE_DELETE(ret);
    }
    return ret;
}

Boids* Boids::create(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist) {
    return Boids::create(boundary, population, singleBoid, limitSpeed, radius, mininumDist, nullptr, nullptr, nullptr, nullptr);
}

bool Boids::init(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, function<float()> x, function<float()> y, function<float()> dx, function<float()> dy) {
    this->boundary = boundary;
    this->limitSpeed = limitSpeed;
    this->radius = radius;
    this->mininumDist = mininumDist;
    this->isRunning = false;
    
    // Handle non-callable functions
    if (x == nullptr) {
        x = [&]() -> float {
            return boundary.getMinX() + CCRANDOM_0_1() * (boundary.getMaxX() - boundary.getMinX());
        };
    }

    if (y == nullptr) {
        y = [&]() -> float {
            return boundary.getMinY() + CCRANDOM_0_1() * (boundary.getMaxY() - boundary.getMinY());
        };
    }

    if (dx == nullptr) {
        dx = [&]() -> float {
            return sqrtf(limitSpeed / 2) * CCRANDOM_MINUS1_1();
        };
    }

    if (dy == nullptr) {
        dy = [&]() -> float {
            return sqrtf(limitSpeed / 2) * CCRANDOM_MINUS1_1();
        };
    }

    // Initialize population
    for (auto i = 0; i < population; i++) {
        boids.push_back(Boid::create(singleBoid(), limitSpeed, x(), y(), dx(), dy()));
    }
    return true;
}

void Boids::setParent(Node* parent) {
    for (auto boid : boids) {
        parent->addChild(boid);
    }
    this->isRunning = true;
}

void Boids::removeFromParent() {
    for (auto boid : boids) {
        boid->removeFromParent();
    }
    this->isRunning = false;
}

void Boids::update() {
    for (auto boid : boids) {
        boid->update();
    }
}

Vec2 Boids::updateSpeedCoherence(Boid* target) {
    return Vec2(0.0f, 0.0f);
}

Vec2 Boids::updateSpeedSeperation(Boid* target) {
    return Vec2(0.0f, 0.0f);
}

Vec2 Boids::updateSpeedAlignment(Boid* target) {
    return Vec2(0.0f, 0.0f);
}

};