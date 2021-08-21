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
    this->dx = dx;
    this->dy = dy;
    this->node->setPosition(x, y);
    if (dx != 0.0f || dy != 0.0f) {
        auto pi = atanf(1.0f) * 4;
        auto rotation = atan2f(dy, dx) / pi * 180;
        this->node->setRotation(-rotation);
    }
    return true;
}

float Boid::getDistanceTo(Boid* other) {
    return sqrtf((x - other->x) * (x - other->x) + (y - other->y) * (y - other->y));
}

void Boid::limitSpeedToMax() {
    auto speed = sqrtf(dx * dx + dy * dy);
    dx = (dx / speed) * limitSpeed;
    dy = (dy / speed) * limitSpeed;
}

void Boid::update() {
    limitSpeedToMax();
    x += dx;
    y += dy;
    node->setPosition(x, y);
    if (dx != 0.0f || dy != 0.0f) {
        auto pi = atanf(1.0f) * 4;
        auto rotation = atan2f(dy, dx) / pi * 180;
        node->setRotation(-rotation);
    }
}

// Boids class

Boids* Boids::create(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, function<float()> x, function<float()> y, function<float()> dx, function<float()> dy) {
    auto ret = new (std::nothrow) Boids(boundary, population, singleBoid, limitSpeed, radius, mininumDist, x, y, dx, dy);
    ret->autorelease();
    return ret;
}

Boids* Boids::create(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist) {
    return Boids::create(boundary, population, singleBoid, limitSpeed, radius, mininumDist, nullptr, nullptr, nullptr, nullptr);
}

Boids::Boids(Rect boundary, int population, function<Node*()> singleBoid, float limitSpeed, float radius, float mininumDist, function<float()> x, function<float()> y, function<float()> dx, function<float()> dy) : Node() {
    this->boundary = boundary;
    this->limitSpeed = limitSpeed;
    this->radius = radius;
    this->mininumDist = mininumDist;
    this->isRunning = false;
    coherenceCoef = 0.005f;
    seperationCoef = 0.1f;
    alignmentCoef = 0.2f;
    boundaryFactor = 1.0f;
    
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
        auto boid = Boid::create(singleBoid(), limitSpeed, x(), y(), dx(), dy());
        boids.push_back(boid);
    }
}

void Boids::setParentForBoids(Node* parent) {
    parent->addChild(this);
    for (auto boid : boids) {
        
        parent->addChild(boid->node);
        parent->addChild(boid);
    }
    this->isRunning = true;
}

void Boids::removeFromParent() {
    this->Node::removeFromParent();
    for (auto boid : boids) {
        boid->node->removeFromParent();
        boid->removeFromParent();
    }
    this->isRunning = false;
}

void Boids::update() {
    for (auto boid : boids) {
        auto speedChange = updateSpeedCoherence(boid) + updateSpeedSeperation(boid) + updateSpeedAlignment(boid) + updateSpeedBoundary(boid);
        boid->dx += speedChange.x;
        boid->dy += speedChange.y;
        boid->update();  
    }
}

Vec2 Boids::updateSpeedCoherence(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    auto count = 0;
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= radius && other != target) {
            count += 1;
            res = res + Vec2(other->x, other->y);
        }
    }
    if (count > 0) {
        res = res / ((float) count);
    }
    return (res - Vec2(target->x, target->y)) * coherenceCoef;
}

Vec2 Boids::updateSpeedSeperation(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= mininumDist) {
            res = res + Vec2(target->dx, target->dy) - Vec2(other->dx, other->dy);
        }
    }
    return res * seperationCoef;
}

Vec2 Boids::updateSpeedAlignment(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    auto count = 0;
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= radius && target != other) {
            count += 1;
            res = res + Vec2(other->dx, other->dy);
        }
    }
    if (count > 0) {
        res = res / ((float) count);
    }
    return (res - Vec2(target->dx, target->dy)) * alignmentCoef;
}

Vec2 Boids::updateSpeedBoundary(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    if (target->x < boundary.getMinX()) {
        res = res + Vec2(boundaryFactor * limitSpeed, 0.0f);
    } 
    else if (target->x > boundary.getMaxX()) {
        res = res + Vec2(-boundaryFactor * limitSpeed, 0.0f);
    }
    if (target->y < boundary.getMinY()) {
        res = res + Vec2(0.0f, boundaryFactor * limitSpeed);
    } else if (target->y > boundary.getMaxY()) {
        res = res + Vec2(0.0f, -boundaryFactor * limitSpeed);
    }
    return res;
}

};