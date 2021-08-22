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
    coherenceCoef = COHERENCE_COEF;
    seperationCoef = SEPERATION_COEF;
    alignmentCoef = ALIGNMENT_COEF;
    boundaryFactor = BOUNDARY_FACTOR;
    
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

void Boids::setParent(Node* parent) {
    _parent = parent;
    _transformUpdated = _transformDirty = _inverseDirty = true;

    for (auto boid : boids) {
        if (parent != nullptr) {
            parent->addChild(boid->node, this->getLocalZOrder());
            parent->addChild(boid, this->getLocalZOrder());
        }
    }
    this->isRunning = true;
}

void Boids::removeFromParent() {
    for (auto boid : boids) {
        boid->node->removeFromParent();
        boid->removeFromParent();
    }
    this->Node::removeFromParent();
    this->isRunning = false;
}

void Boids::update() {
    for (auto boid : boids) {
        Vec2 speedChange = Vec2(0.0f, 0.0f);
        speedChange = speedChange + updateSpeedCoherence(boid);
        speedChange = speedChange + updateSpeedSeperation(boid);
        speedChange = speedChange + updateSpeedAlignment(boid);
        speedChange = speedChange + updateSpeedBoundary(boid);
        boid->dx += speedChange.x;
        boid->dy += speedChange.y;

        CCLOG("ax: %f, ay: %f", speedChange.x, speedChange.y);
    }

    for (auto boid : boids) {
        boid->update();
    }
}

Vec2 Boids::updateSpeedCoherence(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    auto count = 0;
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= radius && other != target
            // && Vec2(target->dx, target->dy).dot(Vec2(other->x - target->x, other->y - target->y)) > 0.0f
            ) {
            count += 1;
            res = res + Vec2(other->x, other->y);
        }
    }
    if (count > 0) {
        res = res * (1.0f / count);
        return (res - Vec2(target->x, target->y)) * coherenceCoef;
    }
    return res;
}

Vec2 Boids::updateSpeedSeperation(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= mininumDist && other != target
            // && Vec2(target->dx, target->dy).dot(Vec2(other->x - target->x, other->y - target->y)) > 0.0f
            ) {
            Vec2 pulse = Vec2(target->dx, target->dy) - Vec2(other->dx, other->dy);
            // if (pulse.length() != 0) {
            //     pulse = pulse * (mininumDist - pulse.length()) / pulse.length();
            // }
            res = res + pulse;
        }
    }
    return res * seperationCoef;
}

Vec2 Boids::updateSpeedAlignment(Boid* target) {
    auto res = Vec2(0.0f, 0.0f);
    auto count = 0;
    for (auto other : boids) {
        if (target->getDistanceTo(other) <= radius && target != other
            // && Vec2(target->dx, target->dy).dot(Vec2(other->x - target->x, other->y - target->y)) > 0.0f
            ) {
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
    } 
    else if (target->y > boundary.getMaxY()) {
        res = res + Vec2(0.0f, -boundaryFactor * limitSpeed);
    }
    return res;
}

};