#include "physics/SoftBodyAdapter.h"
#include <cstdlib>
#define CHECK(cond) do { if (!(cond)) { std::cerr << "FAIL: " #cond " at line " << __LINE__ << "\n"; return EXIT_FAILURE; } } while(0)
#include <cmath>
#include <vector>
#include <iostream>

int main() {
    SoftWorld* world=softWorldCreate({0.0f,0.0f,0.0f});
    CHECK(world);
    SoftBody* cloth=softBodyCreateCloth(world,{0.0f,1.0f,0.0f},2.0f,2.0f,8,8,1.0f);
    CHECK(cloth);
    const size_t n=softBodyGetNodeCount(cloth);
    CHECK(n>0);
    std::vector<SoftVec3> before(n),after(n);
    CHECK(softBodyCopyNodePositions(cloth,before.data(),n)==n);
    // Fixed sphere intersecting cloth surface; triangle contact must deform cloth.
    SoftRigidBody* sphere=softRigidBodyCreateSphere(world,{0.0f,1.20f,0.0f},0.55f,0.0f);
    CHECK(sphere);
    for(int i=0;i<5;++i) CHECK(softWorldStep(world,1.0f/120.0f)==1);
    CHECK(softBodyCopyNodePositions(cloth,after.data(),n)==n);
    bool moved=false;
    for(size_t i=0;i<n;++i){
        CHECK(std::isfinite(after[i].x)&&std::isfinite(after[i].y)&&std::isfinite(after[i].z));
        if(std::abs(after[i].x-before[i].x)+std::abs(after[i].y-before[i].y)+
           std::abs(after[i].z-before[i].z)>1e-4f)moved=true;
    }
    CHECK(moved && "Sphere should deform cloth");
    SoftWorldStats stats{};
    CHECK(softWorldGetStats(world,&stats)==1);
    CHECK(stats.collision_ms>=0.0f);
    softRigidBodyDestroy(sphere);
    softBodyDestroy(cloth);
    softWorldDestroy(world);
    std::cout << "sphere-cloth collision: PASS\n";
}
