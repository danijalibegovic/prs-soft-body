#include "app/Application.h"
#include <spdlog/spdlog.h>
#include <vector>

#include "BulletSoftBodyScene.h"
#include "XpbdSoftBodyScene.h"

int main()
{
    spdlog::info("[TestApp] Starting");

    Application app;
    if (!app.Initialize())
    {
        spdlog::error("[TestApp] Application::Initialize() failed — aborting");
        return -1;
    }

    BulletSoftBodyScene bulletSoftBodyScene;
    XpbdSoftBodyScene xpbdSoftBodyScene;

    bool bulletSoftBodyOk = bulletSoftBodyScene.Setup();
    bool xpbdSoftBodyOk = xpbdSoftBodyScene.Setup();

    if (!bulletSoftBodyOk)
        spdlog::error("[TestApp] BulletSoftBodyScene::Setup() failed");
    if (!xpbdSoftBodyOk)
        spdlog::warn("[TestApp] XPBD scene disabled");

    std::vector<Scene*> scenes;
    if (bulletSoftBodyOk) scenes.push_back(&bulletSoftBodyScene);
    if (xpbdSoftBodyOk) scenes.push_back(&xpbdSoftBodyScene);

    for (Scene* scene : scenes)
        spdlog::info("[TestApp] Loaded {}", scene->GetName());

    if (!scenes.empty())
        app.Run(scenes, 0);
    else
        spdlog::error("[TestApp] No scenes were available to run");

    spdlog::info("[TestApp] Shutting down");
    return 0;
}
