#include "XpbdSoftBodyApi.h"

#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <filesystem>
#include <system_error>

namespace
{
    std::filesystem::path GetExecutableDirectory()
    {
        std::wstring buffer(32768, L'\0');
        const DWORD length = GetModuleFileNameW(nullptr, buffer.data(),
                                                  static_cast<DWORD>(buffer.size()));
        if (length == 0 || length == buffer.size())
            return {};
        buffer.resize(length);
        return std::filesystem::path(buffer).parent_path();
    }
}
#endif

XpbdSoftBodyApi::~XpbdSoftBodyApi()
{
#ifdef _WIN32
    if (m_module != nullptr)
        FreeLibrary(m_module);
#endif
}

bool XpbdSoftBodyApi::Load()
{
#ifndef _WIN32
    spdlog::error("[XPBD] Runtime loading is currently supported on Windows only");
    return false;
#else
    const std::filesystem::path executable_directory = GetExecutableDirectory();
    const std::filesystem::path adjacent_paths[] = {
        executable_directory / "libsoft_body_xpbd_adapter.dll",
        executable_directory / "soft_body_xpbd_adapter.dll",
    };
    std::filesystem::path path = adjacent_paths[0];
    for (const auto& candidate : adjacent_paths)
    {
        path = candidate;
        spdlog::info("[XPBD] Looking for {}", path.string());
        m_module = LoadLibraryW(path.c_str());
        if (m_module != nullptr)
            break;
    }
#ifdef XPBD_ADAPTER_SOURCE_DLL
    if (m_module == nullptr)
    {
        path = XPBD_ADAPTER_SOURCE_DLL;
        spdlog::info("[XPBD] Trying configured source DLL {}", path.string());
        m_module = LoadLibraryW(path.c_str());
    }
#endif
    if (m_module == nullptr)
    {
        const DWORD error = GetLastError();
        spdlog::error("[XPBD] Could not load {} (Windows error {})",
                      path.string(), error);
        return false;
    }

#define XPBD_LOAD(name) \
    name = reinterpret_cast<decltype(name)>(GetProcAddress(m_module, #name)); \
    if (name == nullptr) \
    { \
        spdlog::error("[XPBD] Missing export {}", #name); \
        FreeLibrary(m_module); \
        m_module = nullptr; \
        return false; \
    }

    XPBD_LOAD(softWorldCreate)
    XPBD_LOAD(softWorldDestroy)
    XPBD_LOAD(softWorldSetGravity)
    XPBD_LOAD(softWorldStep)
    XPBD_LOAD(softWorldGetStats)
    XPBD_LOAD(softBodyCreateCloth)
    XPBD_LOAD(softBodyCreateSphere)
    XPBD_LOAD(softBodyCreateCube)
    XPBD_LOAD(softBodyDestroy)
    XPBD_LOAD(softRigidBodyCreateSphere)
    XPBD_LOAD(softRigidBodyDestroy)
    XPBD_LOAD(softRigidBodyGetPosition)
    XPBD_LOAD(softBodyGetNodeCount)
    XPBD_LOAD(softBodyGetFaceCount)
    XPBD_LOAD(softBodyCopyNodePositions)
    XPBD_LOAD(softBodyCopyIndices)

#undef XPBD_LOAD
    spdlog::info("[XPBD] Loaded {}", path.string());
    return true;
#endif
}
