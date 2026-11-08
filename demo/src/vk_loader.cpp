#include "vk_loader.h"

#define VKL_DEFINE(name) PFN_##name name = nullptr;
VKL_GLOBAL_FUNCS(VKL_DEFINE)
VKL_INSTANCE_FUNCS(VKL_DEFINE)
VKL_DEVICE_FUNCS(VKL_DEFINE)
PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = nullptr;
#undef VKL_DEFINE

bool vkl_load_global(PFN_vkGetInstanceProcAddr get_instance_proc_addr)
{
    vkGetInstanceProcAddr = get_instance_proc_addr;
    if (!vkGetInstanceProcAddr)
        return false;
#define VKL_LOAD(name) name = (PFN_##name)vkGetInstanceProcAddr(VK_NULL_HANDLE, #name);
    VKL_GLOBAL_FUNCS(VKL_LOAD)
#undef VKL_LOAD
    // vkEnumerateInstanceVersion is 1.1+; a 1.0 loader lacks it (optional).
    return vkCreateInstance && vkEnumerateInstanceExtensionProperties;
}

bool vkl_load_instance(VkInstance instance, const char **missing)
{
#define VKL_LOAD(name)                                                    \
    name = (PFN_##name)vkGetInstanceProcAddr(instance, #name);            \
    if (!name)                                                            \
    {                                                                     \
        if (missing)                                                      \
            *missing = #name;                                             \
        return false;                                                     \
    }
    VKL_INSTANCE_FUNCS(VKL_LOAD)
#undef VKL_LOAD
    return true;
}

bool vkl_load_device(VkDevice device, const char **missing)
{
#define VKL_LOAD(name)                                                    \
    name = (PFN_##name)vkGetDeviceProcAddr(device, #name);                \
    if (!name)                                                            \
    {                                                                     \
        if (missing)                                                      \
            *missing = #name;                                             \
        return false;                                                     \
    }
    VKL_DEVICE_FUNCS(VKL_LOAD)
#undef VKL_LOAD
    return true;
}
