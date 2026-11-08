#pragma once

// Hand-rolled Vulkan loader: no SDK and no import library are needed. Everything is fetched at run time through
// SDL_Vulkan_GetVkGetInstanceProcAddr (SDL loads vulkan-1.dll / libvulkan.so.1). VK_NO_PROTOTYPES is defined
// for the whole demo target, so the names below are plain function pointers.

#ifndef VK_NO_PROTOTYPES
#define VK_NO_PROTOTYPES
#endif
#include <vulkan/vulkan.h>

// Global (no instance) functions.
#define VKL_GLOBAL_FUNCS(X)                  \
    X(vkCreateInstance)                      \
    X(vkEnumerateInstanceExtensionProperties) \
    X(vkEnumerateInstanceVersion)

// Instance functions.
#define VKL_INSTANCE_FUNCS(X)                      \
    X(vkDestroyInstance)                           \
    X(vkEnumeratePhysicalDevices)                  \
    X(vkGetPhysicalDeviceProperties)               \
    X(vkGetPhysicalDeviceFeatures)                 \
    X(vkGetPhysicalDeviceQueueFamilyProperties)    \
    X(vkGetPhysicalDeviceMemoryProperties)         \
    X(vkGetPhysicalDeviceFormatProperties)         \
    X(vkEnumerateDeviceExtensionProperties)        \
    X(vkCreateDevice)                              \
    X(vkGetDeviceProcAddr)                         \
    X(vkDestroySurfaceKHR)                         \
    X(vkGetPhysicalDeviceSurfaceSupportKHR)        \
    X(vkGetPhysicalDeviceSurfaceCapabilitiesKHR)   \
    X(vkGetPhysicalDeviceSurfaceFormatsKHR)        \
    X(vkGetPhysicalDeviceSurfacePresentModesKHR)

// Device functions.
#define VKL_DEVICE_FUNCS(X)              \
    X(vkDestroyDevice)                   \
    X(vkGetDeviceQueue)                  \
    X(vkDeviceWaitIdle)                  \
    X(vkQueueSubmit)                     \
    X(vkQueueWaitIdle)                   \
    X(vkCreateSwapchainKHR)              \
    X(vkDestroySwapchainKHR)             \
    X(vkGetSwapchainImagesKHR)           \
    X(vkAcquireNextImageKHR)             \
    X(vkQueuePresentKHR)                 \
    X(vkCreateImage)                     \
    X(vkDestroyImage)                    \
    X(vkCreateImageView)                 \
    X(vkDestroyImageView)                \
    X(vkGetImageMemoryRequirements)      \
    X(vkBindImageMemory)                 \
    X(vkCreateBuffer)                    \
    X(vkDestroyBuffer)                   \
    X(vkGetBufferMemoryRequirements)     \
    X(vkBindBufferMemory)                \
    X(vkAllocateMemory)                  \
    X(vkFreeMemory)                      \
    X(vkMapMemory)                       \
    X(vkUnmapMemory)                     \
    X(vkCreateRenderPass)                \
    X(vkDestroyRenderPass)               \
    X(vkCreateFramebuffer)               \
    X(vkDestroyFramebuffer)              \
    X(vkCreateShaderModule)              \
    X(vkDestroyShaderModule)             \
    X(vkCreatePipelineLayout)            \
    X(vkDestroyPipelineLayout)           \
    X(vkCreateGraphicsPipelines)         \
    X(vkDestroyPipeline)                 \
    X(vkCreateCommandPool)               \
    X(vkDestroyCommandPool)              \
    X(vkAllocateCommandBuffers)          \
    X(vkFreeCommandBuffers)              \
    X(vkBeginCommandBuffer)              \
    X(vkEndCommandBuffer)                \
    X(vkResetCommandBuffer)              \
    X(vkCreateFence)                     \
    X(vkDestroyFence)                    \
    X(vkWaitForFences)                   \
    X(vkResetFences)                     \
    X(vkCreateSemaphore)                 \
    X(vkDestroySemaphore)                \
    X(vkCmdBeginRenderPass)              \
    X(vkCmdEndRenderPass)                \
    X(vkCmdBindPipeline)                 \
    X(vkCmdBindVertexBuffers)            \
    X(vkCmdBindIndexBuffer)              \
    X(vkCmdDraw)                         \
    X(vkCmdDrawIndexed)                  \
    X(vkCmdPushConstants)                \
    X(vkCmdSetViewport)                  \
    X(vkCmdSetScissor)                   \
    X(vkCmdCopyBuffer)                   \
    X(vkCmdPipelineBarrier)              \
    X(vkCmdCopyImageToBuffer)

#define VKL_DECLARE(name) extern PFN_##name name;
VKL_GLOBAL_FUNCS(VKL_DECLARE)
VKL_INSTANCE_FUNCS(VKL_DECLARE)
VKL_DEVICE_FUNCS(VKL_DECLARE)
extern PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr;
#undef VKL_DECLARE

// Step 1: after SDL_Vulkan_LoadLibrary. Returns false when the loader is missing.
bool vkl_load_global(PFN_vkGetInstanceProcAddr get_instance_proc_addr);
// Step 2: after vkCreateInstance. Returns false and names the first missing function.
bool vkl_load_instance(VkInstance instance, const char **missing);
// Step 3: after vkCreateDevice.
bool vkl_load_device(VkDevice device, const char **missing);
