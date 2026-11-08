#include "renderer.h"

#include <SDL.h>
#include <SDL_vulkan.h>

#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"

#include "../shaders/generated/blob_frag.h"
#include "../shaders/generated/blob_vert.h"
#include "../shaders/generated/ground_frag.h"
#include "../shaders/generated/ground_vert.h"
#include "../shaders/generated/lines_frag.h"
#include "../shaders/generated/lines_vert.h"
#include "../shaders/generated/mesh_frag.h"
#include "../shaders/generated/mesh_vert.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#define VK_TRY(expr, what)                                                       \
    do                                                                           \
    {                                                                            \
        const VkResult vk_try_r = (expr);                                        \
        if (vk_try_r != VK_SUCCESS)                                              \
        {                                                                        \
            if (error)                                                           \
            {                                                                    \
                char vk_try_buf[256];                                            \
                snprintf(vk_try_buf, sizeof(vk_try_buf), "%s failed (VkResult %d)", what, (int)vk_try_r); \
                *error = vk_try_buf;                                             \
            }                                                                    \
            return false;                                                        \
        }                                                                        \
    } while (0)

struct push_constants
{
    float viewproj[16];
    float light_dir[4];
    float cam_pos[4];
    float params[4];
};
static_assert(sizeof(push_constants) == 112, "push constants must stay under 128 bytes");

enum
{
    VERTEX_MESH = 0,     // binding 0: pos + normal, binding 1: instances
    VERTEX_NONE = 1,     // generated in the shader
    VERTEX_INSTANCE = 2, // binding 1 only
    VERTEX_LINES = 3,    // binding 0: pos, binding 1: instances
};

// ---------------------------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------------------------

static PFN_vkVoidFunction imgui_loader(const char *name, void *user_data)
{
    return vkGetInstanceProcAddr((VkInstance)user_data, name);
}

bool renderer::init(SDL_Window *window, std::string *error)
{
    m_window = window;
    if (!vkl_load_global((PFN_vkGetInstanceProcAddr)SDL_Vulkan_GetVkGetInstanceProcAddr()))
    {
        if (error)
            *error = "Vulkan loader not found (SDL_Vulkan_GetVkGetInstanceProcAddr)";
        return false;
    }
    if (vkEnumerateInstanceVersion)
    {
        uint32_t v = VK_API_VERSION_1_0;
        if (vkEnumerateInstanceVersion(&v) == VK_SUCCESS && v >= VK_API_VERSION_1_1)
            m_api_version = VK_API_VERSION_1_1;
    }

    unsigned int ext_count = 0;
    if (!SDL_Vulkan_GetInstanceExtensions(window, &ext_count, nullptr))
    {
        if (error)
            *error = std::string("SDL_Vulkan_GetInstanceExtensions: ") + SDL_GetError();
        return false;
    }
    std::vector<const char *> exts(ext_count);
    SDL_Vulkan_GetInstanceExtensions(window, &ext_count, exts.data());

    VkApplicationInfo app = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "treybo_demo";
    app.pEngineName = "TreyBO";
    app.apiVersion = m_api_version;
    VkInstanceCreateInfo ici = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    ici.enabledExtensionCount = ext_count;
    ici.ppEnabledExtensionNames = exts.data();
    VK_TRY(vkCreateInstance(&ici, nullptr, &m_instance), "vkCreateInstance");
    const char *missing = nullptr;
    if (!vkl_load_instance(m_instance, &missing))
    {
        if (error)
            *error = std::string("missing instance function ") + (missing ? missing : "?");
        return false;
    }
    if (!SDL_Vulkan_CreateSurface(window, m_instance, &m_surface))
    {
        if (error)
            *error = std::string("SDL_Vulkan_CreateSurface: ") + SDL_GetError();
        return false;
    }
    if (!create_device(error))
        return false;

    VkCommandPoolCreateInfo cpci = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cpci.queueFamilyIndex = m_queue_family;
    VK_TRY(vkCreateCommandPool(m_device, &cpci, nullptr, &m_cmd_pool), "vkCreateCommandPool");

    for (frame_data &f : m_frames)
    {
        VkCommandBufferAllocateInfo cai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = m_cmd_pool;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        VK_TRY(vkAllocateCommandBuffers(m_device, &cai, &f.cmd), "vkAllocateCommandBuffers");
        VkFenceCreateInfo fci = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fci.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        VK_TRY(vkCreateFence(m_device, &fci, nullptr, &f.fence), "vkCreateFence");
        VkSemaphoreCreateInfo sci = {VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VK_TRY(vkCreateSemaphore(m_device, &sci, nullptr, &f.image_available), "vkCreateSemaphore");
        if (!create_buffer((VkDeviceSize)m_max_instances * sizeof(render_instance), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &f.instances))
        {
            if (error)
                *error = "instance buffer allocation failed";
            return false;
        }
        VK_TRY(vkMapMemory(m_device, f.instances.mem, 0, VK_WHOLE_SIZE, 0, &f.instances.mapped), "vkMapMemory");
    }

    // Formats and sample count are fixed for the life of the device (the render pass depends on them).
    {
        uint32_t n = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(m_gpu, m_surface, &n, nullptr);
        std::vector<VkSurfaceFormatKHR> formats(n);
        vkGetPhysicalDeviceSurfaceFormatsKHR(m_gpu, m_surface, &n, formats.data());
        if (formats.empty())
        {
            if (error)
                *error = "surface has no formats";
            return false;
        }
        m_surface_format = formats[0];
        for (const VkSurfaceFormatKHR &f : formats)
            if ((f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_R8G8B8A8_UNORM) && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                m_surface_format = f;
                break;
            }
        if (m_surface_format.format == VK_FORMAT_UNDEFINED)
            m_surface_format.format = VK_FORMAT_B8G8R8A8_UNORM;

        const VkFormat depth_candidates[] = {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D16_UNORM};
        for (VkFormat f : depth_candidates)
        {
            VkFormatProperties fp;
            vkGetPhysicalDeviceFormatProperties(m_gpu, f, &fp);
            if (fp.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
            {
                m_depth_format = f;
                break;
            }
        }
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(m_gpu, &props);
        const VkSampleCountFlags counts = props.limits.framebufferColorSampleCounts & props.limits.framebufferDepthSampleCounts;
        m_samples = (counts & VK_SAMPLE_COUNT_4_BIT) ? VK_SAMPLE_COUNT_4_BIT : VK_SAMPLE_COUNT_1_BIT;
    }

    if (!create_render_pass(error) || !create_pipelines(error) || !create_swapchain(error))
        return false;
    return true;
}

bool renderer::create_device(std::string *error)
{
    uint32_t n = 0;
    vkEnumeratePhysicalDevices(m_instance, &n, nullptr);
    std::vector<VkPhysicalDevice> gpus(n);
    vkEnumeratePhysicalDevices(m_instance, &n, gpus.data());
    int best_score = -1;
    for (VkPhysicalDevice gpu : gpus)
    {
        uint32_t qn = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(gpu, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qf(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(gpu, &qn, qf.data());
        int family = -1;
        for (uint32_t i = 0; i < qn; ++i)
        {
            VkBool32 present = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(gpu, i, m_surface, &present);
            if ((qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && present)
            {
                family = (int)i;
                break;
            }
        }
        if (family < 0)
            continue;
        uint32_t en = 0;
        vkEnumerateDeviceExtensionProperties(gpu, nullptr, &en, nullptr);
        std::vector<VkExtensionProperties> ext(en);
        vkEnumerateDeviceExtensionProperties(gpu, nullptr, &en, ext.data());
        bool has_swapchain = false;
        for (const VkExtensionProperties &e : ext)
            has_swapchain |= strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0;
        if (!has_swapchain)
            continue;
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(gpu, &props);
        const int score = props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 3 : props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 2 : 1;
        if (score > best_score)
        {
            best_score = score;
            m_gpu = gpu;
            m_queue_family = (uint32_t)family;
            m_device_name = props.deviceName;
        }
    }
    if (!m_gpu)
    {
        if (error)
            *error = "no Vulkan device with graphics + present + swapchain";
        return false;
    }
    vkGetPhysicalDeviceMemoryProperties(m_gpu, &m_mem_props);

    const float prio = 1.0f;
    VkDeviceQueueCreateInfo qci = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = m_queue_family;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    const char *dev_exts[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkPhysicalDeviceFeatures feats = {};
    VkDeviceCreateInfo dci = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    dci.enabledExtensionCount = 1;
    dci.ppEnabledExtensionNames = dev_exts;
    dci.pEnabledFeatures = &feats;
    VK_TRY(vkCreateDevice(m_gpu, &dci, nullptr, &m_device), "vkCreateDevice");
    const char *missing = nullptr;
    if (!vkl_load_device(m_device, &missing))
    {
        if (error)
            *error = std::string("missing device function ") + (missing ? missing : "?");
        return false;
    }
    vkGetDeviceQueue(m_device, m_queue_family, 0, &m_queue);
    return true;
}

// ---------------------------------------------------------------------------------------------
// Memory helpers
// ---------------------------------------------------------------------------------------------

uint32_t renderer::find_memory(uint32_t type_bits, VkMemoryPropertyFlags flags, VkMemoryPropertyFlags fallback) const
{
    for (uint32_t i = 0; i < m_mem_props.memoryTypeCount; ++i)
        if ((type_bits & (1u << i)) && (m_mem_props.memoryTypes[i].propertyFlags & flags) == flags)
            return i;
    if (fallback)
        for (uint32_t i = 0; i < m_mem_props.memoryTypeCount; ++i)
            if ((type_bits & (1u << i)) && (m_mem_props.memoryTypes[i].propertyFlags & fallback) == fallback)
                return i;
    return UINT32_MAX;
}

bool renderer::create_buffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, buffer *out)
{
    VkBufferCreateInfo bci = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size ? size : 16;
    bci.usage = usage;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(m_device, &bci, nullptr, &out->buf) != VK_SUCCESS)
        return false;
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(m_device, out->buf, &req);
    const uint32_t type = find_memory(req.memoryTypeBits, flags);
    if (type == UINT32_MAX)
        return false;
    VkMemoryAllocateInfo mai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = type;
    if (vkAllocateMemory(m_device, &mai, nullptr, &out->mem) != VK_SUCCESS)
        return false;
    vkBindBufferMemory(m_device, out->buf, out->mem, 0);
    out->size = bci.size;
    return true;
}

bool renderer::create_static_buffer(const void *data, VkDeviceSize size, VkBufferUsageFlags usage, buffer *out)
{
    // Staging copy into device-local memory, one-shot command buffer.
    buffer staging;
    if (!create_buffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &staging))
        return false;
    void *p = nullptr;
    vkMapMemory(m_device, staging.mem, 0, VK_WHOLE_SIZE, 0, &p);
    memcpy(p, data, (size_t)size);
    vkUnmapMemory(m_device, staging.mem);
    if (!create_buffer(size, usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, out))
    {
        destroy_buffer(staging);
        return false;
    }
    VkCommandBufferAllocateInfo cai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool = m_cmd_pool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(m_device, &cai, &cmd);
    VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    VkBufferCopy region = {0, 0, size};
    vkCmdCopyBuffer(cmd, staging.buf, out->buf, 1, &region);
    vkEndCommandBuffer(cmd);
    VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    vkQueueSubmit(m_queue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(m_queue);
    vkFreeCommandBuffers(m_device, m_cmd_pool, 1, &cmd);
    destroy_buffer(staging);
    return true;
}

void renderer::destroy_buffer(buffer &b)
{
    if (b.mapped)
        vkUnmapMemory(m_device, b.mem);
    if (b.buf)
        vkDestroyBuffer(m_device, b.buf, nullptr);
    if (b.mem)
        vkFreeMemory(m_device, b.mem, nullptr);
    b = buffer();
}

bool renderer::create_image(uint32_t w, uint32_t h, VkFormat format, VkImageUsageFlags usage, VkSampleCountFlagBits samples,
                            VkImageAspectFlags aspect, image *out)
{
    VkImageCreateInfo ici = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ici.imageType = VK_IMAGE_TYPE_2D;
    ici.format = format;
    ici.extent = {w, h, 1};
    ici.mipLevels = 1;
    ici.arrayLayers = 1;
    ici.samples = samples;
    ici.tiling = VK_IMAGE_TILING_OPTIMAL;
    ici.usage = usage;
    ici.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(m_device, &ici, nullptr, &out->img) != VK_SUCCESS)
        return false;
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(m_device, out->img, &req);
    // Transient attachments prefer lazily allocated memory (tilers); fall back to device local.
    uint32_t type = UINT32_MAX;
    if (usage & VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT)
        type = find_memory(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT);
    if (type == UINT32_MAX)
        type = find_memory(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0);
    if (type == UINT32_MAX)
        return false;
    VkMemoryAllocateInfo mai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = type;
    if (vkAllocateMemory(m_device, &mai, nullptr, &out->mem) != VK_SUCCESS)
        return false;
    vkBindImageMemory(m_device, out->img, out->mem, 0);
    VkImageViewCreateInfo vci = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vci.image = out->img;
    vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vci.format = format;
    vci.subresourceRange = {aspect, 0, 1, 0, 1};
    return vkCreateImageView(m_device, &vci, nullptr, &out->view) == VK_SUCCESS;
}

void renderer::destroy_image(image &i)
{
    if (i.view)
        vkDestroyImageView(m_device, i.view, nullptr);
    if (i.img)
        vkDestroyImage(m_device, i.img, nullptr);
    if (i.mem)
        vkFreeMemory(m_device, i.mem, nullptr);
    i = image();
}

// ---------------------------------------------------------------------------------------------
// Render pass, swapchain
// ---------------------------------------------------------------------------------------------

bool renderer::create_render_pass(std::string *error)
{
    const bool msaa = m_samples != VK_SAMPLE_COUNT_1_BIT;
    VkAttachmentDescription att[3] = {};
    // 0: colour (MSAA target, or the swapchain image directly)
    att[0].format = m_surface_format.format;
    att[0].samples = m_samples;
    att[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att[0].storeOp = msaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
    att[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    att[0].finalLayout = msaa ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    // 1: depth
    att[1].format = m_depth_format;
    att[1].samples = m_samples;
    att[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    att[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    // 2: resolve into the swapchain image
    att[2].format = m_surface_format.format;
    att[2].samples = VK_SAMPLE_COUNT_1_BIT;
    att[2].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att[2].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    att[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    att[2].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference color_ref = {0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depth_ref = {1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference resolve_ref = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sub = {};
    sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sub.colorAttachmentCount = 1;
    sub.pColorAttachments = &color_ref;
    sub.pResolveAttachments = msaa ? &resolve_ref : nullptr;
    sub.pDepthStencilAttachment = &depth_ref;

    // The acquire semaphore is waited at COLOR_ATTACHMENT_OUTPUT; depth is reused across frames.
    VkSubpassDependency dep = {};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    VkRenderPassCreateInfo rpci = {VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rpci.attachmentCount = msaa ? 3 : 2;
    rpci.pAttachments = att;
    rpci.subpassCount = 1;
    rpci.pSubpasses = &sub;
    rpci.dependencyCount = 1;
    rpci.pDependencies = &dep;
    VK_TRY(vkCreateRenderPass(m_device, &rpci, nullptr, &m_render_pass), "vkCreateRenderPass");
    return true;
}

bool renderer::create_swapchain(std::string *error)
{
    VkSurfaceCapabilitiesKHR caps;
    VK_TRY(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_gpu, m_surface, &caps), "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
    int w = 0, h = 0;
    SDL_Vulkan_GetDrawableSize(m_window, &w, &h);
    VkExtent2D extent = caps.currentExtent;
    if (extent.width == UINT32_MAX)
    {
        extent.width = (uint32_t)std::max(1, w);
        extent.height = (uint32_t)std::max(1, h);
    }
    extent.width = std::min(std::max(extent.width, caps.minImageExtent.width), caps.maxImageExtent.width);
    extent.height = std::min(std::max(extent.height, caps.minImageExtent.height), caps.maxImageExtent.height);
    m_extent = extent;
    if (extent.width == 0 || extent.height == 0)
        return true; // minimised: try again later

    VkPresentModeKHR mode = VK_PRESENT_MODE_FIFO_KHR; // always available
    if (!m_vsync)
    {
        uint32_t n = 0;
        vkGetPhysicalDeviceSurfacePresentModesKHR(m_gpu, m_surface, &n, nullptr);
        std::vector<VkPresentModeKHR> modes(n);
        vkGetPhysicalDeviceSurfacePresentModesKHR(m_gpu, m_surface, &n, modes.data());
        for (VkPresentModeKHR m : modes)
            if (m == VK_PRESENT_MODE_MAILBOX_KHR)
                mode = m;
        if (mode == VK_PRESENT_MODE_FIFO_KHR)
            for (VkPresentModeKHR m : modes)
                if (m == VK_PRESENT_MODE_IMMEDIATE_KHR)
                    mode = m;
    }

    uint32_t image_count = std::max(caps.minImageCount + 1, 3u);
    if (caps.maxImageCount)
        image_count = std::min(image_count, caps.maxImageCount);
    m_min_image_count = std::max(2u, caps.minImageCount);

    VkSwapchainKHR old = m_swapchain;
    VkSwapchainCreateInfoKHR sci = {VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    sci.surface = m_surface;
    sci.minImageCount = image_count;
    sci.imageFormat = m_surface_format.format;
    sci.imageColorSpace = m_surface_format.colorSpace;
    sci.imageExtent = extent;
    sci.imageArrayLayers = 1;
    sci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (caps.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT)
        sci.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT; // frame capture
    sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    sci.preTransform = (caps.supportedTransforms & VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ? VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR : caps.currentTransform;
    sci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    if (!(caps.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR))
        for (uint32_t bit = 1; bit <= VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR; bit <<= 1)
            if (caps.supportedCompositeAlpha & bit)
            {
                sci.compositeAlpha = (VkCompositeAlphaFlagBitsKHR)bit;
                break;
            }
    sci.presentMode = mode;
    sci.clipped = VK_TRUE;
    sci.oldSwapchain = old;
    VK_TRY(vkCreateSwapchainKHR(m_device, &sci, nullptr, &m_swapchain), "vkCreateSwapchainKHR");
    if (old)
        vkDestroySwapchainKHR(m_device, old, nullptr);

    uint32_t n = 0;
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &n, nullptr);
    m_swap_images.resize(n);
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &n, m_swap_images.data());

    const bool msaa = m_samples != VK_SAMPLE_COUNT_1_BIT;
    if (msaa && !create_image(extent.width, extent.height, m_surface_format.format,
                              VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT, m_samples, VK_IMAGE_ASPECT_COLOR_BIT, &m_color_msaa))
    {
        if (error)
            *error = "MSAA colour target allocation failed";
        return false;
    }
    if (!create_image(extent.width, extent.height, m_depth_format, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                      m_samples, VK_IMAGE_ASPECT_DEPTH_BIT, &m_depth))
    {
        if (error)
            *error = "depth buffer allocation failed";
        return false;
    }

    m_swap_views.resize(n);
    m_framebuffers.resize(n);
    m_render_done.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        VkImageViewCreateInfo vci = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vci.image = m_swap_images[i];
        vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vci.format = m_surface_format.format;
        vci.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VK_TRY(vkCreateImageView(m_device, &vci, nullptr, &m_swap_views[i]), "vkCreateImageView");
        VkImageView views[3];
        uint32_t count;
        if (msaa)
        {
            views[0] = m_color_msaa.view;
            views[1] = m_depth.view;
            views[2] = m_swap_views[i];
            count = 3;
        }
        else
        {
            views[0] = m_swap_views[i];
            views[1] = m_depth.view;
            count = 2;
        }
        VkFramebufferCreateInfo fci = {VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        fci.renderPass = m_render_pass;
        fci.attachmentCount = count;
        fci.pAttachments = views;
        fci.width = extent.width;
        fci.height = extent.height;
        fci.layers = 1;
        VK_TRY(vkCreateFramebuffer(m_device, &fci, nullptr, &m_framebuffers[i]), "vkCreateFramebuffer");
        VkSemaphoreCreateInfo sem = {VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VK_TRY(vkCreateSemaphore(m_device, &sem, nullptr, &m_render_done[i]), "vkCreateSemaphore");
    }
    if (m_imgui)
        ImGui_ImplVulkan_SetMinImageCount(m_min_image_count);
    m_swapchain_dirty = false;
    return true;
}

void renderer::destroy_swapchain()
{
    for (VkFramebuffer fb : m_framebuffers)
        vkDestroyFramebuffer(m_device, fb, nullptr);
    for (VkImageView v : m_swap_views)
        vkDestroyImageView(m_device, v, nullptr);
    for (VkSemaphore s : m_render_done)
        vkDestroySemaphore(m_device, s, nullptr);
    m_framebuffers.clear();
    m_swap_views.clear();
    m_render_done.clear();
    m_swap_images.clear();
    destroy_image(m_color_msaa);
    destroy_image(m_depth);
    // m_swapchain itself is kept as oldSwapchain for the next create (destroyed there or in shutdown).
}

void renderer::set_vsync(bool vsync)
{
    if (vsync != m_vsync)
    {
        m_vsync = vsync;
        m_swapchain_dirty = true;
    }
}

// ---------------------------------------------------------------------------------------------
// Pipelines
// ---------------------------------------------------------------------------------------------

VkPipeline renderer::make_pipeline(const uint32_t *vs, size_t vs_size, const uint32_t *fs, size_t fs_size, int vertex_mode,
                                   VkPrimitiveTopology topology, bool depth_write, bool blend, VkCullModeFlags cull)
{
    VkShaderModule mods[2] = {};
    VkShaderModuleCreateInfo smci = {VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    smci.codeSize = vs_size;
    smci.pCode = vs;
    if (vkCreateShaderModule(m_device, &smci, nullptr, &mods[0]) != VK_SUCCESS)
        return VK_NULL_HANDLE;
    smci.codeSize = fs_size;
    smci.pCode = fs;
    if (vkCreateShaderModule(m_device, &smci, nullptr, &mods[1]) != VK_SUCCESS)
    {
        vkDestroyShaderModule(m_device, mods[0], nullptr);
        return VK_NULL_HANDLE;
    }
    VkPipelineShaderStageCreateInfo stages[2] = {};
    stages[0].sType = stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = mods[0];
    stages[0].pName = "main";
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = mods[1];
    stages[1].pName = "main";

    std::vector<VkVertexInputBindingDescription> bindings;
    std::vector<VkVertexInputAttributeDescription> attrs;
    if (vertex_mode == VERTEX_MESH)
    {
        bindings.push_back({0, (uint32_t)sizeof(render_vertex), VK_VERTEX_INPUT_RATE_VERTEX});
        attrs.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        attrs.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12});
    }
    else if (vertex_mode == VERTEX_LINES)
    {
        bindings.push_back({0, 12, VK_VERTEX_INPUT_RATE_VERTEX});
        attrs.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
    }
    if (vertex_mode != VERTEX_NONE)
    {
        bindings.push_back({1, (uint32_t)sizeof(render_instance), VK_VERTEX_INPUT_RATE_INSTANCE});
        for (uint32_t i = 0; i < 5; ++i)
            attrs.push_back({2 + i, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 16 * i});
    }
    VkPipelineVertexInputStateCreateInfo vi = {VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vi.vertexBindingDescriptionCount = (uint32_t)bindings.size();
    vi.pVertexBindingDescriptions = bindings.data();
    vi.vertexAttributeDescriptionCount = (uint32_t)attrs.size();
    vi.pVertexAttributeDescriptions = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo ia = {VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = topology;
    VkPipelineViewportStateCreateInfo vp = {VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo rs = {VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = cull;
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;
    VkPipelineMultisampleStateCreateInfo ms = {VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = m_samples;
    VkPipelineDepthStencilStateCreateInfo ds = {VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    VkPipelineColorBlendAttachmentState cba = {};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    if (blend)
    {
        cba.blendEnable = VK_TRUE;
        cba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        cba.colorBlendOp = VK_BLEND_OP_ADD;
        cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        cba.alphaBlendOp = VK_BLEND_OP_ADD;
    }
    VkPipelineColorBlendStateCreateInfo cb = {VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &cba;
    const VkDynamicState dyn[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dy = {VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dy.dynamicStateCount = 2;
    dy.pDynamicStates = dyn;

    VkGraphicsPipelineCreateInfo gpci = {VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gpci.stageCount = 2;
    gpci.pStages = stages;
    gpci.pVertexInputState = &vi;
    gpci.pInputAssemblyState = &ia;
    gpci.pViewportState = &vp;
    gpci.pRasterizationState = &rs;
    gpci.pMultisampleState = &ms;
    gpci.pDepthStencilState = &ds;
    gpci.pColorBlendState = &cb;
    gpci.pDynamicState = &dy;
    gpci.layout = m_layout;
    gpci.renderPass = m_render_pass;
    gpci.subpass = 0;
    VkPipeline pipe = VK_NULL_HANDLE;
    vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &gpci, nullptr, &pipe);
    vkDestroyShaderModule(m_device, mods[0], nullptr);
    vkDestroyShaderModule(m_device, mods[1], nullptr);
    return pipe;
}

bool renderer::create_pipelines(std::string *error)
{
    VkPushConstantRange pcr = {VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push_constants)};
    VkPipelineLayoutCreateInfo plci = {VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges = &pcr;
    VK_TRY(vkCreatePipelineLayout(m_device, &plci, nullptr, &m_layout), "vkCreatePipelineLayout");

    // The logo mesh is not guaranteed watertight / consistently wound: no culling.
    m_pipe_mesh = make_pipeline(spv_mesh_vert, sizeof(spv_mesh_vert), spv_mesh_frag, sizeof(spv_mesh_frag), VERTEX_MESH,
                                VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, false, VK_CULL_MODE_NONE);
    m_pipe_ground = make_pipeline(spv_ground_vert, sizeof(spv_ground_vert), spv_ground_frag, sizeof(spv_ground_frag), VERTEX_NONE,
                                  VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, false, VK_CULL_MODE_NONE);
    m_pipe_blob = make_pipeline(spv_blob_vert, sizeof(spv_blob_vert), spv_blob_frag, sizeof(spv_blob_frag), VERTEX_INSTANCE,
                                VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, false, true, VK_CULL_MODE_NONE);
    m_pipe_lines = make_pipeline(spv_lines_vert, sizeof(spv_lines_vert), spv_lines_frag, sizeof(spv_lines_frag), VERTEX_LINES,
                                 VK_PRIMITIVE_TOPOLOGY_LINE_LIST, true, false, VK_CULL_MODE_NONE);
    if (!m_pipe_mesh || !m_pipe_ground || !m_pipe_blob || !m_pipe_lines)
    {
        if (error)
            *error = "vkCreateGraphicsPipelines failed";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------------------------
// Meshes
// ---------------------------------------------------------------------------------------------

int renderer::create_mesh(const render_vertex *verts, uint32_t vert_count, const uint32_t *indices, uint32_t index_count)
{
    mesh m;
    if (!create_static_buffer(verts, (VkDeviceSize)vert_count * sizeof(render_vertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, &m.vb) ||
        !create_static_buffer(indices, (VkDeviceSize)index_count * sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, &m.ib))
        return -1;
    m.index_count = index_count;
    m_meshes.push_back(m);
    return (int)m_meshes.size() - 1;
}

int renderer::create_lines(const float *positions, uint32_t vert_count, const uint32_t *indices, uint32_t index_count)
{
    mesh m;
    if (!create_static_buffer(positions, (VkDeviceSize)vert_count * 12, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, &m.vb) ||
        !create_static_buffer(indices, (VkDeviceSize)index_count * sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, &m.ib))
        return -1;
    m.index_count = index_count;
    m_meshes.push_back(m);
    return (int)m_meshes.size() - 1;
}

// ---------------------------------------------------------------------------------------------
// ImGui
// ---------------------------------------------------------------------------------------------

static void imgui_check(VkResult r)
{
    if (r != VK_SUCCESS)
        fprintf(stderr, "imgui vulkan: VkResult %d\n", (int)r);
}

bool renderer::init_imgui(std::string *error)
{
    if (!ImGui_ImplVulkan_LoadFunctions(m_api_version, imgui_loader, (void *)m_instance))
    {
        if (error)
            *error = "ImGui_ImplVulkan_LoadFunctions failed";
        return false;
    }
    ImGui_ImplVulkan_InitInfo ii = {};
    ii.ApiVersion = m_api_version;
    ii.Instance = m_instance;
    ii.PhysicalDevice = m_gpu;
    ii.Device = m_device;
    ii.QueueFamily = m_queue_family;
    ii.Queue = m_queue;
    ii.DescriptorPoolSize = 16; // the backend makes its own pool
    ii.MinImageCount = m_min_image_count;
    ii.ImageCount = (uint32_t)std::max<size_t>(m_min_image_count, m_swap_images.size());
    ii.PipelineInfoMain.RenderPass = m_render_pass;
    ii.PipelineInfoMain.Subpass = 0;
    ii.PipelineInfoMain.MSAASamples = m_samples;
    ii.CheckVkResultFn = imgui_check;
    if (!ImGui_ImplVulkan_Init(&ii))
    {
        if (error)
            *error = "ImGui_ImplVulkan_Init failed";
        return false;
    }
    m_imgui = true;
    return true;
}

void renderer::shutdown_imgui()
{
    if (m_imgui)
    {
        vkDeviceWaitIdle(m_device);
        ImGui_ImplVulkan_Shutdown();
        m_imgui = false;
    }
}

// ---------------------------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------------------------

bool renderer::render(const render_frame &rf, ImDrawData *imgui)
{
    if (m_swapchain_dirty || m_extent.width == 0 || m_extent.height == 0 || !m_swapchain)
    {
        vkDeviceWaitIdle(m_device);
        destroy_swapchain();
        std::string err;
        if (!create_swapchain(&err))
        {
            fprintf(stderr, "swapchain: %s\n", err.c_str());
            return false;
        }
        if (m_extent.width == 0 || m_extent.height == 0)
            return true; // minimised
    }

    frame_data &f = m_frames[m_frame_index];
    vkWaitForFences(m_device, 1, &f.fence, VK_TRUE, UINT64_MAX);

    uint32_t image_index = 0;
    VkResult r = vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, f.image_available, VK_NULL_HANDLE, &image_index);
    if (r == VK_ERROR_OUT_OF_DATE_KHR)
    {
        m_swapchain_dirty = true;
        return true;
    }
    if (r != VK_SUCCESS && r != VK_SUBOPTIMAL_KHR)
        return false;
    vkResetFences(m_device, 1, &f.fence);

    const uint32_t ninst = std::min(rf.instance_count, m_max_instances);
    if (ninst)
        memcpy(f.instances.mapped, rf.instances, ninst * sizeof(render_instance));

    VkCommandBuffer cmd = f.cmd;
    vkResetCommandBuffer(cmd, 0);
    VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);

    VkClearValue clears[3];
    // UNORM swapchain (ImGui colours are authored for it): everything is in display space.
    for (int i = 0; i < 4; ++i)
        clears[0].color.float32[i] = rf.clear[i];
    clears[1].depthStencil = {1.0f, 0};
    clears[2] = clears[0];
    VkRenderPassBeginInfo rpbi = {VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rpbi.renderPass = m_render_pass;
    rpbi.framebuffer = m_framebuffers[image_index];
    rpbi.renderArea.extent = m_extent;
    rpbi.clearValueCount = m_samples != VK_SAMPLE_COUNT_1_BIT ? 3 : 2;
    rpbi.pClearValues = clears;
    vkCmdBeginRenderPass(cmd, &rpbi, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport = {0, 0, (float)m_extent.width, (float)m_extent.height, 0.0f, 1.0f};
    VkRect2D scissor = {{0, 0}, m_extent};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    push_constants pc;
    memcpy(pc.viewproj, rf.viewproj.m, sizeof(pc.viewproj));
    const vec3 ld = normalize(rf.light_dir);
    pc.light_dir[0] = ld.x;
    pc.light_dir[1] = ld.y;
    pc.light_dir[2] = ld.z;
    pc.light_dir[3] = rf.ambient;
    pc.cam_pos[0] = rf.cam_pos.x;
    pc.cam_pos[1] = rf.cam_pos.y;
    pc.cam_pos[2] = rf.cam_pos.z;
    pc.cam_pos[3] = rf.fog_distance;
    pc.params[0] = rf.ground_half;
    pc.params[1] = rf.blob_radius;
    pc.params[2] = rf.ground_grid;
    pc.params[3] = 0.0f;
    vkCmdPushConstants(cmd, m_layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);

    const VkDeviceSize zero = 0;
    if (rf.ground)
    {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipe_ground);
        vkCmdDraw(cmd, 6, 1, 0, 0);
    }
    if (rf.blobs.instance_count && ninst)
    {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipe_blob);
        vkCmdBindVertexBuffers(cmd, 1, 1, &f.instances.buf, &zero);
        const uint32_t count = std::min(rf.blobs.instance_count, ninst - std::min(ninst, rf.blobs.first_instance));
        if (count)
            vkCmdDraw(cmd, 6, count, 0, rf.blobs.first_instance);
    }
    if (ninst)
    {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipe_mesh);
        for (const render_draw &d : rf.meshes)
        {
            if (d.mesh < 0 || d.mesh >= (int)m_meshes.size() || !d.instance_count || d.first_instance >= ninst)
                continue;
            const mesh &m = m_meshes[d.mesh];
            VkBuffer bufs[2] = {m.vb.buf, f.instances.buf};
            VkDeviceSize offs[2] = {0, 0};
            vkCmdBindVertexBuffers(cmd, 0, 2, bufs, offs);
            vkCmdBindIndexBuffer(cmd, m.ib.buf, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, m.index_count, std::min(d.instance_count, ninst - d.first_instance), 0, 0, d.first_instance);
        }
        if (!rf.lines.empty())
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipe_lines);
            for (const render_draw &d : rf.lines)
            {
                if (d.mesh < 0 || d.mesh >= (int)m_meshes.size() || !d.instance_count || d.first_instance >= ninst)
                    continue;
                const mesh &m = m_meshes[d.mesh];
                VkBuffer bufs[2] = {m.vb.buf, f.instances.buf};
                VkDeviceSize offs[2] = {0, 0};
                vkCmdBindVertexBuffers(cmd, 0, 2, bufs, offs);
                vkCmdBindIndexBuffer(cmd, m.ib.buf, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmd, m.index_count, std::min(d.instance_count, ninst - d.first_instance), 0, 0, d.first_instance);
            }
        }
    }

    if (imgui && m_imgui)
        ImGui_ImplVulkan_RenderDrawData(imgui, cmd);

    vkCmdEndRenderPass(cmd);

    // Optional capture: swapchain image -> host buffer (PRESENT_SRC -> TRANSFER_SRC -> PRESENT_SRC).
    buffer capture;
    const bool capturing = !m_capture_path.empty() &&
                           create_buffer((VkDeviceSize)m_extent.width * m_extent.height * 4, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &capture);
    if (capturing)
    {
        VkImageMemoryBarrier b = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        b.oldLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = m_swap_images[image_index];
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
        VkBufferImageCopy region = {};
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {m_extent.width, m_extent.height, 1};
        vkCmdCopyImageToBuffer(cmd, m_swap_images[image_index], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, capture.buf, 1, &region);
        b.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        b.dstAccessMask = 0;
        b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    }
    vkEndCommandBuffer(cmd);

    const VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.waitSemaphoreCount = 1;
    si.pWaitSemaphores = &f.image_available;
    si.pWaitDstStageMask = &wait_stage;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores = &m_render_done[image_index];
    if (vkQueueSubmit(m_queue, 1, &si, f.fence) != VK_SUCCESS)
        return false;
    if (capturing)
    {
        vkWaitForFences(m_device, 1, &f.fence, VK_TRUE, UINT64_MAX);
        void *pixels = nullptr;
        vkMapMemory(m_device, capture.mem, 0, VK_WHOLE_SIZE, 0, &pixels);
        if (FILE *bmp = fopen(m_capture_path.c_str(), "wb"))
        {
            const uint32_t w = m_extent.width, h = m_extent.height, size = w * h * 4;
            const bool rgba = m_surface_format.format == VK_FORMAT_R8G8B8A8_UNORM || m_surface_format.format == VK_FORMAT_R8G8B8A8_SRGB;
            uint8_t hdr[54] = {'B', 'M'};
            const uint32_t file_size = 54 + size, offset = 54, info = 40, planes_bpp = 1 | (32 << 16);
            const int32_t ww = (int32_t)w, hh = -(int32_t)h; // top-down
            memcpy(hdr + 2, &file_size, 4);
            memcpy(hdr + 10, &offset, 4);
            memcpy(hdr + 14, &info, 4);
            memcpy(hdr + 18, &ww, 4);
            memcpy(hdr + 22, &hh, 4);
            memcpy(hdr + 26, &planes_bpp, 4);
            memcpy(hdr + 34, &size, 4);
            fwrite(hdr, 1, 54, bmp);
            std::vector<uint8_t> row(w * 4);
            for (uint32_t y = 0; y < h; ++y)
            {
                memcpy(row.data(), (const uint8_t *)pixels + (size_t)y * w * 4, w * 4);
                if (rgba)
                    for (uint32_t x = 0; x < w; ++x)
                        std::swap(row[x * 4], row[x * 4 + 2]);
                fwrite(row.data(), 1, row.size(), bmp);
            }
            fclose(bmp);
        }
        vkUnmapMemory(m_device, capture.mem);
        destroy_buffer(capture);
        m_capture_path.clear();
    }

    VkPresentInfoKHR pi = {VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores = &m_render_done[image_index];
    pi.swapchainCount = 1;
    pi.pSwapchains = &m_swapchain;
    pi.pImageIndices = &image_index;
    r = vkQueuePresentKHR(m_queue, &pi);
    if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR)
        m_swapchain_dirty = true;
    else if (r != VK_SUCCESS)
        return false;

    m_frame_index = (m_frame_index + 1) % FRAMES;
    return true;
}

void renderer::shutdown()
{
    if (!m_device)
    {
        if (m_instance)
        {
            if (m_surface)
                vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
            vkDestroyInstance(m_instance, nullptr);
        }
        m_instance = VK_NULL_HANDLE;
        return;
    }
    vkDeviceWaitIdle(m_device);
    shutdown_imgui();
    for (mesh &m : m_meshes)
    {
        destroy_buffer(m.vb);
        destroy_buffer(m.ib);
    }
    m_meshes.clear();
    for (frame_data &f : m_frames)
    {
        destroy_buffer(f.instances);
        if (f.fence)
            vkDestroyFence(m_device, f.fence, nullptr);
        if (f.image_available)
            vkDestroySemaphore(m_device, f.image_available, nullptr);
        f = frame_data();
    }
    destroy_swapchain();
    if (m_swapchain)
        vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
    m_swapchain = VK_NULL_HANDLE;
    VkPipeline pipes[] = {m_pipe_mesh, m_pipe_ground, m_pipe_blob, m_pipe_lines};
    for (VkPipeline p : pipes)
        if (p)
            vkDestroyPipeline(m_device, p, nullptr);
    if (m_layout)
        vkDestroyPipelineLayout(m_device, m_layout, nullptr);
    if (m_render_pass)
        vkDestroyRenderPass(m_device, m_render_pass, nullptr);
    if (m_cmd_pool)
        vkDestroyCommandPool(m_device, m_cmd_pool, nullptr);
    vkDestroyDevice(m_device, nullptr);
    m_device = VK_NULL_HANDLE;
    if (m_surface)
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    vkDestroyInstance(m_instance, nullptr);
    m_instance = VK_NULL_HANDLE;
}
