#pragma once

// Vulkan renderer for the demo: one graphics+present queue, two frames in flight, a classic render pass with
// optional MSAA (resolved into the swapchain image), no descriptor sets for the scene (push constants + an
// instance vertex stream), and Dear ImGui drawn in the same pass. No allocator library: one VkDeviceMemory per
// buffer / image (there are only a handful).

#include "demo_math.h"
#include "vk_loader.h"

#include <cstdint>
#include <string>
#include <vector>

struct SDL_Window;
struct ImDrawData;

struct render_vertex
{
    float pos[3];
    float nrm[3];
};

// Instance stream: phys_mat44 rows (x, y, z axes, translation) and a tint. 80 bytes.
struct render_instance
{
    float r[4][4];
    float tint[4];
};

struct render_draw
{
    int mesh = -1;
    uint32_t first_instance = 0;
    uint32_t instance_count = 0;
};

struct render_frame
{
    mat4 viewproj;
    vec3 cam_pos;
    vec3 light_dir = vec3(-0.35f, -0.25f, -0.9f);
    float ambient = 0.55f;
    float fog_distance = 9000.0f;
    float ground_half = 4000.0f;
    float ground_grid = 64.0f;
    float blob_radius = 30.0f;
    float clear[4] = {0.62f, 0.68f, 0.76f, 1.0f};
    const render_instance *instances = nullptr;
    uint32_t instance_count = 0;
    std::vector<render_draw> meshes; // lit instanced meshes
    std::vector<render_draw> lines;  // instanced line meshes (collision hull overlay)
    render_draw blobs;               // blob shadows (mesh ignored)
    bool ground = true;
};

class renderer
{
public:
    bool init(SDL_Window *window, std::string *error);
    void shutdown();

    // ImGui: call after ImGui::CreateContext + ImGui_ImplSDL2_InitForVulkan.
    bool init_imgui(std::string *error);
    void shutdown_imgui();

    int create_mesh(const render_vertex *verts, uint32_t vert_count, const uint32_t *indices, uint32_t index_count);
    int create_lines(const float *positions, uint32_t vert_count, const uint32_t *indices, uint32_t index_count);

    // Records and presents one frame. false = device lost / fatal.
    bool render(const render_frame &frame, ImDrawData *imgui);

    void set_vsync(bool vsync);

    // Saves the next presented frame as a 32-bit BMP (self-check for -shot).
    void request_capture(const char *path) { m_capture_path = path ? path : ""; }
    bool capture_pending() const { return !m_capture_path.empty(); }
    void notify_resize() { m_swapchain_dirty = true; }

    uint32_t max_instances() const { return m_max_instances; }
    const char *device_name() const { return m_device_name.c_str(); }
    int msaa_samples() const { return (int)m_samples; }
    VkExtent2D extent() const { return m_extent; }

private:
    struct buffer
    {
        VkBuffer buf = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        VkDeviceSize size = 0;
        void *mapped = nullptr;
    };
    struct image
    {
        VkImage img = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
    };
    struct mesh
    {
        buffer vb, ib;
        uint32_t index_count = 0;
    };
    struct frame_data
    {
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
        VkSemaphore image_available = VK_NULL_HANDLE;
        buffer instances;
    };

    bool create_device(std::string *error);
    bool create_swapchain(std::string *error);
    void destroy_swapchain();
    bool create_render_pass(std::string *error);
    bool create_pipelines(std::string *error);
    VkPipeline make_pipeline(const uint32_t *vs, size_t vs_size, const uint32_t *fs, size_t fs_size, int vertex_mode,
                             VkPrimitiveTopology topology, bool depth_write, bool blend, VkCullModeFlags cull);

    uint32_t find_memory(uint32_t type_bits, VkMemoryPropertyFlags flags, VkMemoryPropertyFlags fallback = 0) const;
    bool create_buffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, buffer *out);
    bool create_static_buffer(const void *data, VkDeviceSize size, VkBufferUsageFlags usage, buffer *out);
    void destroy_buffer(buffer &b);
    bool create_image(uint32_t w, uint32_t h, VkFormat format, VkImageUsageFlags usage, VkSampleCountFlagBits samples,
                      VkImageAspectFlags aspect, image *out);
    void destroy_image(image &i);

    SDL_Window *m_window = nullptr;
    VkInstance m_instance = VK_NULL_HANDLE;
    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
    VkPhysicalDevice m_gpu = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties m_mem_props = {};
    VkDevice m_device = VK_NULL_HANDLE;
    uint32_t m_queue_family = 0;
    VkQueue m_queue = VK_NULL_HANDLE;
    VkCommandPool m_cmd_pool = VK_NULL_HANDLE;
    std::string m_device_name;
    uint32_t m_api_version = VK_API_VERSION_1_0;

    VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
    VkSurfaceFormatKHR m_surface_format = {};
    VkFormat m_depth_format = VK_FORMAT_UNDEFINED;
    VkExtent2D m_extent = {0, 0};
    uint32_t m_min_image_count = 2;
    std::vector<VkImage> m_swap_images;
    std::vector<VkImageView> m_swap_views;
    std::vector<VkFramebuffer> m_framebuffers;
    std::vector<VkSemaphore> m_render_done; // one per swapchain image
    image m_color_msaa, m_depth;
    VkSampleCountFlagBits m_samples = VK_SAMPLE_COUNT_1_BIT;
    bool m_swapchain_dirty = false;
    bool m_vsync = true;

    VkRenderPass m_render_pass = VK_NULL_HANDLE;
    VkPipelineLayout m_layout = VK_NULL_HANDLE;
    VkPipeline m_pipe_mesh = VK_NULL_HANDLE;
    VkPipeline m_pipe_ground = VK_NULL_HANDLE;
    VkPipeline m_pipe_blob = VK_NULL_HANDLE;
    VkPipeline m_pipe_lines = VK_NULL_HANDLE;

    static const int FRAMES = 2;
    frame_data m_frames[FRAMES];
    int m_frame_index = 0;
    uint32_t m_max_instances = 4096;

    std::vector<mesh> m_meshes;
    bool m_imgui = false;
    std::string m_capture_path;
};
