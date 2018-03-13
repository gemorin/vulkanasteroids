#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#pragma clang diagnostic pop

#include <cube.h>

using namespace std;

static constexpr bool enableValidation = true;

class VulkanApp
{
    GLFWwindow *window = nullptr;
    VkInstance instance;
    VkSurfaceKHR surface;
    VkDebugReportCallbackEXT callback;

    struct PhysicalDeviceInfo {
        VkPhysicalDevice device = VK_NULL_HANDLE;
        VkPhysicalDeviceFeatures deviceFeatures;
        VkSurfaceCapabilitiesKHR capabilities;

        // idx 0 is graphics, 1 is presentation
        uint32_t families[2];

        // color depth
        VkSurfaceFormatKHR format;
        // how we display images
        VkPresentModeKHR presentMode;
        VkExtent2D extent;
        uint32_t imageCount;

        bool hasUniqueFamily() const {
            return families[0] == families[1];
        }
    } devInfo;
    VkDevice device;
    VkQueue presentationQueue;
    VkQueue graphicsQueue;

    VkSwapchainKHR vkSwapChain;
    struct SwapChainEntry {
        VkImage image;
        VkImageView view;

        // Anti aliasing stuff
        VkImage msaaImage;
        VkDeviceMemory msaaMemory;
        VkImageView msaaView;

        VkImage depthImage;
        VkDeviceMemory depthImageMemory;
        VkImageView depthImageView;

        // Synchronization
        VkSemaphore imageAvailableSem;
        VkSemaphore renderFinishedSem;
        VkFence fence;
    };
    vector<SwapChainEntry> swapChain;

    VkRenderPass renderPass;

    // Right now we use the same layout
    struct Descriptor {
        VkDescriptorPool pool;
        VkDescriptorSet set;
    };
    // Vertex attribute buffers
    struct __attribute__((packed)) Vertex {
        MyPoint pos;
        MyPoint color;  // for debugging
        float u,v;

        Vertex() = default;
        Vertex(MyPoint p, MyPoint _color, float _u, float _v)
            : pos(p), color(_color), u(_u), v(_v) {}
    };
    struct VertexBuffer {
        vector<Vertex> vertices;
        VkBuffer buffer;
        VkDeviceMemory memory;

        void cleanup(VkDevice device) {
            vkDestroyBuffer(device, buffer, nullptr);
            vkFreeMemory(device, memory, nullptr);
        }
    };
    struct Texture {
        VkImage image;
        VkDeviceMemory memory;
        VkImageView view;
        uint32_t width;
        uint32_t height;
        VkSampler sampler;

        void cleanup(VkDevice device)
        {
            vkDestroyImageView(device, view, nullptr);
            vkFreeMemory(device, memory, nullptr);
            vkDestroyImage(device, image, nullptr);
            vkDestroySampler(device, sampler, nullptr);
        }
    };
    VkDescriptorSetLayout descriptorSetLayout;

    // Background
    Descriptor backgroundDescriptor;
    VkShaderModule backgroundVertexShader;
    VkShaderModule backgroundFragmentShader;
    VertexBuffer backgroundVertex;
    Texture background;

    // Ship
    Descriptor shipDescriptor;
    Texture ship;
    VkShaderModule shipVertexShader;
    VkShaderModule shipFragmentShader;
    VertexBuffer shipVertex;

    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;

    vector<VkFramebuffer> frameBuffers;

    VkCommandPool commandPool;
    vector<VkCommandBuffer> commandBuffers;

    struct __attribute__((packed)) VpUniform {
        MyMatrix view;
        MyMatrix proj;
    } vp;

    // Uniforms
    VkBuffer vpUniformBuffer;
    VkDeviceMemory vpUniformMemory;
    VpUniform *vpUniformPtr;

   public:
    VulkanApp() = default;
    ~VulkanApp() = default;

    VulkanApp(VulkanApp&) = delete;
    VulkanApp& operator=(const VulkanApp&) = delete;

    void run();

  private:
    bool readFile(vector<char> *data, const char *filename);
    VkPresentModeKHR choosePresentMode(const VkPresentModeKHR* modes,
                                       uint32_t modeCount);
    VkSurfaceFormatKHR chooseSwapSurfaceFormat(
                                             const VkSurfaceFormatKHR* formats,
                                             uint32_t formatCount);

    static void glfwErrorCallback(int /*error*/, const char* description)
    {
        puts(description);
    }

    bool init();
    bool initGlFw();
    bool initVulkanInstance();
    bool createSurface();
    bool choosePhysicalDevice();
    bool createLogicalDevice();
    bool createSwapChain();
    bool loadShaders();
    bool createRenderPass();
    bool createDescriptorSetLayout();
    bool createPipeline();
    bool createFrameBuffers();
    bool createDepthResources();
    bool createCommandPool();
    bool createCommandBuffers();
    bool setupCommandBuffers();
    bool createVertexBuffers();
    bool createUniformBuffers();
    bool createDescriptors();
    bool createBackgroundDescriptor();
    bool createShipDescriptor();
    bool createTextures();
    bool setupDebugCallback();
    bool createShaders(VkShaderModule *vs, VkShaderModule *fs,
                       const char *vertexPath, const char *fragPath);
    bool createTexture(struct Texture *texture, const char *filename);

    void cleanup();
    void cleanupSwapChain();

    void waitForIdle();
    bool recreateSwapChain();
    void onResize(int width, int height);
    void onKey(int key, int action);
    void updateExtent();
    void resetVp();

    bool createBuffer(VkBuffer *buffer, VkDeviceMemory *bufferMemory,
                      VkDeviceSize size, VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags properties, bool isShared);
    uint32_t findMemoryType(const VkMemoryRequirements& memReqs,
                            VkMemoryPropertyFlags typeFilter);
    VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates,
                                 VkImageTiling tiling,
                                 VkFormatFeatureFlags features);
    VkFormat findDepthFormat();
    void transitionImageLayout(VkImage image, VkFormat format,
                               VkImageLayout oldLayout,
                               VkImageLayout newLayout);
    VkCommandBuffer beginSingleTimeCommands();
    void endSingleTimeCommands(VkCommandBuffer commandBuffer);

    void copyBufferToImage(VkBuffer buffer, VkImage image,
                           uint32_t width, uint32_t height);

    static bool hasStencilComponent(VkFormat format) {
        return format == VK_FORMAT_D32_SFLOAT_S8_UINT
            || format == VK_FORMAT_D24_UNORM_S8_UINT;
    }

    bool renderFrame(uint32_t renderCount, double currentTime);

    // Glfw glue
    static void glfw_onResize(GLFWwindow * window, int width, int height)
    {
        VulkanApp *app = (VulkanApp *) glfwGetWindowUserPointer(window);
        app->onResize(width, height);
    }
    static void glfw_onKey(GLFWwindow *window, int key, int /*scancode*/,
                           int action, int /*mods*/)
    {
        VulkanApp *app = (VulkanApp *) glfwGetWindowUserPointer(window);
        app->onKey(key, action);
    }

    // Utils
    static MyMatrix perspective(float fovy, float aspect, float n, float f);
    static MyMatrix ortho(float bottom, float top, float left, float right,
                          float near, float far);

     // Debug stuff
     static void DestroyDebugReportCallbackEXT(VkInstance instance,
                                    VkDebugReportCallbackEXT callback,
                                    const VkAllocationCallbacks* pAllocator);
     static VkResult CreateDebugReportCallbackEXT(
                         VkInstance instance,
                         const VkDebugReportCallbackCreateInfoEXT* pCreateInfo,
                         const VkAllocationCallbacks* pAllocator,
                         VkDebugReportCallbackEXT* pCallback);
     static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
                                         VkDebugReportFlagsEXT flags,
                                         VkDebugReportObjectTypeEXT objType,
                                         uint64_t obj, size_t location,
                                         int32_t code, const char* layerPrefix,
                                         const char* msg, void* userData);
};

VkResult VulkanApp::CreateDebugReportCallbackEXT(
                        VkInstance instance,
                        const VkDebugReportCallbackCreateInfoEXT* pCreateInfo,
                        const VkAllocationCallbacks* pAllocator,
                        VkDebugReportCallbackEXT* pCallback) {
    auto func = (PFN_vkCreateDebugReportCallbackEXT)
            vkGetInstanceProcAddr(instance, "vkCreateDebugReportCallbackEXT");
    if (func != nullptr) {
        return func(instance, pCreateInfo, pAllocator, pCallback);
    } else {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void VulkanApp::DestroyDebugReportCallbackEXT(VkInstance instance,
                                   VkDebugReportCallbackEXT callback,
                                   const VkAllocationCallbacks* pAllocator) {
    auto func = (PFN_vkDestroyDebugReportCallbackEXT)
        vkGetInstanceProcAddr(instance, "vkDestroyDebugReportCallbackEXT");
    if (func != nullptr) {
        func(instance, callback, pAllocator);
    }
}

VKAPI_ATTR VkBool32 VKAPI_CALL
VulkanApp::debugCallback(VkDebugReportFlagsEXT /*flags*/,
                         VkDebugReportObjectTypeEXT /*objType*/,
                         uint64_t /*obj*/, size_t /*location*/,
                         int32_t /*code*/, const char* /*layerPrefix*/,
                         const char* msg, void* /*userData*/)
{
    printf("validation layer: %s\n", msg);
    return VK_FALSE;
}

MyMatrix VulkanApp::perspective(float fovy, float aspect, float n, float f)
{
    const float q = 1.0 / tan((0.5 * double(fovy)) * (M_PI/180.0));
    const float A = q / aspect;
    const float B = double(n + f) / double(n - f);
    const float C = n * f / (f - n);

    MyMatrix result;
    result.set(0, 0, A);
    result.set(1, 1, -q);
    result.set(2, 2, B);
    result.set(2, 3, C);
    result.set(3, 2, -1.0f);
    return result;
}

MyMatrix VulkanApp::ortho(float bottom, float top, float left, float right,
                          float near, float far)
{
    MyMatrix result;
    result.set(0, 0, 2.0f / (right - left));
    result.set(0, 3, -(right + left) / (right - left));
    result.set(1, 1, 2.0f / (top - bottom));
    result.set(1, 3, -(top + bottom) / (top - bottom));
    result.set(2, 2, -2.0f / (far - near));
    result.set(2, 3, -(far + near) / (far - near));
    result.print();
    puts("");

    MyMatrix correct;
    correct.set(1, 1, -1.0f);
    correct.set(2, 2, 0.5f);
    correct.set(2, 3, 0.5f);

    result = correct * result;
    result.print();


    return result;
}

bool VulkanApp::init()
{
    if (!initGlFw()
     || !initVulkanInstance()
     || !setupDebugCallback()
     || !createSurface()
     || !choosePhysicalDevice()
     || !createLogicalDevice()
     || !createSwapChain()
     || !loadShaders()
     || !createRenderPass()
     || !createDescriptorSetLayout()
     || !createPipeline()
     || !createCommandPool()
     || !createCommandBuffers()
     || !createDepthResources()
     || !createFrameBuffers()
     || !createTextures()
     || !createVertexBuffers()
     || !createUniformBuffers()
     || !createDescriptors()
     || !setupCommandBuffers())
        return false;
    return true;
}

void VulkanApp::run()
{
    if (!init())
        return;

    uint32_t renderCount = 0;
    while(1) {
        bool running = renderFrame(renderCount++, glfwGetTime());

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_RELEASE)
            running = false;
        if (glfwWindowShouldClose(window))
            running = false;
        if (!running) {
            break;
        }
    }

    waitForIdle();
    cleanup();
}

void VulkanApp::waitForIdle()
{
    for (auto & swpe : swapChain)
        vkWaitForFences(device, 1, &swpe.fence, VK_TRUE, UINT64_MAX);
    vkDeviceWaitIdle(device);
}

bool VulkanApp::readFile(vector<char> *buf, const char *filename)
{
    FILE *fd = fopen(filename, "r");
    if (!fd)
        return false;
    size_t numRead = 0;
    while (1) {
        buf->resize(numRead + 4096);
        size_t ret = fread(buf->data() + numRead, 1, 4096, fd);
        numRead += ret;
        if (ret != 4096)
            break;
    }
    buf->resize(numRead);
    fclose(fd);
    return true;
}

VkPresentModeKHR VulkanApp::choosePresentMode(const VkPresentModeKHR* modes,
                                              uint32_t modeCount)
{
    // This is guaranteed to be avail per spec but can be buggy
    VkPresentModeKHR bestMode = VK_PRESENT_MODE_FIFO_KHR;
    for (uint32_t i = 0; i < modeCount; ++i) {
        if (modes[i] == VK_PRESENT_MODE_MAILBOX_KHR)
            return VK_PRESENT_MODE_MAILBOX_KHR;
        if (modes[i] == VK_PRESENT_MODE_IMMEDIATE_KHR)
            bestMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    }
    return bestMode;
}

VkSurfaceFormatKHR VulkanApp::chooseSwapSurfaceFormat(
                                             const VkSurfaceFormatKHR* formats,
                                             uint32_t formatCount)
{
    if (formatCount == 1 && formats[0].format == VK_FORMAT_UNDEFINED) {
        // Guaranteed to be avail in this case
        return {VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
    }
    for (uint32_t i = 0; i < formatCount; ++i) {
        if (formats[i].format == VK_FORMAT_B8G8R8A8_UNORM
         && formats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            return formats[i];
    }
    return formats[0];
}

bool VulkanApp::initGlFw() {
    // Init glfw
    glfwInit();
    glfwSetErrorCallback(glfwErrorCallback);

    // This prevents glfw from creating a gl context
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    window = glfwCreateWindow(800, 600, "Vulkan", nullptr, nullptr);
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowSizeCallback(window, &VulkanApp::glfw_onResize);
    glfwSetKeyCallback(window, &VulkanApp::glfw_onKey);

    VkExtensionProperties properties[16];
    uint32_t extensionCount = 16;
    vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount,
                                           properties);

    printf("%u extensions supported: ", extensionCount);
    for (unsigned i = 0; i < extensionCount; ++i) {
        if (i)
            printf(", ");
        printf("%s", properties[i].extensionName);
    }
    puts("");
    if (glfwVulkanSupported() != GLFW_TRUE) {
        puts("glfw does not support vulkan! Upgrade to the latest version");
        return false;
    }
    return true;
}

bool VulkanApp::initVulkanInstance() {
    VkApplicationInfo appInfo = {};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Vulkan triangle";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "DeadCanard's Engine";
    appInfo.engineVersion = VK_MAKE_VERSION(0, 0, 1);
    appInfo.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;

    uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(
                                                          &glfwExtensionCount);
     vector<const char*> extensions(glfwExtensions,
                              glfwExtensions + glfwExtensionCount);
     if (enableValidation)
         extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);


     for (uint32_t i = 0; i < extensions.size(); ++i) {
           if (i == 0) {
             printf("Enabled extensions: %s", extensions[i]);
          }
          else {
             printf(", %s", extensions[i]);
          }
     }
     puts("");
     createInfo.enabledExtensionCount = extensions.size();
     createInfo.ppEnabledExtensionNames = extensions.data();

    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    VkLayerProperties layers[layerCount];
    vkEnumerateInstanceLayerProperties(&layerCount, layers);
    vector<char *> enabledLayers;
    for (uint32_t i = 0; i < layerCount; ++i) {
        char *layerName = layers[i].layerName;
        if (i == 0) {
            printf("Available layers: %s", layerName);
        }
        else {
            printf(", %s", layerName);
        }
        if (0 == strcmp("VK_LAYER_LUNARG_standard_validation", layerName)
          || 0 == strcmp("VK_LAYER_LUNARG_core_validation", layerName)) {
            if (enableValidation)
                enabledLayers.push_back(layerName);
        }
    }
    puts("");

    for (uint32_t i = 0; i < enabledLayers.size(); ++i) {
        if (i == 0) {
            printf("Enabled layers: %s", enabledLayers[i]);
        }
        else {
            printf(", %s", enabledLayers[i]);
        }
    }
    if (!enabledLayers.empty())
        puts("");

    createInfo.enabledLayerCount = enabledLayers.size();
    createInfo.ppEnabledLayerNames = enabledLayers.data();

    VkResult vkRet = vkCreateInstance(&createInfo, nullptr, &instance);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateInstance failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createSurface() {
    VkResult vkRet = glfwCreateWindowSurface(instance, window, nullptr,
                                             &surface);
    if (vkRet != VK_SUCCESS) {
        printf("failed to create surface: glfwCreateWindowSurface() failed "
               "with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::setupDebugCallback()
{
    if (!enableValidation)
        return true;
    VkDebugReportCallbackCreateInfoEXT createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
    createInfo.flags = VK_DEBUG_REPORT_ERROR_BIT_EXT |
                       VK_DEBUG_REPORT_WARNING_BIT_EXT;
    createInfo.pfnCallback = debugCallback;

    VkResult vkRet;
    vkRet = CreateDebugReportCallbackEXT(instance, &createInfo, nullptr,
                                         &callback);
   if (vkRet != VK_SUCCESS) {
       printf("CreateDebugReportCallbackEXT failed with %d\n", vkRet);
       return false;
   }
   return true;
}

void VulkanApp::updateExtent()
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width, &height);
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(devInfo.device, surface,
                                              &devInfo.capabilities);
    if (devInfo.capabilities.currentExtent.width != UINT_MAX) {
        devInfo.extent = devInfo.capabilities.currentExtent;
    }
    else {
        devInfo.extent.width =
            max(devInfo.capabilities.minImageExtent.width,
                min(devInfo.capabilities.maxImageExtent.width,
                    (unsigned) width));
        devInfo.extent.height =
            max(devInfo.capabilities.minImageExtent.height,
                min(devInfo.capabilities.maxImageExtent.height,
                    (unsigned) height));
    }
    devInfo.extent.width = width;
    devInfo.extent.height = height;
}

bool VulkanApp::choosePhysicalDevice()
{
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
    if (deviceCount == 0) {
        puts("Found no physical device");
        return false;
    }
    VkPhysicalDevice devices[deviceCount];
    vkEnumeratePhysicalDevices(instance, &deviceCount, devices);
    for (unsigned i = 0; i < deviceCount; ++i) {
        // We're looking for a device that has
        // * graphics family queue
        // * presentation family queue
        // * swap chain khr extension
        // * valid swap chain format/present mode
        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(devices[i], &properties);

        if (properties.deviceType != VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            // Only consider discreate cpu type
            continue;
        }
        // XXX check for anisotropic filtering
        vkGetPhysicalDeviceFeatures(devices[i], &devInfo.deviceFeatures);

        uint32_t extensionCount;
        vkEnumerateDeviceExtensionProperties(devices[i], nullptr,
                                             &extensionCount, nullptr);
        if (extensionCount == 0)
            continue;

        VkExtensionProperties extProps[extensionCount];
        vkEnumerateDeviceExtensionProperties(devices[i], nullptr,
                                             &extensionCount, extProps);
        bool hasSwapChain = false;
        for (unsigned k = 0; k < extensionCount; ++k) {
            if (0 == strcmp(extProps[k].extensionName,
                            VK_KHR_SWAPCHAIN_EXTENSION_NAME)) {
                hasSwapChain = true;
                break;
            }
        }
        if (!hasSwapChain)
            continue;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(devices[i], &queueFamilyCount,
                                                 nullptr);
        VkQueueFamilyProperties queueProps[queueFamilyCount];
        vkGetPhysicalDeviceQueueFamilyProperties(devices[i], &queueFamilyCount,
                                                 queueProps);
        uint32_t graphicsFamily;
        uint32_t presentationFamily;
        bool graphicsFamilySet = false;
        bool presentationFamilySet = false;
        for (unsigned j = 0; j != queueFamilyCount; ++j) {
            if (queueProps[j].queueCount <= 0)
                continue;

            if (queueProps[j].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                graphicsFamily = j;
                graphicsFamilySet = true;
            }
            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(devices[i], j, surface,
                                                 &presentSupport);
            if (presentSupport) {
                presentationFamily = j;
                presentationFamilySet = true;
            }
        }
        if (!presentationFamilySet || !graphicsFamilySet)
            continue;

        uint32_t formatCount;
        vkGetPhysicalDeviceSurfaceFormatsKHR(devices[i], surface, &formatCount,
                                             nullptr);
        if (0 == formatCount) {
            continue;
        }
        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(devices[i], surface,
                                                  &presentModeCount,
                                                  nullptr);
        if (0 == presentModeCount) {
            continue;
        }

        // Ok, we're going to use this device.  Populate devInfo
        devInfo.device = devices[i];
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(devices[i], surface,
                                                  &devInfo.capabilities);
        updateExtent();

        VkSurfaceFormatKHR formats[formatCount];
        vkGetPhysicalDeviceSurfaceFormatsKHR(devices[i], surface,
                                             &formatCount, formats);
        devInfo.format = chooseSwapSurfaceFormat(formats, formatCount);

        VkPresentModeKHR presentModes[presentModeCount];
        vkGetPhysicalDeviceSurfacePresentModesKHR(devices[i], surface,
                                                  &presentModeCount,
                                                  presentModes);
        devInfo.presentMode = choosePresentMode(presentModes,
                                                presentModeCount);

        uint32_t imgCount = devInfo.capabilities.minImageCount + 1;
        if (devInfo.capabilities.maxImageCount > 0)
            imgCount = min(devInfo.capabilities.maxImageCount, imgCount);
        devInfo.imageCount = imgCount;
        devInfo.families[0] = graphicsFamily;
        devInfo.families[1] = presentationFamily;
        printf("Using device %s\n", properties.deviceName);
        return true;
    }
    return false;
}

// Create logical device
bool VulkanApp::createLogicalDevice() {
    const uint32_t numQueues = devInfo.hasUniqueFamily() ? 1 : 2;

    VkDeviceQueueCreateInfo queueCreateInfo[numQueues];
    memset(queueCreateInfo, 0, sizeof(queueCreateInfo));
    float priority[1] = {1.0f};
    for (unsigned i = 0; i < numQueues; ++i) {
        queueCreateInfo[i].sType =
                                VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo[i].queueFamilyIndex = devInfo.families[i];
        // Only one queue of each type
        queueCreateInfo[i].queueCount = 1;
        queueCreateInfo[i].pQueuePriorities = priority;
    }

    // Note that we've already verified what we need is supported when
    // picking the physical device.
    uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(devInfo.device, nullptr,
                                         &extensionCount, nullptr);
    VkExtensionProperties extProps[extensionCount];
    vkEnumerateDeviceExtensionProperties(devInfo.device, nullptr,
                                         &extensionCount, extProps);
    char *extNames[extensionCount];
    for (unsigned i = 0; i < extensionCount; ++i) {
        extNames[i] = extProps[i].extensionName;
    }

    VkDeviceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pQueueCreateInfos = queueCreateInfo;
    createInfo.queueCreateInfoCount = numQueues;
    // FIXME Enabling all features - maybe we probably want something
    // more targeted
    createInfo.pEnabledFeatures = &devInfo.deviceFeatures;
    createInfo.enabledExtensionCount = extensionCount;
    createInfo.ppEnabledExtensionNames = extNames;
    // FIXME validation here at some point
    createInfo.enabledLayerCount = 0;

    VkResult vkRet = vkCreateDevice(devInfo.device, &createInfo, nullptr,
                                    &device);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDevice failed with %d\n", vkRet);
        return false;
    }
    vkGetDeviceQueue(device, devInfo.families[0], 0, &graphicsQueue);
    vkGetDeviceQueue(device, devInfo.families[1], 0, &presentationQueue);
    return true;
}

uint32_t VulkanApp::findMemoryType(const VkMemoryRequirements& memReqs,
                                   VkMemoryPropertyFlags typeFilter)
{
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(devInfo.device, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if (memReqs.memoryTypeBits & (1U << i)) {
            if (memProps.memoryTypes[i].propertyFlags & typeFilter) {
                return i;
            }
        }
    }
    // Well, let's go with 0
    return 0;
}

bool VulkanApp::createSwapChain()
{
    VkSwapchainCreateInfoKHR createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = surface;
    createInfo.minImageCount = devInfo.imageCount;
    createInfo.imageFormat = devInfo.format.format;
    createInfo.imageColorSpace = devInfo.format.colorSpace;
    createInfo.imageExtent = devInfo.extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    createInfo.imageSharingMode = devInfo.hasUniqueFamily()
                                ? VK_SHARING_MODE_EXCLUSIVE
                                : VK_SHARING_MODE_CONCURRENT;
    createInfo.queueFamilyIndexCount = devInfo.hasUniqueFamily() ? 0 : 2;
    createInfo.pQueueFamilyIndices = devInfo.hasUniqueFamily()
                                   ? nullptr
                                   : devInfo.families;
    createInfo.preTransform = devInfo.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = devInfo.presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    VkResult vkRet = vkCreateSwapchainKHR(device, &createInfo, nullptr,
                                          &vkSwapChain);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSwapchainKHR failed with %d\n", vkRet);
        return false;
    }
    // Vulkan is allowed to create a bigger swap chain than requested so let's
    // figure out the size
    uint32_t size;
    vkGetSwapchainImagesKHR(device, vkSwapChain, &size, nullptr);

    swapChain.resize(size);

    // Now get the images
    VkImage images[size];
    vkGetSwapchainImagesKHR(device, vkSwapChain, &size, images);
    for (uint32_t i = 0; i < size; ++i) {
        swapChain[i].image = images[i];
    }

    // Initialize the rest of the entries
    for (auto& swpe : swapChain) {
        // Image views
        VkImageViewCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        createInfo.image = swpe.image;
        createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        createInfo.format = devInfo.format.format;

        createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        createInfo.subresourceRange.baseMipLevel = 0;
        createInfo.subresourceRange.levelCount = 1;
        createInfo.subresourceRange.baseArrayLayer = 0;
        createInfo.subresourceRange.layerCount = 1;

        VkResult vkRet = vkCreateImageView(device, &createInfo, nullptr,
                                           &swpe.view);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateImageView failed with %d\n", vkRet);
            return false;
        }

        // Synchronization apparatus
        VkSemaphoreCreateInfo semaphoreInfo = {};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        vkRet = vkCreateSemaphore(device, &semaphoreInfo, nullptr,
                                  &swpe.imageAvailableSem);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateSemaphore failed with %d\n", vkRet);
            return false;
        }
        vkRet = vkCreateSemaphore(device, &semaphoreInfo, nullptr,
                                  &swpe.renderFinishedSem);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateSemaphore failed with %d\n", vkRet);
            return false;
        }

        VkFenceCreateInfo fenceCreateInfo = {};
        fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceCreateInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        vkRet = vkCreateFence(device, &fenceCreateInfo, nullptr, &swpe.fence);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateFence failed with %d\n", vkRet);
            return false;
        }

        // MSAA init
        VkImageCreateInfo info = {};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = devInfo.format.format;
        info.extent.width = devInfo.extent.width;
        info.extent.height = devInfo.extent.height;
        info.extent.depth = 1;
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = VK_SAMPLE_COUNT_4_BIT;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        // This image will only be used as a transient render target.  Its
        // purpose is only to hold the multisampled data before resolving the
        // render pass.
        info.usage = VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT |
                     VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        // Create texture.
        vkRet = vkCreateImage(device, &info, nullptr, &swpe.msaaImage);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateImage failed with %d\n", vkRet);
            return false;
        }

        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, swpe.msaaImage, &memReqs);
        VkMemoryAllocateInfo alloc = { };
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReqs.size;
        // For multisampled attachments, we will want to use LAZILY
        // allocated if such a type is available.
        alloc.memoryTypeIndex = findMemoryType(
                                      memReqs,
                                      VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT);
        vkRet = vkAllocateMemory(device, &alloc, nullptr, &swpe.msaaMemory);

        if (vkRet != VK_SUCCESS) {
            printf("vkAllocateMemory failed with %d\n", vkRet);
            return false;
        }
        vkBindImageMemory(device, swpe.msaaImage, swpe.msaaMemory, 0);

        VkImageViewCreateInfo viewInfo = { };
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = swpe.msaaImage;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = devInfo.format.format;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_G;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_B;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_A;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;

        vkRet = vkCreateImageView(device, &viewInfo, nullptr, &swpe.msaaView);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateImageView failed with %d\n", vkRet);
            return false;
        }
    }
    return true;
}

bool VulkanApp::createShaders(VkShaderModule *vs,
                              VkShaderModule *fs,
                              const char *vertexPath,
                              const char *fragPath)
{
    vector<char> shader;
    if (!readFile(&shader, vertexPath)) {
        printf("Could not read %s\n", vertexPath);
        return false;
    }

    VkShaderModuleCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = shader.size();
    createInfo.pCode = (uint32_t *) shader.data();

    VkResult vkRet;
    vkRet = vkCreateShaderModule(device, &createInfo, nullptr, vs);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateShaderModule failed with %d\n", vkRet);
        return false;
    }
    if (!readFile(&shader, fragPath)) {
        printf("Could not read %s\n", fragPath);
        return false;
    }
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = shader.size();
    createInfo.pCode = (uint32_t *) shader.data();

    vkRet = vkCreateShaderModule(device, &createInfo, nullptr, fs);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateShaderModule failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::loadShaders() {
    if (!createShaders(&backgroundVertexShader, &backgroundFragmentShader,
                       "vertex_background.spv", "fragment_background.spv")) {
        return false;
    }
    return true;
}

bool VulkanApp::createRenderPass()
{
    // MSAA attachment
    // from https://arm-software.github.io/vulkan-sdk/multisampling.html
    VkAttachmentDescription attachments[3];
    memset(&attachments, 0, sizeof(attachments));
    attachments[0].format = devInfo.format.format;
    attachments[0].samples = VK_SAMPLE_COUNT_4_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    // Don't write to memory, we just want to compute
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    // this does not go to the presentation
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    attachments[1].format = devInfo.format.format;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT; // required for resolve
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    attachments[2].format = findDepthFormat();
    attachments[2].samples = VK_SAMPLE_COUNT_4_BIT;
    attachments[2].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[2].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[2].finalLayout =
                              VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    // The color attachment is referenced by
    // 'layout (location = 0) out vec4 outColor' in the frag shader
    VkAttachmentReference colorRef = {};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference resolveRef = {};
    resolveRef.attachment = 1;
    resolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthRef = {};
    depthRef.attachment = 2;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;


    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pResolveAttachments = &resolveRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dependency = {};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                               VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo = {};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = 3;
    renderPassInfo.pAttachments = attachments;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    VkResult vkRet =  vkCreateRenderPass(device, &renderPassInfo, nullptr,
                                         &renderPass);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateRenderPass failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createDescriptorSetLayout()
{
    // Right now we use only one layout for all - XXX may change later
    VkDescriptorSetLayoutBinding layout[2];
    memset(layout, 0, sizeof(layout));
    layout[0].binding = 0;
    layout[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    layout[0].descriptorCount = 1;
    layout[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    layout[0].pImmutableSamplers = nullptr;

    layout[1].binding = 1;
    layout[1].descriptorCount = 1;
    layout[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layout[1].pImmutableSamplers = nullptr;
    layout[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo = {};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = sizeof(layout)/ sizeof(*layout);
    layoutInfo.pBindings = layout;

    VkResult vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                                 &descriptorSetLayout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createPipeline()
{
    // Stader stages
    VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
    vertShaderStageInfo.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = backgroundVertexShader;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
    fragShaderStageInfo.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = backgroundFragmentShader;
    fragShaderStageInfo.pName = "main";
    VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo,
                                                      fragShaderStageInfo};

    // Fixed function stages

    // vertext input
    // Point input: vertices, colors, normals
    // XXX fix this
    VkVertexInputBindingDescription bindingDescriptions[1];
    memset(bindingDescriptions, 0, sizeof(bindingDescriptions));
    bindingDescriptions[0].binding = 0;
    bindingDescriptions[0].stride = sizeof(Vertex);
    bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
#if 0
    // The bindings are different because they're different buffers.
    bindingDescriptions[1].binding = 1;
    bindingDescriptions[1].stride = sizeof(MyPoint);
    bindingDescriptions[1].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
#endif

    VkVertexInputAttributeDescription vertexInputDescriptions[3];
    memset(vertexInputDescriptions, 0, sizeof(vertexInputDescriptions));
    // Vertices
    vertexInputDescriptions[0].binding = 0;
    vertexInputDescriptions[0].location = 0;
    vertexInputDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    vertexInputDescriptions[0].offset = offsetof(Vertex, pos);
    // Colors
    vertexInputDescriptions[1].binding = 0;
    vertexInputDescriptions[1].location = 1;
    vertexInputDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    vertexInputDescriptions[1].offset = offsetof(Vertex, color);
    // uv
    vertexInputDescriptions[2].binding = 0;
    vertexInputDescriptions[2].location = 2;
    vertexInputDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
    vertexInputDescriptions[2].offset = offsetof(Vertex, u);

    VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
    vertexInputInfo.sType =
                VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount =
        sizeof(bindingDescriptions)/sizeof(*bindingDescriptions);
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions;
    vertexInputInfo.vertexAttributeDescriptionCount =
        sizeof(vertexInputDescriptions)/sizeof(*vertexInputDescriptions);
    vertexInputInfo.pVertexAttributeDescriptions = vertexInputDescriptions;

    // Using triangles
    VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
    inputAssembly.sType =
                   VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // View port
    VkViewport viewport = {};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = (float) devInfo.extent.width;
    viewport.height = (float) devInfo.extent.height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor = {};
    scissor.offset = {0, 0};
    scissor.extent = devInfo.extent;

    VkPipelineViewportStateCreateInfo viewportState = {};
    viewportState.sType =
                         VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    // Rasterizer
    VkPipelineRasterizationStateCreateInfo rasterizer = {};
    rasterizer.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    // Try VK_POLYGON_MODE_LINE, VK_POLYGON_MODE_POINT
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f;
    rasterizer.depthBiasClamp = 0.0f;
    rasterizer.depthBiasSlopeFactor = 0.0f;

    // MSAA
    VkPipelineMultisampleStateCreateInfo multisampling = {};
    multisampling.sType =
                      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_4_BIT;
    multisampling.minSampleShading = 1.0f;
    multisampling.pSampleMask = nullptr;
    multisampling.alphaToCoverageEnable = VK_FALSE;
    multisampling.alphaToOneEnable = VK_FALSE;

    // Stencil/depth buffer
    VkPipelineDepthStencilStateCreateInfo depthStencil = {};
    depthStencil.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.minDepthBounds = 0.0f; // Optional
    depthStencil.maxDepthBounds = 1.0f; // Optional
    depthStencil.stencilTestEnable = VK_FALSE;
    depthStencil.front = {}; // Optional
    depthStencil.back = {}; // Optional

    // Color blending
    VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
                                          VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT |
                                          VK_COLOR_COMPONENT_A_BIT;
    // no alpha blending
    colorBlendAttachment.blendEnable = VK_FALSE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending = {};
    colorBlending.sType =
                      VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY;
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;
    colorBlending.blendConstants[0] = 0.0f;
    colorBlending.blendConstants[1] = 0.0f;
    colorBlending.blendConstants[2] = 0.0f;
    colorBlending.blendConstants[3] = 0.0f;

    // vulkan dynamic states
    // VkPipelineDynamicStateCreateInfo dynamicState = {};

    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType =
                            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 0;
    pipelineLayoutInfo.pPushConstantRanges = 0;

    VkResult vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo,
                                            nullptr, &pipelineLayout);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreatePipelineLayout failed with ret %d\n", vkRet);
       return false;
    }

    VkGraphicsPipelineCreateInfo pipelineInfo = {};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = nullptr;
    pipelineInfo.layout = pipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;
    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
    pipelineInfo.basePipelineIndex = -1;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &graphicsPipeline);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreateGraphicsPipelines failed with ret %d\n", vkRet);
       return false;
    }
    return true;
}

bool VulkanApp::createFrameBuffers()
{
    frameBuffers.resize(swapChain.size());
    for (unsigned i = 0; i != swapChain.size(); ++i) {
        VkImageView attachments[] = { swapChain[i].msaaView,
                                      swapChain[i].view,
                                      swapChain[i].depthImageView
                                    };
        VkFramebufferCreateInfo framebufferInfo = {};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = 3;
        framebufferInfo.pAttachments = attachments;
        framebufferInfo.width = devInfo.extent.width;
        framebufferInfo.height = devInfo.extent.height;
        framebufferInfo.layers = 1;

        VkResult vkRet = vkCreateFramebuffer(device, &framebufferInfo,
                                             nullptr, &frameBuffers[i]);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateFramebuffer failed with %d\n", vkRet);
            return false;
        }
    }
    return true;
}

VkFormat VulkanApp::findSupportedFormat(
                             const std::vector<VkFormat>& candidates,
                             VkImageTiling tiling,
                             VkFormatFeatureFlags features)
{
    for (VkFormat format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(devInfo.device, format, &props);

        if (tiling == VK_IMAGE_TILING_LINEAR
         && (props.linearTilingFeatures & features) == features) {
            return format;
        }
        if (tiling == VK_IMAGE_TILING_OPTIMAL
        && (props.optimalTilingFeatures & features) == features) {
            return format;
        }
    }

    abort();
}

VkFormat VulkanApp::findDepthFormat() {
    return findSupportedFormat(
        {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
         VK_FORMAT_D24_UNORM_S8_UINT
        },
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
    );
}

bool VulkanApp::createDepthResources()
{
    const VkFormat depthFormat = findDepthFormat();
    for (unsigned i = 0; i != swapChain.size(); ++i) {
        VkImageCreateInfo info = {};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = depthFormat;
        info.extent.width = devInfo.extent.width;
        info.extent.height = devInfo.extent.height;
        info.extent.depth = 1;
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = VK_SAMPLE_COUNT_4_BIT;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VkResult vkRet;
        vkRet = vkCreateImage(device, &info, nullptr, &swapChain[i].depthImage);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateImage failed with %d\n", vkRet);
            return false;
        }

        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, swapChain[i].depthImage, &memReqs);
        VkMemoryAllocateInfo alloc = { };
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReqs.size;
        alloc.memoryTypeIndex = findMemoryType(
                                      memReqs,
                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkRet = vkAllocateMemory(device, &alloc, nullptr,
                                 &swapChain[i].depthImageMemory);

        if (vkRet != VK_SUCCESS) {
            printf("vkAllocateMemory failed with %d\n", vkRet);
            return false;
        }
        vkBindImageMemory(device,swapChain[i].depthImage,
                          swapChain[i].depthImageMemory, 0);


        VkImageViewCreateInfo viewInfo = { };
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = swapChain[i].depthImage;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = depthFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_G;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_B;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_A;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;

        vkRet = vkCreateImageView(device, &viewInfo, nullptr,
                                  &swapChain[i].depthImageView);
        if (vkRet != VK_SUCCESS) {
            printf("vkCreateImageView failed with %d\n", vkRet);
            return false;
        }

        transitionImageLayout(swapChain[i].depthImage, depthFormat,
                              VK_IMAGE_LAYOUT_UNDEFINED,
                              VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    }
    return true;
}

VkCommandBuffer VulkanApp::beginSingleTimeCommands()
{
    VkCommandBufferAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = commandPool;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    return commandBuffer;
}

void VulkanApp::endSingleTimeCommands(VkCommandBuffer commandBuffer)
{
    vkEndCommandBuffer(commandBuffer);

    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);

    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
}

void VulkanApp::transitionImageLayout(VkImage image, VkFormat format,
                                      VkImageLayout oldLayout,
                                      VkImageLayout newLayout)
{
    VkCommandBuffer commandBuffer = beginSingleTimeCommands();

    VkImageMemoryBarrier barrier = {};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;

    if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

        if (hasStencilComponent(format)) {
            barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
        }
    } else {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    }

    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    VkPipelineStageFlags sourceStage;
    VkPipelineStageFlags destinationStage;

    if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED
     && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

        sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
            && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    } else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED
            && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    } else {
        abort();
    }

    vkCmdPipelineBarrier(
        commandBuffer,
        sourceStage, destinationStage,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );

    endSingleTimeCommands(commandBuffer);
}

bool VulkanApp::createCommandPool()
{
    VkCommandPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = devInfo.families[0];
    poolInfo.flags = 0;

    VkResult vkRet = vkCreateCommandPool(device, &poolInfo, nullptr,
                                         &commandPool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateCommandPool failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createCommandBuffers()
{
    // Command buffers
    VkCommandBufferAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = swapChain.size();

    commandBuffers.resize(swapChain.size());
    VkResult vkRet;
    vkRet = vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data());
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateCommandPool failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createBuffer(VkBuffer *buffer,
                             VkDeviceMemory *bufferMemory,
                             VkDeviceSize size,
                             VkBufferUsageFlags usage,
                             VkMemoryPropertyFlags properties,
                             bool isShared)
{
    VkBufferCreateInfo bufferInfo = {};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = isShared
                           ? VK_SHARING_MODE_CONCURRENT
                           : VK_SHARING_MODE_EXCLUSIVE;

    VkResult vkRet = vkCreateBuffer(device, &bufferInfo, nullptr, buffer);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateBuffer failed with %d\n", vkRet);
        return false;
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(device, *buffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements, properties);

    vkRet = vkAllocateMemory(device, &allocInfo, nullptr, bufferMemory);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateMemory failed with %d\n", vkRet);
        return false;
    }
    vkBindBufferMemory(device, *buffer, *bufferMemory, 0);

    return true;
}

bool VulkanApp::createVertexBuffers()
{
    const float height = (float) devInfo.extent.height;
    const float width = (float) devInfo.extent.width;
    const float aspect = height / width;
    float bottom = -aspect / 2.0f;
    printf("bottom is %f\n", bottom);
    float top = bottom + float(background.height) / float(background.width);
    printf("top of texture %f\n", top);
    constexpr float z = 0.0f;

    // First triangle of background
    constexpr MyPoint red(1.0f, 0.0f, 0.0f);
    auto* v = &backgroundVertex.vertices;
    v->emplace_back(MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f, bottom, z}, red, 1.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f);
    // 2nd
    v->emplace_back(MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f);
    v->emplace_back(MyPoint{-0.5f,    top, z}, red, 0.0f, 0.0f);

    uint32_t numBytes = v->size() * sizeof(*v->data());
    const VkBufferUsageFlags vertexUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    if (!createBuffer(&backgroundVertex.buffer, &backgroundVertex.memory,
                      numBytes, vertexUsage, memFlags, false)) {
        return false;
    }

    void* data;
    vkMapMemory(device, backgroundVertex.memory, 0, numBytes, 0, &data);
    memcpy(data, v->data(), numBytes);
    vkUnmapMemory(device, backgroundVertex.memory);

#if 0
    // ship
    v = &shipVertex.vertices;
    v->emplace_back(MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f, bottom, z}, red, 1.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f);
    // 2nd
    v->emplace_back(MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f);
    v->emplace_back(MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f);
    v->emplace_back(MyPoint{-0.5f,    top, z}, red, 0.0f, 0.0f);
#endif

    return true;
}

void VulkanApp::resetVp()
{
    // FIXME
#if 0
    const float aspect = (float) devInfo.extent.width /
                         (float) devInfo.extent.height;
    cubeRot.rotateY(M_PI / 4.0f); // 45deg
    mvp.model = cubeRot.toMatrix();
    memcpy(mvpUniformPtr, &mvp, sizeof(mvp));
#endif
    const float height = (float) devInfo.extent.height;
    const float width = (float) devInfo.extent.width;
    const float aspect = height / width;
    float bottom = -aspect / 2.0f;
    printf("bottom is %f\n", bottom);
    vp.proj = ortho(bottom, -bottom, -0.5f, 0.5f, -1.0f, 1.0f);
    //vp.proj = ortho(-5.0f, 5.0f, -5.0f, 5.0f, -1.0f, 1.0f);
    //vp.proj = perspective(50.0f, aspect, 0.1f, 1000.0f);
    //vp.view.set(2, 3, -0.2f);
    memcpy(vpUniformPtr, &vp, sizeof(vp));
}

bool VulkanApp::createUniformBuffers()
{
    const VkBufferUsageFlags usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
#if 0
    if (!createBuffer(&cubeTransformsUniformBuffer,
                      &cubeTransformsUniformBufferMemory,
                      sizeof(rubik.mTransforms), usage, memFlags, false)) {
        return false;
    }

    // Initialize
    vkMapMemory(device, cubeTransformsUniformBufferMemory, 0,
                sizeof(rubik.mTransforms), 0, &cubeTransformsUniformBufferPtr);
    memcpy(cubeTransformsUniformBufferPtr, rubik.mTransforms,
           sizeof(rubik.mTransforms));
#endif

    if (!createBuffer(&vpUniformBuffer, &vpUniformMemory, sizeof(vp), usage,
                      memFlags, false)) {
        return false;
    }

    // init
    vkMapMemory(device, vpUniformMemory, 0, sizeof(vp), 0,
                (void **) &vpUniformPtr);
    resetVp();

    return true;
}

bool VulkanApp::createBackgroundDescriptor()
{
    VkDescriptorPoolSize poolSizes[2];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 2;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &backgroundDescriptor.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetLayout layouts[] = {descriptorSetLayout};
    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = backgroundDescriptor.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = layouts;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo,
                                     &backgroundDescriptor.set);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateDescriptorSets failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorBufferInfo bufferInfo[1];
    memset(bufferInfo, 0, sizeof(bufferInfo));
    bufferInfo[0].buffer = vpUniformBuffer;
    bufferInfo[0].offset = 0;
    bufferInfo[0].range = sizeof(vp);

    VkDescriptorImageInfo imageInfo = {};
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo.imageView = background.view;
    imageInfo.sampler = background.sampler;

    VkWriteDescriptorSet descriptorWrite[2];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = backgroundDescriptor.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pBufferInfo = bufferInfo;
    descriptorWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[1].dstSet = backgroundDescriptor.set;
    descriptorWrite[1].dstBinding = 1;
    descriptorWrite[1].dstArrayElement = 0;
    descriptorWrite[1].descriptorType =
                                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite[1].descriptorCount = 1;
    descriptorWrite[1].pImageInfo = &imageInfo;

    vkUpdateDescriptorSets(device,
                           sizeof(descriptorWrite)/sizeof(*descriptorWrite),
                           descriptorWrite, 0, nullptr);

    return true;
}

bool VulkanApp::createShipDescriptor()
{
    VkDescriptorPoolSize poolSizes[2];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 2;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &shipDescriptor.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetLayout layouts[] = {descriptorSetLayout};
    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = shipDescriptor.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = layouts;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo,
                                     &shipDescriptor.set);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateDescriptorSets failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorBufferInfo bufferInfo[1];
    memset(bufferInfo, 0, sizeof(bufferInfo));
    bufferInfo[0].buffer = vpUniformBuffer;
    bufferInfo[0].offset = 0;
    bufferInfo[0].range = sizeof(vp);

    VkDescriptorImageInfo imageInfo = {};
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo.imageView = ship.view;
    imageInfo.sampler = ship.sampler;

    VkWriteDescriptorSet descriptorWrite[2];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = shipDescriptor.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pBufferInfo = bufferInfo;
    descriptorWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[1].dstSet = shipDescriptor.set;
    descriptorWrite[1].dstBinding = 1;
    descriptorWrite[1].dstArrayElement = 0;
    descriptorWrite[1].descriptorType =
                                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite[1].descriptorCount = 1;
    descriptorWrite[1].pImageInfo = &imageInfo;

    vkUpdateDescriptorSets(device,
                           sizeof(descriptorWrite)/sizeof(*descriptorWrite),
                           descriptorWrite, 0, nullptr);

    return true;
}

bool VulkanApp::createDescriptors()
{
    if (!createBackgroundDescriptor()
     || !createShipDescriptor()) {
        return false;
    }
    return true;
}

bool VulkanApp::setupCommandBuffers()
{
    // Setup command buffers
    for (unsigned i = 0; i < commandBuffers.size(); ++i) {
        VkCommandBuffer& b = commandBuffers[i];
        VkCommandBufferBeginInfo beginInfo = {};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
        beginInfo.pInheritanceInfo = nullptr;

        vkBeginCommandBuffer(b, &beginInfo);

        VkRenderPassBeginInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = renderPass;
        renderPassInfo.framebuffer = frameBuffers[i];
        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = devInfo.extent;

        VkClearValue clearColor[3] = {};
        memset(clearColor, 0, sizeof(clearColor));
        //clearColor[0].color.float32[0] = 210.0f / 255.0f;
        //clearColor[0].color.float32[1] = 230.0f / 255.0f;
        //clearColor[0].color.float32[2] = 255.0f / 255.0f;
        //clearColor[0].color.float32[3] = 1.0f;
        //clearColor[1] is for the resolve attachment
        clearColor[2].depthStencil = {1.0f, 0};
        renderPassInfo.clearValueCount = 3;
        renderPassInfo.pClearValues = clearColor;

        vkCmdBeginRenderPass(b, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                          graphicsPipeline);

        // XXX FIXME
        VkBuffer buffers[] = {backgroundVertex.buffer};// , colorBuffer};
        VkDeviceSize offsets[] = {0}; //,0};
        vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
        vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipelineLayout, 0, 1,
                                &backgroundDescriptor.set, 0, nullptr);

        vkCmdDraw(b, backgroundVertex.vertices.size(), 1, 0, 0);

        vkCmdEndRenderPass(b);

        VkResult vkRet = vkEndCommandBuffer(b);
        if (vkRet != VK_SUCCESS) {
            printf("vkEndCommandBuffer failed with %d\n", vkRet);
            return false;
        }
    }
    return true;
}

bool VulkanApp::renderFrame(uint32_t renderCount, double currentTime)
{
    (void) currentTime; // remove when we actually render stuff
    // Draw
    const uint32_t idx = renderCount % swapChain.size();

    vkWaitForFences(device, 1, &swapChain[idx].fence, VK_TRUE, UINT64_MAX);
    vkResetFences(device, 1, &swapChain[idx].fence);

    VkResult vkRet;
    uint32_t imageIndex;
    vkRet = vkAcquireNextImageKHR(device, vkSwapChain, ULONG_MAX,
                                  swapChain[idx].imageAvailableSem,
                                  VK_NULL_HANDLE, &imageIndex);
    if (vkRet != VK_SUCCESS) {
        // XXX Handle VK_SUBOPTIMAL_KHR VK_ERROR_OUT_OF_DATE_KHR
        printf("vkAcquireNextImageKHR returned %d\n", vkRet);
    }

    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = {swapChain[idx].imageAvailableSem};
    VkPipelineStageFlags waitStages[] = {
                                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffers[imageIndex];

    VkSemaphore signalSemaphores[] = {swapChain[idx].renderFinishedSem};
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    vkRet = vkQueueSubmit(graphicsQueue, 1, &submitInfo, swapChain[idx].fence);
    if (vkRet != VK_SUCCESS) {
        printf("vkQueueSubmit failed with %d\n", vkRet);
        return false;
    }

    VkPresentInfoKHR presentInfo = {};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    VkSwapchainKHR swapChains[] = {vkSwapChain};
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &imageIndex;
    presentInfo.pResults = nullptr;

    vkRet = vkQueuePresentKHR(presentationQueue, &presentInfo);
    if (vkRet != VK_SUCCESS) {
        // XXX Handle VK_SUBOPTIMAL_KHR VK_ERROR_OUT_OF_DATE_KHR
        printf("vkQueuePresentKHR returned %d\n", vkRet);
    }

    return true;
}

bool VulkanApp::recreateSwapChain()
{
    // XXX FIXME
    waitForIdle();

    updateExtent();

    cleanupSwapChain();
    if (!createSwapChain()
     || !createDepthResources()
     || !loadShaders()
     || !createRenderPass()
     || !createPipeline()
     || !createFrameBuffers()
     || !createCommandBuffers()
     || !setupCommandBuffers())
        return false;
    return true;
}

void VulkanApp::cleanupSwapChain()
{
    vkFreeCommandBuffers(device, commandPool,
                         static_cast<uint32_t>(commandBuffers.size()),
                         commandBuffers.data());
    for (unsigned i = 0; i < swapChain.size(); ++i) {
        vkDestroyFramebuffer(device, frameBuffers[i], nullptr);
    }
    vkDestroyPipeline(device, graphicsPipeline, nullptr);
    vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
    vkDestroyRenderPass(device, renderPass, nullptr);
    vkDestroyShaderModule(device, backgroundVertexShader, nullptr);
    vkDestroyShaderModule(device, backgroundFragmentShader, nullptr);
    for (auto& swpe : swapChain) {
        vkDestroyFence(device, swpe.fence, nullptr);
        vkDestroySemaphore(device, swpe.imageAvailableSem, nullptr);
        vkDestroySemaphore(device, swpe.renderFinishedSem, nullptr);
        vkDestroyImageView(device, swpe.view, nullptr);
        // Note that the swpe.image is owned by and will be deallocated
        // through vkSwapChain
        vkDestroyImageView(device, swpe.msaaView, nullptr);
        vkFreeMemory(device, swpe.msaaMemory, nullptr);
        vkDestroyImage(device, swpe.msaaImage, nullptr);
        vkDestroyImageView(device, swpe.depthImageView, nullptr);
        vkFreeMemory(device, swpe.depthImageMemory, nullptr);
        vkDestroyImage(device, swpe.depthImage, nullptr);
    }
    background.cleanup(device);
    ship.cleanup(device);

    vkDestroySwapchainKHR(device, vkSwapChain, nullptr);
}

void VulkanApp::cleanup()
{
    cleanupSwapChain();
    vkDestroyCommandPool(device, commandPool, nullptr);

    backgroundVertex.cleanup(device);

    //vkDestroyBuffer(device, cubeTransformsUniformBuffer, nullptr);
    //vkUnmapMemory(device, cubeTransformsUniformBufferMemory);
    //vkFreeMemory(device, cubeTransformsUniformBufferMemory, nullptr);

    vkDestroyBuffer(device, vpUniformBuffer, nullptr);
    vkUnmapMemory(device, vpUniformMemory);
    vkFreeMemory(device, vpUniformMemory, nullptr);

    vkDestroyDescriptorPool(device, backgroundDescriptor.pool, nullptr);
    vkDestroyDescriptorPool(device, shipDescriptor.pool, nullptr);
    vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyDevice(device, nullptr);
    if (enableValidation)
        DestroyDebugReportCallbackEXT(instance, callback, nullptr);
    vkDestroyInstance(instance, nullptr);
    glfwDestroyWindow(window);

    glfwTerminate();
}

void VulkanApp::onResize(int width, int height)
{
    // Not actually used for now, the window is not resizable
    if (width == 0 || height == 0)
        return;
    recreateSwapChain();
}

void VulkanApp::onKey(int /*key*/, int /*action*/)
{
}

void VulkanApp::copyBufferToImage(VkBuffer buffer, VkImage image,
                                  uint32_t width, uint32_t height)
{
    VkCommandBuffer commandBuffer = beginSingleTimeCommands();

    VkBufferImageCopy region = {};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {
        width,
        height,
        1
    };

    vkCmdCopyBufferToImage(commandBuffer, buffer, image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    endSingleTimeCommands(commandBuffer);
}


bool VulkanApp::createTexture(struct Texture *texture,
                              const char *filename)
{
    int texWidth, texHeight, texChannels;
    stbi_uc* pixels = stbi_load(filename, &texWidth, &texHeight,
                                &texChannels, STBI_rgb_alpha);
    VkDeviceSize imageSize = texWidth * texHeight * 4;

    if (!pixels || texWidth < 0 || texHeight < 0) {
        printf("stbi_load failed\n");
        return false;
    }
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer(&stagingBuffer, &stagingBufferMemory,
                 imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                 VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 false);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
        memcpy(data, pixels, static_cast<size_t>(imageSize));
    vkUnmapMemory(device, stagingBufferMemory);
    stbi_image_free(pixels);
    texture->width = texWidth;
    texture->height = texHeight;

    VkImageCreateInfo imageInfo = {};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = texture->width;
    imageInfo.extent.height = texture->height;
    imageInfo.extent.depth = 1; // required for 2D image
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                      VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    // sampling is just for images used as attachment
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

    VkResult vkRet;
    vkRet = vkCreateImage(device, &imageInfo, nullptr, &texture->image);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateImage failed with %d\n", vkRet);
        return false;
    }
    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(device, texture->image, &memRequirements);

    VkMemoryAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vkRet = vkAllocateMemory(device, &allocInfo, nullptr,
                             &texture->memory);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateMemory failed with %d\n", vkRet);
        return false;
    }
    vkBindImageMemory(device, texture->image, texture->memory, 0);

    transitionImageLayout(texture->image, VK_FORMAT_R8G8B8A8_UNORM,
                          VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    copyBufferToImage(stagingBuffer, texture->image, (uint32_t)texWidth,
                      (uint32_t) texHeight);
    transitionImageLayout(texture->image, VK_FORMAT_R8G8B8A8_UNORM,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);

    VkImageViewCreateInfo viewInfo = {};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = texture->image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    vkRet = vkCreateImageView(device, &viewInfo, nullptr, &texture->view);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateImageView failed with %d\n", vkRet);
        return false;
    }

    VkSamplerCreateInfo samplerInfo = {};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.anisotropyEnable = VK_TRUE;
    samplerInfo.maxAnisotropy = 16;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = 0.0f;

    vkRet = vkCreateSampler(device, &samplerInfo, nullptr, &texture->sampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }

    return true;
}

bool VulkanApp::createTextures() {
    if (!createTexture(&background, "assets/background.png")
     || !createTexture(&ship, "assets/ship.png"))
        return false;
    return true;
}

int main(void)
{
    VulkanApp app;
    app.run();

    return 0;
}
