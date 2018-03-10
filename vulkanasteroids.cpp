#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>

#include <cube.h>

using namespace std;

class VulkanApp
{
    GLFWwindow *window = nullptr;
    VkInstance instance;
    VkSurfaceKHR surface;

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

    // Shaders
    VkShaderModule vertexShader;
    VkShaderModule fragShader;

    VkRenderPass renderPass;

    VkDescriptorSetLayout descriptorSetLayout;
    VkDescriptorPool descriptorPool;
    VkDescriptorSet descriptorSet;

    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;

    vector<VkFramebuffer> frameBuffers;

    VkCommandPool commandPool;
    vector<VkCommandBuffer> commandBuffers;

    MyRubik rubik;

    struct __attribute__((packed)) MvpUniform {
        MyMatrix model;
        MyMatrix view;
        MyMatrix proj;
    } mvp;

    // Vertex attribute buffers
    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer colorBuffer;
    VkDeviceMemory colorBufferMemory;

    // Uniforms
    VkBuffer cubeTransformsUniformBuffer;
    VkDeviceMemory cubeTransformsUniformBufferMemory;
    void *cubeTransformsUniformBufferPtr;
    VkBuffer mvpUniformBuffer;
    VkDeviceMemory mvpUniformMemory;
    MvpUniform *mvpUniformPtr;

    // Rotation states
    int keyPress = -1;
    int currentMoveKey = -1;
    double rotStartTime;
    static constexpr double cubeRotTime = 0.3; // view rot take 300ms
    MyQuaternion cubeRotStart;
    MyQuaternion cubeRotEnd;
    MyQuaternion cubeRot;

   public:
    VulkanApp();
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
    bool createCommandBuffers();
    bool setupCommandBuffers();
    bool createVertexBuffers();
    bool createUniformBuffers();
    bool createDescriptorPool();

    void cleanup();
    void cleanupSwapChain();

    void waitForIdle();
    bool recreateSwapChain();
    void onResize(int width, int height);
    void onKey(int key, int action);
    void updateExtent();
    void resetMvp();

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
    bool hasStencilComponent(VkFormat format) {
        return format == VK_FORMAT_D32_SFLOAT_S8_UINT
            || format == VK_FORMAT_D24_UNORM_S8_UINT;
    }

    void startCubeRot(double currentTime);
    void processCubeRot(double currentTime);
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
};

VulkanApp::VulkanApp()
{
    rubik.initialize();
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

bool VulkanApp::init()
{
    if (!initGlFw()
     || !initVulkanInstance()
       // Setup debug cb
     || !createSurface()
     || !choosePhysicalDevice()
     || !createLogicalDevice()
     || !createSwapChain()
     || !loadShaders()
     || !createRenderPass()
     || !createDescriptorSetLayout()
     || !createPipeline()
     || !createCommandBuffers()
     || !createDepthResources()
     || !createFrameBuffers()
     || !createVertexBuffers()
     || !createUniformBuffers()
     || !createDescriptorPool()
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
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
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
    createInfo.enabledExtensionCount = glfwExtensionCount;
    createInfo.ppEnabledExtensionNames = glfwExtensions;

    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    VkLayerProperties layers[layerCount];
    vkEnumerateInstanceLayerProperties(&layerCount, layers);
    char *enabledLayers[layerCount];
    for (uint32_t i = 0; i < layerCount; ++i) {
        char *layerName = layers[i].layerName;
        printf("Enabled layers: %s", layerName);
        if (i == 0) {
        }
        else {
            printf(", %s", layerName);
        }
        // We enable everything for now
        enabledLayers[i] = layerName;
    }
    puts("");

    createInfo.enabledLayerCount = layerCount;
    createInfo.ppEnabledLayerNames = enabledLayers;

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
    createInfo.pQueueCreateInfos = queueCreateInfo;
    createInfo.queueCreateInfoCount = numQueues;
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
        info.usage = VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
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

bool VulkanApp::loadShaders() {
    vector<char> shader;
    if (!readFile(&shader, "vertex.spv")) {
        printf("Could not read vertex.spv\n");
        return false;
    }

    VkShaderModuleCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = shader.size();
    createInfo.pCode = (uint32_t *) shader.data();

    VkResult vkRet;
    vkRet = vkCreateShaderModule(device, &createInfo, nullptr, &vertexShader);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateShaderModule failed with %d\n", vkRet);
        return false;
    }
    if (!readFile(&shader, "fragment.spv")) {
        printf("Could not read fragment.spv\n");
        return false;
    }
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = shader.size();
    createInfo.pCode = (uint32_t *) shader.data();

    vkRet = vkCreateShaderModule(device, &createInfo, nullptr, &fragShader);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateShaderModule failed with %d\n", vkRet);
        return false;
    }
    return true;
}

bool VulkanApp::createRenderPass()
{
    // MSAA attachment
    // from https://arm-software.github.io/vulkan-sdk/multisampling.html
    VkAttachmentDescription attachments[3];
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
    attachments[1].samples = VK_SAMPLE_COUNT_4_BIT; // 4 or 1?
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    attachments[2].format = findDepthFormat();
    attachments[2].samples = VK_SAMPLE_COUNT_1_BIT;
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
    VkDescriptorSetLayoutBinding uboLayoutBindings[2];
    memset(uboLayoutBindings, 0, sizeof(uboLayoutBindings));
    uboLayoutBindings[0].binding = 0;
    uboLayoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBindings[0].descriptorCount = 1;
    uboLayoutBindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    uboLayoutBindings[0].pImmutableSamplers = nullptr;

    uboLayoutBindings[1].binding = 1;
    uboLayoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBindings[1].descriptorCount = 1;
    uboLayoutBindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    uboLayoutBindings[1].pImmutableSamplers = nullptr;

    VkDescriptorSetLayoutCreateInfo layoutInfo = {};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = uboLayoutBindings;

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
    vertShaderStageInfo.module = vertexShader;
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
    fragShaderStageInfo.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShader;
    fragShaderStageInfo.pName = "main";
    VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo,
                                                      fragShaderStageInfo};

    // Fixed function stages

    // vertext input
    // Point input: vertices, colors, normals
    VkVertexInputBindingDescription bindingDescriptions[2];
    memset(bindingDescriptions, 0, sizeof(bindingDescriptions));
    bindingDescriptions[0].binding = 0;
    bindingDescriptions[0].stride = sizeof(MyPoint);
    bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    // The bindings are different because they're different buffers.
    bindingDescriptions[1].binding = 1;
    bindingDescriptions[1].stride = sizeof(MyPoint);
    bindingDescriptions[1].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription vertexInputDescriptions[2];
    memset(vertexInputDescriptions, 0, sizeof(vertexInputDescriptions));
    // Vertices
    vertexInputDescriptions[0].binding = 0;
    vertexInputDescriptions[0].location = 0;
    vertexInputDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    vertexInputDescriptions[0].offset = 0;
    // Colors
    vertexInputDescriptions[1].binding = 1;
    vertexInputDescriptions[1].location = 1;
    vertexInputDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    vertexInputDescriptions[1].offset = 0;

    VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
    vertexInputInfo.sType =
                VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = 2;
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions;
    vertexInputInfo.vertexAttributeDescriptionCount = 2;
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

    // vulkan dynamic states - XXX resizing
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

bool VulkanApp::createCommandBuffers()
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

    // Command buffers
    VkCommandBufferAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = swapChain.size();

    commandBuffers.resize(swapChain.size());
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
    const VkBufferUsageFlags vertexUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    // First our vertices
    if (!createBuffer(&vertexBuffer, &vertexBufferMemory, sizeof(rubik.cubes),
                      vertexUsage, memFlags, false)) {
        return false;
    }
    void* data;
    vkMapMemory(device, vertexBufferMemory, 0, sizeof(rubik.cubes), 0, &data);
    memcpy(data, rubik.cubes, sizeof(rubik.cubes));
    vkUnmapMemory(device, vertexBufferMemory);

    // Then colors
    if (!createBuffer(&colorBuffer, &colorBufferMemory, sizeof(rubik.colors),
                      vertexUsage, memFlags, false)) {
        return false;
    }
    vkMapMemory(device, colorBufferMemory, 0, sizeof(rubik.colors), 0, &data);
    memcpy(data, rubik.colors, sizeof(rubik.colors));
    vkUnmapMemory(device, colorBufferMemory);

    return true;
}

void VulkanApp::resetMvp()
{
    const float aspect = (float) devInfo.extent.width /
                         (float) devInfo.extent.height;
    cubeRot.rotateY(M_PI / 4.0f); // 45deg
    mvp.model = cubeRot.toMatrix();
    mvp.view.set(2, 3, -5.0f);
    mvp.proj = perspective(50.0f, aspect, 0.1f, 1000.0f);
    memcpy(mvpUniformPtr, &mvp, sizeof(mvp));
}

bool VulkanApp::createUniformBuffers()
{
    const VkBufferUsageFlags usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
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

    if (!createBuffer(&mvpUniformBuffer, &mvpUniformMemory, sizeof(mvp), usage,
                      memFlags, false)) {
        return false;
    }

    // init
    vkMapMemory(device, mvpUniformMemory, 0, sizeof(mvp), 0,
                (void **) &mvpUniformPtr);
    resetMvp();

    return true;
}

bool VulkanApp::createDescriptorPool()
{
    VkDescriptorPoolSize poolSize = {};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &descriptorPool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetLayout layouts[] = {descriptorSetLayout};
    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = layouts;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorBufferInfo bufferInfo[2];
    memset(bufferInfo, 0, sizeof(bufferInfo));
    bufferInfo[0].buffer = mvpUniformBuffer;
    bufferInfo[0].offset = 0;
    bufferInfo[0].range = sizeof(mvp);
    bufferInfo[1].buffer = cubeTransformsUniformBuffer;
    bufferInfo[1].offset = 0;
    bufferInfo[1].range = sizeof(rubik.mTransforms);

    VkWriteDescriptorSet descriptorWrite[2];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = descriptorSet;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pBufferInfo = bufferInfo;
    descriptorWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[1].dstSet = descriptorSet;
    descriptorWrite[1].dstBinding = 1;
    descriptorWrite[1].dstArrayElement = 0;
    descriptorWrite[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[1].descriptorCount = 1;
    descriptorWrite[1].pBufferInfo = bufferInfo + 1;

    vkUpdateDescriptorSets(device, 2, descriptorWrite, 0, nullptr);

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
        clearColor[0].color.float32[0] = 210.0f / 255.0f;
        clearColor[0].color.float32[1] = 230.0f / 255.0f;
        clearColor[0].color.float32[2] = 255.0f / 255.0f;
        clearColor[0].color.float32[3] = 1.0f;
        //clearColor[1] is for the resolve attachment
        clearColor[2].depthStencil = {1.0f, 0};
        renderPassInfo.clearValueCount = 3;
        renderPassInfo.pClearValues = clearColor;

        vkCmdBeginRenderPass(b, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                          graphicsPipeline);

        VkBuffer buffers[] = {vertexBuffer, colorBuffer};
        VkDeviceSize offsets[] = {0,0};
        vkCmdBindVertexBuffers(b, 0, 2, buffers, offsets);
        vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipelineLayout, 0, 1, &descriptorSet, 0,
                                nullptr);

        vkCmdDraw(b, 27*36, 1, 0, 0);

        vkCmdEndRenderPass(b);

        VkResult vkRet = vkEndCommandBuffer(b);
        if (vkRet != VK_SUCCESS) {
            printf("vkEndCommandBuffer failed with %d\n", vkRet);
            return false;
        }
    }
    return true;
}

void VulkanApp::processCubeRot(double currentTime)
{
    const float t = (currentTime - rotStartTime) / cubeRotTime;
    if (t < 1.0f) {
        // We're still in the middle of the rotation, just continue slerping.
        cubeRot = MyQuaternion::slerp(cubeRotStart, cubeRotEnd, t);
        return;
    }

    // We're done with this one
    if (keyPress == -1) {
        // No more key presses
        cubeRot = cubeRotEnd;
        currentMoveKey = -1;
        return;
    }
    // Initiate another rotation
    startCubeRot(currentTime);
}

bool VulkanApp::renderFrame(uint32_t renderCount, double currentTime)
{
    // Draw
    const uint32_t idx = renderCount % swapChain.size();

    vkWaitForFences(device, 1, &swapChain[idx].fence, VK_TRUE, UINT64_MAX);
    vkResetFences(device, 1, &swapChain[idx].fence);

    if (currentMoveKey != -1) {
        // We're in the middle of a cube rot
        processCubeRot(currentTime);
        mvp.model = cubeRot.toMatrix();
        memcpy(&mvpUniformPtr->model, &mvp.model, sizeof(mvp.model));
    }

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
     || !createVertexBuffers()
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
    vkDestroyShaderModule(device, fragShader, nullptr);
    vkDestroyShaderModule(device, vertexShader, nullptr);
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
    vkDestroySwapchainKHR(device, vkSwapChain, nullptr);
}

void VulkanApp::cleanup()
{
    cleanupSwapChain();
    vkDestroyCommandPool(device, commandPool, nullptr);

    vkDestroyBuffer(device, vertexBuffer, nullptr);
    vkFreeMemory(device, vertexBufferMemory, nullptr);
    vkDestroyBuffer(device, colorBuffer, nullptr);
    vkFreeMemory(device, colorBufferMemory, nullptr);

    vkDestroyBuffer(device, cubeTransformsUniformBuffer, nullptr);
    vkUnmapMemory(device, cubeTransformsUniformBufferMemory);
    vkFreeMemory(device, cubeTransformsUniformBufferMemory, nullptr);

    vkDestroyBuffer(device, mvpUniformBuffer, nullptr);
    vkUnmapMemory(device, mvpUniformMemory);
    vkFreeMemory(device, mvpUniformMemory, nullptr);

    vkDestroyDescriptorPool(device, descriptorPool, nullptr);
    vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    glfwDestroyWindow(window);

    glfwTerminate();
}

void VulkanApp::onResize(int width, int height)
{
    if (width == 0 || height == 0)
        return;
    recreateSwapChain();
}

void VulkanApp::startCubeRot(double currentTime)
{
    if (keyPress == GLFW_KEY_SPACE) {
        resetMvp();
        currentMoveKey = -1;
        return ;
    }

    constexpr float angle = (M_PI/8.0f); // 22.5deg
    constexpr MyPoint xAxis(1.0f, 0.0f, 0.0f);
    constexpr MyPoint yAxis(0.0f, 1.0f, 0.0f);
    MyQuaternion newRot;
    switch (keyPress) {
      case GLFW_KEY_UP: newRot.setRotation(-angle, xAxis); break;
      case GLFW_KEY_DOWN: newRot.setRotation(angle, xAxis); break;
      case GLFW_KEY_LEFT: newRot.setRotation(-angle, yAxis); break;
      case GLFW_KEY_RIGHT: newRot.setRotation(angle, yAxis); break;
    }
    MyQuaternion newEnd = newRot * cubeRot;
    newEnd.normalize();

    if (keyPress != currentMoveKey) {
        // Different kind of rotation
        currentMoveKey = keyPress;
        cubeRotStart = cubeRot;
        rotStartTime = currentTime;
    }
    else {
        // Continueing the one we were doing.
        cubeRotStart = cubeRotEnd;
        rotStartTime += cubeRotTime;

        // We may be slightly past the end of the previous one so make sure
        // this is smooth.
        const float t = (currentTime - rotStartTime) / cubeRotTime;
        cubeRot = MyQuaternion::slerp(cubeRotEnd, newEnd, t);
    }
    cubeRotEnd = newEnd;
}

void VulkanApp::onKey(int key, int action)
{
#if 0
    if (GLFW_KEY_UP == key
     || GLFW_KEY_DOWN == key
     || GLFW_KEY_LEFT == key
     || GLFW_KEY_RIGHT == key
     || GLFW_KEY_SPACE == key) {
        if (action == GLFW_RELEASE) {
            keyPress = -1;
            return;
        }
        if (key == GLFW_KEY_SPACE && action != GLFW_PRESS) {
            return;
        }

        keyPress = key;
        if (currentMoveKey == -1) {
            startCubeRot(glfwGetTime());
        }
    }
#endif
}

int main(void)
{
    VulkanApp app;
    app.run();

    return 0;
}
