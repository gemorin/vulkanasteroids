#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <float.h>
#include <vector>
#include <random>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#include "stb_image.h"
#pragma clang diagnostic pop

#include "mymath.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

using namespace std;

static constexpr bool enableValidation = true;

//#define TEST_COLLISIONS 1

struct MyAABB2 {
    struct {
        float x;
        float y;
    } min, max;

    bool overlap(const MyAABB2& rhs) const
    {
        if (min.x > rhs.max.x
         || max.x < rhs.min.x
         || min.y > rhs.max.y
         || max.y < rhs.min.y)
            return false;
        return true;
    }
};

class VulkanApp
{
    random_device randomDevice;
    mt19937 randomGen{randomDevice()};

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
    float minY;

    VkSwapchainKHR vkSwapChain;
    struct SwapChainEntry {
        VkImage image;
        VkImageView view;

#ifdef MSAA
        // Anti aliasing stuff
        VkImage msaaImage;
        VkDeviceMemory msaaMemory;
        VkImageView msaaView;
#endif

        //VkImage depthImage;
        //VkDeviceMemory depthImageMemory;
        //VkImageView depthImageView;

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

        void cleanup(VkDevice device)
        {
            vkDestroyImageView(device, view, nullptr);
            vkFreeMemory(device, memory, nullptr);
            vkDestroyImage(device, image, nullptr);
        }
    };

    // Background
    VkDescriptorSetLayout backgroundDescriptorLayout;
    Descriptor backgroundDescriptor;
    VkShaderModule backgroundVertexShader;
    VkShaderModule backgroundFragmentShader;
    VertexBuffer backgroundVertex;
    Texture backgroundTexture;
    VkPipeline backgroundPipeline;
    VkPipelineLayout backgroundPipelineLayout;
    VkSampler backgroundSampler;

    // Sprites (ship, asteroids, bullets)
    VkDescriptorSetLayout spritesDescriptorLayout;
    Descriptor spritesDescriptor;
    VkShaderModule spriteVertexShader;
    VkShaderModule spritesFragmentShader;
    static constexpr uint32_t shipVertexIndex = 0;
    static constexpr uint32_t shipEngineVertexIndex = 1;
    static constexpr uint32_t shipBulletVertexIndex = 2;
    static constexpr uint32_t asteroidVertexIndexBig = 3;
    static constexpr uint32_t asteroidVertexIndexMiddle = 4;
    static constexpr uint32_t asteroidVertexIndexSmall = 5;
    VertexBuffer spriteVertex;
    VkPipeline spritesPipeline;
    VkPipelineLayout spritesPipelineLayout;
    VkSampler spritesSampler;

    struct ShipState {
        MyPoint position;
        MyPoint velocity;

        MyQuaternion orientation;

        double lastFrame;

        MyQuaternion rotStart;
        MyQuaternion rotEnd;
        double rotStartTime = 0.0f;
        bool inRotation = false;

        bool inHitAnim = false;
        double startHitAnim = 0.0;

        ShipState() { orientation.rotateZ(M_PI); }

        void initiateRotation(bool pos, double currentTime);
        void updateOrientation(double currentTime, GLFWwindow *window);
        void updatePosition(double currentTime, VulkanApp *app);
        void update(double currentTime, double dt, VulkanApp *app);
        const MyPoint& getPosition() const { return position; }
        MyMatrix getTransform() const;
    };
    ShipState shipState;
    float shipSize[2];

    struct AsteroidState {
        MyPoint position;
        MyPoint velocity;
        MyQuaternion orientation;
        float radius;
        float angularVelocity = M_PI/8.0f; // 22.5deg/sec over z axis
        MyMatrix inverseInertiaTensor;

        MyQuaternion rotStart;
        MyQuaternion rotEnd;
        double rotStartTime = 0.0;
        double birth;

        float mass = 5.0f;
        uint32_t textureIndex;
        uint8_t sizeType = 0;

        AsteroidState() = default;

        void generateTensor();

        MyMatrix getTransform() const;
        void update(double currentTime, double dt, const VulkanApp *app);
    };
    constexpr static uint32_t maxNumAsteroids = 10;
    constexpr static uint32_t vertexPushConstantsSize = MAX_SPRITES_PER_DRAW*
                                                        sizeof(MyMatrix);
    constexpr static uint32_t fragTexPushConstantSize =
                            MAX_SPRITES_PER_DRAW * (sizeof(int)+sizeof(float)) +
                            sizeof(float);
    struct SpriteTextures {
        static constexpr uint32_t shipTextureIndex = 0;
        static constexpr uint32_t shipEngineTextureIndex = 1;
        static constexpr uint32_t shipBulletTextureIndex = 2;
        static constexpr uint32_t asteroidTextureStartIndex = 3;
        static constexpr uint32_t asteroidTextureEndIndex = 6;

        vector<Texture> textures;
        // Move sizes in our coord here XXX FIXME

        SpriteTextures() { textures.resize(asteroidTextureEndIndex + 1); }
        static_assert((asteroidTextureEndIndex+1) == SPRITE_TEXTURE_ARRAY_SIZE,
                      "texture array is not sized properly");
    } sprites;

    struct BulletState {
        MyPoint position;
        MyQuaternion orientation;
        MyPoint velocity;
        bool live = false;

        void update(double dt, const VulkanApp *app);
        MyMatrix getTransform() const;
    } bulletState;
    float bulletSize[2];

    vector<AsteroidState> asteroidStates;
    float bigAsteroidSize[2];
    uniform_int_distribution<> asteroidSpawnRand{1, 3};
    double lastSpawnRandCheck = 0.0;
    uint32_t score = 0;
    uint32_t health = 100;

    struct Explosions {
        Texture atlas;
        VertexBuffer vertex;
        VkDescriptorSetLayout layout;
        Descriptor desc;

        VkSampler sampler;
        VkShaderModule vert;
        VkShaderModule frag;
        VkPipelineLayout pipelineLayout;
        VkPipeline pipeline;

        struct CurrentExplosions {
            MyPoint center;
            double start;

            CurrentExplosions(MyPoint c, double s)
                : center(c), start(s) {}
        };
        vector<CurrentExplosions> actives;
    } explosions;

    // textOverlay
    struct Overlay {
        Texture font;
        //stb_fontchar fontData[STB_NUM_CHARS];
        static constexpr int fontFirstChar = 32;
        stbtt_bakedchar fontData[126 - fontFirstChar];
        VkSampler fontSampler;
        VertexBuffer vertex;
        void *vertexData;
        size_t scoreStartIdx;
        VkDescriptorSetLayout layout;
        Descriptor desc;

        VkShaderModule vert;
        VkShaderModule frag;
        VkPipelineLayout pipelineLayout;
        VkPipeline pipeline;
    } textOverlay;
    struct {
        Texture health;
        VkSampler healthSampler;
        VertexBuffer vertex;
        void *vertexData;
        float top, bottom;
        float left;
        float fullWidth;
        VkDescriptorSetLayout layout;
        Descriptor desc;

        VkShaderModule vert;
        VkShaderModule frag;
        VkPipelineLayout pipelineLayout;
        VkPipeline pipeline;
    } hud;

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
    bool createPipelines();
    bool createFrameBuffers();
#if 0
    bool createDepthResources();
#endif
    bool createCommandPool();
    bool createCommandBuffers();
    bool setupCommandBuffers();
    bool resetCommandBuffer(uint32_t i, double currentTime);
    bool createVertexBuffers();
    bool createOverlayVertex();
    void createOverlayVertices(vector<Vertex>& vertices, float x, float y,
                               const string& s);
    bool createHUDVertex();
    bool createUniformBuffers();
    bool createDescriptors();
    bool createBackgroundDescriptor();
    bool createShipDescriptor();
    bool createExplosionDescriptor();
    bool createOverlayDescriptor();
    bool createHUDDescriptor();
    bool createTextures();
    bool setupDebugCallback();
    bool createShaders(VkShaderModule *vs, VkShaderModule *fs,
                       const char *vertexPath, const char *fragPath);
    bool createTexture(struct Texture *texture, const char *filename);
    //bool loadFont(struct Texture *texture, stb_fontchar *data);
    bool loadFont(struct Texture *texture,
                  stbtt_bakedchar *fontData,
                  int firstChar,
                  const char *file);

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

    void spawnNewAsteroid(double currentTime);
    static MyAABB2 getSphereAABB(float *, MyPoint position);
    void resolveAsteroidCollisions(AsteroidState& a, AsteroidState& b,
                                   bool goBackInTime, uint32_t i,
                                   uint32_t j);
    void checkForBulletHit();
    void getAsteroidSize(float *sz, const AsteroidState &a);
    void onBulletHit(AsteroidState &a, uint32_t aIdx);
    void checkForAsteroidHit(double currentTime);
    void processCollisions(AsteroidState& a, AsteroidState& b, uint32_t i,
                           uint32_t j);
    double getDeltaVelocity(const AsteroidState& a,
                            const MyPoint& relativeContactPos,
                            const MyPoint& normal);
    void changeScore(uint32_t add);
    void changeHealth(uint32_t sub);

    bool renderFrame(uint32_t renderCount, double currentTime,
                     double dt);

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

    float getMinX() const {
        return -0.5f;
    }
    float getMinY() const {
        return minY;
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

void VulkanApp::resolveAsteroidCollisions(AsteroidState& a, AsteroidState& b,
                                          bool goBackInTime,
                                          uint32_t i, uint32_t j)
{
    // XXX this does not handle when we wrapped around the screen
    MyPoint segment = b.position - a.position;
    const float len = segment.length();

    const float collisionDistance = a.radius + b.radius;
    if (len > collisionDistance) {
        // Nothing to do
        return;
    }
    const MyPoint velDiff = goBackInTime
                          ? a.velocity - b.velocity
                          : b.velocity - a.velocity;
    const float velDiffLen = velDiff.length();
#if 1
    printf("\ntime %lf goBackInTime %d a idx %u b idx %u\n", glfwGetTime(),
           goBackInTime,
           i, j);
    a.position.print("a pos ");
    a.velocity.print("a vel ");
    b.position.print("b pos ");
    b.velocity.print("b vel ");
#endif

    const float dot = velDiff.dot(segment);

    const float quadA = velDiffLen * velDiffLen;
    const float quadB = 2.0f * dot;
    const float quadC = len * len - collisionDistance * collisionDistance;
    const float sqrtDelta = sqrt(quadB * quadB - 4.0f * quadA * quadC);
#if 0
    printf("delta %f\n", sqrtDelta);
    printf("a %f\n", quadA);
    printf("b %f\n", quadB);
    printf("c %f\n", quadC);
#endif

    const float s1 = (-quadB + sqrtDelta) / (2.0f * quadA);
    const float s2 = (-quadB - sqrtDelta) / (2.0f * quadA);
    printf("s1 %f s2 %f\n", s1, s2);
    float s = s1 > s2 ? s1 : s2;
    if (goBackInTime)
        s *= -1.0f;

    a.position += (a.velocity) * s;
    b.position += (b.velocity) * s;
#if 1
    a.position.print("adj a pos ");
    b.position.print("adj b pos ");
    printf("a radius %.5f b radius %.5f\n", a.radius, b.radius);
    printf("distance %.5f\n", (b.position - a.position).length());
#endif
}

void VulkanApp::getAsteroidSize(float *sz, const AsteroidState &a)
{
    sz[0] = bigAsteroidSize[0];
    sz[1] = bigAsteroidSize[1];

    for (uint8_t i = 0; i < a.sizeType; ++i) {
        sz[0] /= 2.0f;
        sz[1] /= 2.0f;
    }
}

void VulkanApp::checkForBulletHit()
{
    MyPoint vertices[6];
    const uint32_t startIdx = 6 * sprites.shipBulletTextureIndex;
    for (uint32_t i = 0; i < 6; ++i) {
        const Vertex v = spriteVertex.vertices[startIdx + i];
        vertices[i] = v.pos.transform(bulletState.orientation);
        vertices[i] += bulletState.position;
    }

    MyAABB2 bullet;
    bullet.min = {FLT_MAX, FLT_MAX};
    bullet.max = { -FLT_MAX, -FLT_MAX};
    for (uint32_t i = 0; i < 6; ++i) {
        bullet.min.x = min(bullet.min.x, vertices[i].x);
        bullet.min.y = min(bullet.min.y, vertices[i].y);

        bullet.max.x = max(bullet.max.x, vertices[i].x);
        bullet.max.y = max(bullet.max.y, vertices[i].y);
    }
    for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
        float sz[2];
        getAsteroidSize(sz, asteroidStates[i]);
        MyAABB2 a = getSphereAABB(sz, asteroidStates[i].position);

        if (bullet.overlap(a)) {
            changeScore(5 * (1 + asteroidStates[i].sizeType));
            onBulletHit(asteroidStates[i], i);
            bulletState.live = false;
            break;
        }
    }
}

void VulkanApp::onBulletHit(AsteroidState &a, uint32_t aIdx)
{
    // Generate an explosion
    explosions.actives.emplace_back(a.position, 0.0);

    // Now either we split it on we get rid of it entirely
    if (a.sizeType == 2) {
        // Get rid of it
        asteroidStates.erase(asteroidStates.begin() + aIdx);
        return;
    }

    // Create 2 smaller asteroids
    ++a.sizeType;
    a.radius /= 2.0f;
    a.mass /= 2.0f;
    a.velocity = a.velocity.cross(MyPoint(0.0f,0.0f,1.0f));
    a.generateTensor();

    asteroidStates.emplace_back(a);
    asteroidStates.back().velocity *= -1.0f;

    // Adjust their positions
    resolveAsteroidCollisions(a, asteroidStates.back(), false,
                              aIdx, asteroidStates.size() - 1);
}

void VulkanApp::checkForAsteroidHit(double currentTime)
{
    MyPoint vertices[6];
    const uint32_t startIdx = 6 * shipVertexIndex;
    for (uint32_t i = 0; i < 6; ++i) {
        const Vertex v = spriteVertex.vertices[startIdx + i];
        vertices[i] = v.pos.transform(shipState.orientation);
        vertices[i] += shipState.position;
    }

    MyAABB2 ship;
    ship.min = {FLT_MAX, FLT_MAX};
    ship.max = { -FLT_MAX, -FLT_MAX};
    for (uint32_t i = 0; i < 6; ++i) {
        ship.min.x = min(ship.min.x, vertices[i].x);
        ship.min.y = min(ship.min.y, vertices[i].y);

        ship.max.x = max(ship.max.x, vertices[i].x);
        ship.max.y = max(ship.max.y, vertices[i].y);
    }
    for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
        float sz[2];
        getAsteroidSize(sz, asteroidStates[i]);
        MyAABB2 a = getSphereAABB(sz, asteroidStates[i].position);

        if (ship.overlap(a)) {
            shipState.inHitAnim = true;
            changeHealth(10 - 2 * asteroidStates[i].sizeType);
            shipState.startHitAnim = currentTime;
            explosions.actives.emplace_back(asteroidStates[i].position, 0.0);
            asteroidStates.erase(asteroidStates.begin() + i);
            break;
        }
    }
}

double VulkanApp::getDeltaVelocity(const AsteroidState& a,
                                   const MyPoint& relativeContactPos,
                                   const MyPoint& normal)
{
    MyPoint deltaVelocityWorld = relativeContactPos.cross(normal);
    // rotation per impulse
    deltaVelocityWorld = deltaVelocityWorld.transform(a.inverseInertiaTensor);
    // velocity per impulse
    deltaVelocityWorld = deltaVelocityWorld.cross(relativeContactPos);

    return deltaVelocityWorld.dot(normal) + 1.0f / a.mass;
}

void VulkanApp::processCollisions(AsteroidState& a, AsteroidState& b,
                                  uint32_t i, uint32_t j)
{
    const MyPoint segment = a.position - b.position;
    const float len = segment.length();
    const float collisionDistance = a.radius + b.radius;
    if (len > collisionDistance) {
        // Nothing to do
        return;
    }

    printf("time %lf contact a idx %u b idx %u\n", glfwGetTime(), i, j);

    // compute normal
    MyPoint normal = segment * 1.0f;
    normal.normalize();

    // Compute contact basis transform -- FIXME up might not work as a 2nd
    // vector
    MyPoint y,z;

    if (fabsf(normal.x) > fabsf(normal.y)) {
        MyPoint::makeOrthornormalBasis(&y, &z, normal,
                                       MyPoint(0.0f, 1.0f, 0.0f));
    }
    else {
        MyPoint::makeOrthornormalBasis(&y, &z, normal,
                                       MyPoint(1.0f, 0.0f, 0.0f));
    }
    MyMatrix worldToContactTransform;
    worldToContactTransform.set(0, 0, normal.x);
    worldToContactTransform.set(0, 1, normal.y);
    worldToContactTransform.set(0, 2, normal.z);
    worldToContactTransform.set(1, 0, y.x);
    worldToContactTransform.set(1, 1, y.y);
    worldToContactTransform.set(1, 2, y.z);
    worldToContactTransform.set(2, 0, z.x);
    worldToContactTransform.set(2, 1, z.y);
    worldToContactTransform.set(2, 2, z.z);

    // Contact positions
    const MyPoint aRelativeContactPos = segment * -0.5f;
    const MyPoint bRelativeContactPos = aRelativeContactPos * -1.0f;

    // Contact velocity
    MyPoint aContactVelocity = a.velocity +
               MyPoint(0.0f,0.0f, a.angularVelocity).cross(aRelativeContactPos);
    aContactVelocity = aContactVelocity.transform(worldToContactTransform);

    MyPoint bContactVelocity = b.velocity +
               MyPoint(0.0f,0.0f, b.angularVelocity).cross(bRelativeContactPos);
    bContactVelocity = bContactVelocity.transform(worldToContactTransform);

    MyPoint contactVelocity = aContactVelocity - bContactVelocity;
    constexpr float restitution = 2.0f;
    const double desiredDeltaVelocity = -contactVelocity.x * restitution;
    const double deltaVelocity =
                           getDeltaVelocity(a, aRelativeContactPos, normal) +
                           getDeltaVelocity(b, bRelativeContactPos, normal);

    // We're doing frictionless collision, everything happens in the normal
    MyPoint impulseContact = MyPoint(desiredDeltaVelocity/deltaVelocity,
                                     0.0f, 0.0f);
    // Transform back into world coordinates
    MyPoint impulse = impulseContact.transform(
                                          worldToContactTransform.transpose());

    // Split in the impulse into linear and rotational components
    MyPoint impulsiveTorque = aRelativeContactPos.cross(impulse);
    MyPoint rotChange = impulsiveTorque.transform(a.inverseInertiaTensor);
    MyPoint velChange = impulse * (1.0f/a.mass);
    velChange.print("velChange a ");
    rotChange.print("rotChange a ");
    a.velocity += velChange;
    a.velocity.print("a vel");
    a.angularVelocity += rotChange.z;

    impulsiveTorque = impulse.cross(bRelativeContactPos.cross(impulse));
    rotChange = impulsiveTorque.transform(b.inverseInertiaTensor);
    velChange = impulse * (-1.0f/b.mass);
    velChange.print("velChange b ");
    rotChange.print("rotChange b ");
    puts("");

    b.velocity += velChange;
    b.velocity.print("a vel");
    b.angularVelocity += rotChange.z;
}

void VulkanApp::ShipState::initiateRotation(bool pos, double currentTime)
{
    const float delta = M_PI/18.0f;

    rotStart = orientation;
    MyQuaternion deltaQ;
    deltaQ.rotateZ(pos ? delta : -delta);
    rotEnd = deltaQ * rotStart;
    rotEnd.normalize();
    rotStartTime = currentTime;
    inRotation = true;
}

void
VulkanApp::ShipState::updateOrientation(double currentTime, GLFWwindow *window)
{
    if (inRotation) {
        constexpr float totTime = 0.06f;
        const float t = (currentTime - rotStartTime) / totTime;

        orientation = MyQuaternion::slerp(rotStart, rotEnd, t);
        if (t < 1.0f) {
            return;
        }
        inRotation = false;
        return;
    }
    if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_LEFT)
     || GLFW_PRESS == glfwGetKey(window, GLFW_KEY_RIGHT)) {
        bool pos = (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
        initiateRotation(pos, currentTime);
    }
}

void
VulkanApp::ShipState::updatePosition(double dt, VulkanApp *app)
{
    if (velocity.length() < 0.5f &&
        GLFW_PRESS == glfwGetKey(app->window, GLFW_KEY_UP)) {
        const float acceleration = 0.5f;
        // ship model points down
        MyPoint direction(0.0f, -1.0f, 0.0f);
        velocity += direction.transform(orientation) * (acceleration * dt);
    }
    position += velocity * dt;
    velocity *= 0.999f;

    if (position.x > -app->getMinX()) {
        position.x = app->getMinX();
    }
    else if (position.x < app->getMinX()) {
        position.x = -app->getMinX();
    }
    if (position.y > -app->getMinY()) {
        position.y = app->getMinY();
    }
    else if (position.y < app->getMinY()) {
        position.y = -app->getMinY();
    }
}

void VulkanApp::ShipState::update(double currentTime, double dt, VulkanApp *app)
{
    updateOrientation(currentTime, app->window);
    updatePosition(dt, app);

    lastFrame = currentTime;
}

MyMatrix VulkanApp::ShipState::getTransform() const
{
    MyMatrix ret = orientation.toMatrix();
    ret.set(0, 3, position.x);
    ret.set(1, 3, position.y);
    ret.set(2, 3, position.z);
    return ret;
}

MyAABB2 VulkanApp::getSphereAABB(float *sizes, MyPoint position)
{
    MyAABB2 a;
    a.min.x = position.x - sizes[0] / 2.0f;
    a.max.x = position.x + sizes[0] / 2.0f;
    a.min.y = position.y - sizes[1] / 2.0f;
    a.max.y = position.y + sizes[1] / 2.0f;
    return a;
}

void VulkanApp::AsteroidState::update(double currentTime,
                                      double dt,
                                      const VulkanApp *app)
{
    float t = currentTime - rotStartTime;
    if (t >= 1.0f) {
        rotStart = rotEnd;
        rotStartTime += 1.0;

        MyQuaternion newRot;
        newRot.rotateZ(angularVelocity);
        rotEnd = newRot * rotStart;
        rotEnd.normalize();

        // We may be slightly past the end of the previous one so make sure
        // this is smooth.
        t -= 1.0f;
    }
    orientation = MyQuaternion::slerp(rotStart, rotEnd, t);

    position += (velocity * dt);

    if (position.x - radius > -app->getMinX()) {
        position.x = app->getMinX() - radius;
    }
    else if (position.x + radius < app->getMinX()) {
        position.x = -app->getMinX() + radius;
    }
    if (position.y - radius > -app->getMinY()) {
        position.y = app->getMinY() - radius;
    }
    else if (position.y + radius < app->getMinY()) {
        position.y = -app->getMinY() + radius;
    }
}

MyMatrix VulkanApp::AsteroidState::getTransform() const
{
    MyMatrix ret = orientation.toMatrix();
    ret.set(0, 3, position.x);
    ret.set(1, 3, position.y);
    ret.set(2, 3, position.z);
    return ret;
}

void VulkanApp::AsteroidState::generateTensor()
{
    const float value = 2.5f / (mass * radius * radius);
    inverseInertiaTensor.set(0, 0, value);
    inverseInertiaTensor.set(1, 1, value);
    inverseInertiaTensor.set(2, 2, value);
}

void VulkanApp::BulletState::update(double dt, const VulkanApp *app)
{
    position += (velocity * dt);

    if (position.x > -app->getMinX()
     || position.x < app->getMinX()
     || position.y > -app->getMinY()
     || position.y < app->getMinY()) {
        live = false;
    }
}

MyMatrix VulkanApp::BulletState::getTransform() const
{
    MyMatrix ret = orientation.toMatrix();
    ret.set(0, 3, position.x);
    ret.set(1, 3, position.y);
    ret.set(2, 3, position.z);
    return ret;
}

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
    //result.print();
    //puts("");

    MyMatrix correct;
    correct.set(1, 1, -1.0f);
    correct.set(2, 2, 0.5f);
    correct.set(2, 3, 0.5f);

    result = correct * result;
    //result/print();

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
     || !createPipelines()
     || !createCommandPool()
     || !createCommandBuffers()
     //|| !createDepthResources()
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
    double prevFps = 0.0;
    bool updateStart = true;
    double start;
    double prevTime = 0.0;
    while(1) {
        const double currentTime = glfwGetTime();
        if (updateStart) {
            start = currentTime;
            updateStart = false;
        }
        bool running = renderFrame(renderCount++, currentTime,
                                   currentTime - prevTime);
        prevTime = currentTime;

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_RELEASE)
            running = false;
        if (glfwWindowShouldClose(window))
            running = false;
        if (!running) {
            break;
        }
        if (0 == (renderCount % 100)) {
            double fps = 100.0 / (currentTime - start);
            double diff = fps / prevFps;
            if (diff > 1.1 || diff < 0.9) {
                printf("fps %.0lf\n", fps);
            }
            prevFps = fps;
            updateStart = true;
        }
    }

    waitForIdle();
    cleanup();
}

void VulkanApp::waitForIdle()
{
    //for (auto & swpe : swapChain)
    //    vkWaitForFences(device, 1, &swpe.fence, VK_TRUE, UINT64_MAX);
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
    window = glfwCreateWindow(800, 600, "VulkanAsteroids", nullptr, nullptr);
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowSizeCallback(window, &VulkanApp::glfw_onResize);
    // glfwSetKeyCallback(window, &VulkanApp::glfw_onKey);

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
    appInfo.pApplicationName = "Vulkasteroids";
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
          || 0 == strcmp("VK_LAYER_LUNARG_parameter_validation", layerName)
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
    const float aspect = (float) height / (float) width;
    minY = -aspect / 2.0f;
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

#ifdef MSAA
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
#endif
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
                       "vertex_background.spv", "fragment_background.spv")
     || !createShaders(&spriteVertexShader, &spritesFragmentShader,
                       "vertex_sprite.spv", "fragment_sprite.spv")
     || !createShaders(&explosions.vert, &explosions.frag,
                       "vertex_explosions.spv", "fragment_explosions.spv")
     || !createShaders(&textOverlay.vert, &textOverlay.frag,
                       "vertex_overlay.spv", "fragment_overlay.spv")
     || !createShaders(&hud.vert, &hud.frag,
                       "vertex_hud.spv", "fragment_hud.spv")) {
        return false;
    }
    return true;
}

bool VulkanApp::createRenderPass()
{
    // MSAA attachment
    // from https://arm-software.github.io/vulkan-sdk/multisampling.html
    VkAttachmentDescription attachments[2];
    memset(&attachments, 0, sizeof(attachments));
    attachments[0].format = devInfo.format.format;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
#ifdef MSAA
    attachments[0].samples = VK_SAMPLE_COUNT_4_BIT;
    // Don't write to memory, we just want to compute
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
#else
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
#endif
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
#ifdef MSAA
    // this does not go to the presentation
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
#else
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
#endif

#ifdef MSAA
    attachments[1].format = devInfo.format.format;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT; // required for resolve
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
#endif

#if 0
    attachments[2].format = findDepthFormat();
    attachments[2].samples = VK_SAMPLE_COUNT_4_BIT;
    attachments[2].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[2].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[2].finalLayout =
                              VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
#endif

    // The color attachment is referenced by
    // 'layout (location = 0) out vec4 outColor' in the frag shader
    VkAttachmentReference colorRef = {};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

#ifdef MSAA
    VkAttachmentReference resolveRef = {};
    resolveRef.attachment = 1;
    resolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
#endif

#if 0
    VkAttachmentReference depthRef = {};
    depthRef.attachment = 2;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
#endif


    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
#ifdef MSAA
    subpass.pResolveAttachments = &resolveRef;
#else
    subpass.pResolveAttachments = nullptr;
#endif
    subpass.pDepthStencilAttachment = nullptr;

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
#ifdef MSAA
    renderPassInfo.attachmentCount = 2;
#else
    renderPassInfo.attachmentCount = 1;
#endif
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
    VkDescriptorSetLayoutBinding layout[3];
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
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = layout;

    VkResult vkRet;
    vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                        &backgroundDescriptorLayout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }

    layout[1].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
    layout[2].binding = 2;
    layout[2].descriptorCount = sprites.textures.size();
    layout[2].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    layout[2].pImmutableSamplers = nullptr;
    layout[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = layout;

    vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                        &spritesDescriptorLayout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }

    layout[1].binding = 1;
    layout[1].descriptorCount = 1;
    layout[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layout[1].pImmutableSamplers = nullptr;
    layout[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = layout;

    vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                        &explosions.layout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }

    // Text overlay
    layout[0].binding = 0;
    layout[0].descriptorCount = 1;
    layout[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layout[0].pImmutableSamplers = nullptr;
    layout[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = layout;

    vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                        &textOverlay.layout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }

    // HUD
    vkRet = vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                        &hud.layout);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorSetLayout() failed with %d", vkRet);
        return false;
    }

    return true;
}

bool VulkanApp::createPipelines()
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
    // Point input: vertices, colors, uv
    VkVertexInputBindingDescription bindingDescriptions[1];
    memset(bindingDescriptions, 0, sizeof(bindingDescriptions));
    bindingDescriptions[0].binding = 0;
    bindingDescriptions[0].stride = sizeof(Vertex);
    bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

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
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f;
    rasterizer.depthBiasClamp = 0.0f;
    rasterizer.depthBiasSlopeFactor = 0.0f;

    // MSAA
    VkPipelineMultisampleStateCreateInfo multisampling = {};
    multisampling.sType =
                      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
#ifdef MSAA
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_4_BIT;
#else
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
#endif
    multisampling.minSampleShading = 1.0f;
    multisampling.pSampleMask = nullptr;
    multisampling.alphaToCoverageEnable = VK_FALSE;
    multisampling.alphaToOneEnable = VK_FALSE;

    // Stencil/depth buffer
#if 0
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
#endif

    // Color blending
    VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
                                          VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT |
                                          VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    //colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    //colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;

    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
                                          VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT |
                                          VK_COLOR_COMPONENT_A_BIT;

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
    colorBlending.blendConstants[3] = 1.0f;

    // vulkan dynamic states
    // VkPipelineDynamicStateCreateInfo dynamicState = {};

    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType =
                            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &backgroundDescriptorLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 0;
    pipelineLayoutInfo.pPushConstantRanges = 0;

    VkResult vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo,
                                            nullptr, &backgroundPipelineLayout);
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
    pipelineInfo.pDepthStencilState = nullptr; //&depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = nullptr;
    pipelineInfo.layout = backgroundPipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;
    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
    pipelineInfo.basePipelineIndex = -1;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &backgroundPipeline);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreateGraphicsPipelines failed with ret %d\n", vkRet);
       return false;
    }

    // Ship pipeline
    VkPushConstantRange pushConstantsRanges[2];
    memset(&pushConstantsRanges, 0, sizeof(pushConstantsRanges));
    pushConstantsRanges[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstantsRanges[0].offset = 0;
    pushConstantsRanges[0].size = vertexPushConstantsSize;
    pushConstantsRanges[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantsRanges[1].offset = vertexPushConstantsSize;
    pushConstantsRanges[1].size = fragTexPushConstantSize;

    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &spritesDescriptorLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 2;
    pipelineLayoutInfo.pPushConstantRanges = pushConstantsRanges;

    vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr,
                                   &spritesPipelineLayout);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreatePipelineLayout failed with ret %d\n", vkRet);
       return false;
    }
    pipelineInfo.layout = spritesPipelineLayout;

    // Change shaders
    vertShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = spriteVertexShader;
    vertShaderStageInfo.pName = "main";

    fragShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = spritesFragmentShader;
    fragShaderStageInfo.pName = "main";
    shaderStages[0] = vertShaderStageInfo;
    shaderStages[1] = fragShaderStageInfo;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &spritesPipeline);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreateGraphicsPipelines failed with ret %d\n", vkRet);
       return false;
    }

    // particles pipeline
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;

    memset(&pushConstantsRanges, 0, sizeof(pushConstantsRanges));
    pushConstantsRanges[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstantsRanges[0].offset = 0;
    pushConstantsRanges[0].size = sizeof(MyMatrix);
    pushConstantsRanges[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantsRanges[1].offset = sizeof(MyMatrix);
    pushConstantsRanges[1].size = sizeof(int);

    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &explosions.layout;
    pipelineLayoutInfo.pushConstantRangeCount = 2;
    pipelineLayoutInfo.pPushConstantRanges = pushConstantsRanges;

    vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr,
                                   &explosions.pipelineLayout);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreatePipelineLayout failed with ret %d\n", vkRet);
       return false;
    }
    pipelineInfo.layout = explosions.pipelineLayout;

    // Change shaders
    vertShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = explosions.vert;
    vertShaderStageInfo.pName = "main";

    fragShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = explosions.frag;
    fragShaderStageInfo.pName = "main";
    shaderStages[0] = vertShaderStageInfo;
    shaderStages[1] = fragShaderStageInfo;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &explosions.pipeline);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreateGraphicsPipelines failed with ret %d\n", vkRet);
       return false;
    }

    // text overlay
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &textOverlay.layout;
    pipelineLayoutInfo.pushConstantRangeCount = 0;
    pipelineLayoutInfo.pPushConstantRanges = nullptr;

    vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr,
                                   &textOverlay.pipelineLayout);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreatePipelineLayout failed with ret %d\n", vkRet);
       return false;
    }
    pipelineInfo.layout = textOverlay.pipelineLayout;

    // Change shaders
    vertShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = textOverlay.vert;
    vertShaderStageInfo.pName = "main";

    fragShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = textOverlay.frag;
    fragShaderStageInfo.pName = "main";
    shaderStages[0] = vertShaderStageInfo;
    shaderStages[1] = fragShaderStageInfo;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &textOverlay.pipeline);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreateGraphicsPipelines failed with ret %d\n", vkRet);
       return false;
    }

    // HUD
    pushConstantsRanges[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantsRanges[0].offset = 0;
    pushConstantsRanges[0].size = sizeof(int);
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &hud.layout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = pushConstantsRanges;

    vkRet = vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr,
                                   &hud.pipelineLayout);
    if (vkRet != VK_SUCCESS) {
       printf("vkCreatePipelineLayout failed with ret %d\n", vkRet);
       return false;
    }
    pipelineInfo.layout = hud.pipelineLayout;

    // Change shaders
    vertShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = hud.vert;
    vertShaderStageInfo.pName = "main";

    fragShaderStageInfo.sType =
                           VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = hud.frag;
    fragShaderStageInfo.pName = "main";
    shaderStages[0] = vertShaderStageInfo;
    shaderStages[1] = fragShaderStageInfo;

    vkRet = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo,
                                      nullptr, &hud.pipeline);
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
#ifdef MSAA
        VkImageView attachments[] = { swapChain[i].msaaView,
                                      swapChain[i].view,
                                    };
#else
        VkImageView attachments[] = { swapChain[i].view, };
#endif
        VkFramebufferCreateInfo framebufferInfo = {};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = sizeof(attachments)/
                                          sizeof(*attachments);
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

#if 0
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
#endif

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
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

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
    //commandBuffersDirty.resize(swapChain.size(), false);
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
    float bottom = getMinY();
    float top = bottom +
                      float(backgroundTexture.height) /
                      float(backgroundTexture.width);
    constexpr float z = 0.0f;

    // First triangle of background with image
    constexpr MyPoint red(1.0f, 0.0f, 0.0f);
    backgroundVertex.vertices.resize(6);
    auto* v = &backgroundVertex.vertices.front();
    *(v++) = {MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ 0.5f, bottom, z}, red, 1.0f, 1.0f};
    *(v++) = {MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f};
    // 2nd
    *(v++) = {MyPoint{-0.5f, bottom, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ 0.5f,    top, z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{-0.5f,    top, z}, red, 0.0f, 0.0f};

    uint32_t numBytes = backgroundVertex.vertices.size() *
                        sizeof(backgroundVertex.vertices.front());
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
    memcpy(data, backgroundVertex.vertices.data(), numBytes);
    vkUnmapMemory(device, backgroundVertex.memory);

    // Aspect scale
    const float xscale = -2.0f * getMinX() / float(devInfo.extent.width);
    const float yscale = -2.0f * getMinY() / float(devInfo.extent.height);

    // Assuming all sprites are 2 triangles
    spriteVertex.vertices.resize(6 * (asteroidVertexIndexSmall + 1));

    // ship
    float spriteScale = 0.8f;
    const Texture *t = &sprites.textures[sprites.shipTextureIndex];
    v = &spriteVertex.vertices[6 * shipVertexIndex];

    float x = float(t->width) * xscale / 2.0f * spriteScale;
    float y = yscale * float(t->height) / 2.0f * spriteScale;
    *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x, -y, z}, red, 1.0f, 1.0f};
    *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
    // 2nd
    *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{-x,  y, z}, red, 0.0f, 0.0f};
    shipSize[0] = 2 * x;
    shipSize[1] = 2 * y;

    // engine effect
    t = &sprites.textures[sprites.shipEngineTextureIndex];
    v = &spriteVertex.vertices[6 * shipEngineVertexIndex];
    spriteScale = 3.0f;
    x = xscale * float(t->width) / 2.0f * spriteScale;
    float ysize = yscale * float(t->height) * spriteScale;
    float ybot = y;
    float ytop = ybot + ysize;

    *(v++) = {MyPoint{-x, ybot, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x, ybot, z}, red, 1.0f, 1.0f};
    *(v++) = {MyPoint{ x, ytop, z}, red, 1.0f, 0.0f};
    // 2nd
    *(v++) = {MyPoint{-x, ybot, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x, ytop, z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{-x, ytop, z}, red, 0.0f, 0.0f};

    // bullet
    t = &sprites.textures[sprites.shipBulletTextureIndex];
    v = &spriteVertex.vertices[6 * shipBulletVertexIndex];
    spriteScale = 1.0f;
    x = float(t->width) * xscale / 2.0f * spriteScale;
    y = yscale * float(t->height) / 2.0f * spriteScale;
    *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x, -y, z}, red, 1.0f, 1.0f};
    *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
    // 2nd
    *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{-x,  y, z}, red, 0.0f, 0.0f};
    // XXX FIXME sprite size?
    bulletSize[0] = 2 * x;
    bulletSize[1] = 2 * y;


    // asteroids - we have multiple textures - for now to simplify we'll use
    // the same vertices
    spriteScale = 0.25f;
    t = &sprites.textures[sprites.asteroidTextureStartIndex];
    v = &spriteVertex.vertices[6 * asteroidVertexIndexBig];
    for (int i = 0; i < 3; ++i) {
        x = float(t->width) * xscale / 2.0f * spriteScale;
        y = yscale * float(t->height) / 2.0f * spriteScale;
        *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{ x, -y, z}, red, 1.0f, 1.0f};
        *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
        // 2nd
        *(v++) = {MyPoint{-x, -y, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{ x,  y, z}, red, 1.0f, 0.0f};
        *(v++) = {MyPoint{-x,  y, z}, red, 0.0f, 0.0f};
        if (i == 0) {
            bigAsteroidSize[0] = 2 * x;
            bigAsteroidSize[1] = 2 * y;
        }
        spriteScale /= 2.0;
    }

    numBytes = spriteVertex.vertices.size() *
               sizeof(spriteVertex.vertices.front());
    if (!createBuffer(&spriteVertex.buffer, &spriteVertex.memory,
                      numBytes, vertexUsage, memFlags, false)) {
        return false;
    }
    vkMapMemory(device, spriteVertex.memory, 0, numBytes, 0, &data);
    memcpy(data, spriteVertex.vertices.data(), numBytes);
    vkUnmapMemory(device, spriteVertex.memory);

    constexpr uint32_t numParticles = 16;
#if 0
    constexpr uint32_t rowSize = sqrt(numParticles);
    explosions.vertex.vertices.resize(numParticles*6);
    v = &explosions.vertex.vertices.front();
    float radius = bigAsteroidSize[0] / 4;
    for (int i = 0; i < numParticles; ++i) {
        const float particleSz = radius / numParticles / 2;

        const int row = i / rowSize;
        const int col = i % rowSize;

        const float left = sz * (col - 2);
        const float right = left + sz;
        const float top = sz * (2 - row);
        const float bottom = top - sz;

        *(v++) = {MyPoint{left,  bottom, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{right, bottom, z}, red, 1.0f, 1.0f};
        *(v++) = {MyPoint{right,    top, z}, red, 1.0f, 0.0f};
        // 2nd triangle
        *(v++) = {MyPoint{left,  bottom, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{right,    top, z}, red, 1.0f, 0.0f};
        *(v++) = {MyPoint{left,     top, z}, red, 0.0f, 0.0f};
    }
#else
    float partRadius = bigAsteroidSize[0] / 2;
    float partSize = bigAsteroidSize[0] / 2;
    uniform_real_distribution<float> coordGen(-partRadius,
                                              partRadius - partSize);
    explosions.vertex.vertices.resize(numParticles*6);
    v = &explosions.vertex.vertices.front();
    for (uint32_t i = 0; i < numParticles; ++i) {
        const float left = coordGen(randomGen);
        const float right = left + partSize;
        const float bottom = coordGen(randomGen);
        const float top = bottom + partSize;

        *(v++) = {MyPoint{left,  bottom, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{right, bottom, z}, red, 1.0f, 1.0f};
        *(v++) = {MyPoint{right,    top, z}, red, 1.0f, 0.0f};
        // 2nd triangle
        *(v++) = {MyPoint{left,  bottom, z}, red, 0.0f, 1.0f};
        *(v++) = {MyPoint{right,    top, z}, red, 1.0f, 0.0f};
        *(v++) = {MyPoint{left,     top, z}, red, 0.0f, 0.0f};
    }
#endif
    numBytes = explosions.vertex.vertices.size() *
               sizeof(explosions.vertex.vertices.front());
    if (!createBuffer(&explosions.vertex.buffer, &explosions.vertex.memory,
                      numBytes, vertexUsage, memFlags, false)) {
        return false;
    }
    vkMapMemory(device, explosions.vertex.memory, 0, numBytes, 0, &data);
    memcpy(data, explosions.vertex.vertices.data(), numBytes);
    vkUnmapMemory(device, explosions.vertex.memory);

    if (!createOverlayVertex()) {
        return false;
    }

    if (!createHUDVertex()) {
        return false;
    }

    return true;
}

void VulkanApp::createOverlayVertices(vector<Vertex>& vertices,
                                      float x, float y,
                                      const string& s)
{
    const float pixelWidth = 2.0f / float(devInfo.extent.width);
    const float pixelHeight = 2.0f / float(devInfo.extent.height);
    const Texture *t = &textOverlay.font;
    constexpr MyPoint red(1.0f, 0.0f, 0.0f);
    for (char c : s) {
        auto *data = &textOverlay.fontData[c - textOverlay.fontFirstChar];
        float x0 = data->xoff;
        float y0 = data->yoff;
        float x1 = data->xoff + data->x1 - data->x0;
        float y1 = data->yoff + data->y1 - data->y0;
        MyPoint pos;
        float u,v;
        pos.x = x + (float) x0 * pixelWidth;
        pos.y = y + (float) y0 * pixelHeight;
        u = (float) data->x0 / t->width;
        v = (float) data->y0 / t->height;
        vertices.emplace_back(pos, red, u, v);

        pos.x = x + (float) x1 * pixelWidth;
        pos.y = y + (float) y0 * pixelHeight;
        u = (float) data->x1 / t->width;
        v = (float) data->y0 / t->height;
        vertices.emplace_back(pos, red, u, v);

        pos.x = x + (float) x1 * pixelWidth;
        pos.y = y + (float) y1 * pixelHeight;
        u = (float) data->x1 / t->width;
        v = (float) data->y1 / t->height;
        vertices.emplace_back(pos, red, u, v);

        pos.x = x + (float) x0 * pixelWidth;
        pos.y = y + (float) y0 * pixelHeight;
        u = (float) data->x0 / t->width;
        v = (float) data->y0 / t->height;
        vertices.emplace_back(pos, red, u, v);

        pos.x = x + (float) x1 * pixelWidth;
        pos.y = y + (float) y1 * pixelHeight;
        u = (float) data->x1 / t->width;
        v = (float) data->y1 / t->height;
        vertices.emplace_back(pos, red, u, v);

        pos.x = x + (float) x0 * pixelWidth;
        pos.y = y + (float) y1 * pixelHeight;
        u = (float) data->x0 / t->width;
        v = (float) data->y1 / t->height;
        vertices.emplace_back(pos, red, u, v);

        x += data->xadvance * pixelWidth;
    }
}

void VulkanApp::changeScore(uint32_t add)
{
    score += add;
    char buf[32];
    sprintf(buf, "Score %6u", score);
    textOverlay.vertex.vertices.erase(
            textOverlay.vertex.vertices.begin() + textOverlay.scoreStartIdx,
            textOverlay.vertex.vertices.end());
    createOverlayVertices(textOverlay.vertex.vertices, -0.95f, -0.95f, buf);
    uint32_t numBytes = (textOverlay.vertex.vertices.size () -
                         textOverlay.scoreStartIdx) * sizeof(Vertex);
    char *dest = (char *) textOverlay.vertexData;
    dest += textOverlay.scoreStartIdx * sizeof(Vertex);
    memcpy(dest, textOverlay.vertex.vertices.data() + textOverlay.scoreStartIdx,
           numBytes);
}



bool VulkanApp::createOverlayVertex()
{
    const float pixelWidth = 2.0f / float(devInfo.extent.width);
    float totalWidth = 0.0f;
    // Health
    string s = "HEALTH";
    for (char c : s) {
        auto *data = &textOverlay.fontData[c - textOverlay.fontFirstChar];
        totalWidth += data->xadvance * pixelWidth;
        //maxY = max(maxY, (float) data->y1 * pixelHeight);
    }
    float x = totalWidth / -2.0f;
    float y = -0.95f;
    createOverlayVertices(textOverlay.vertex.vertices, x, y, s);
    textOverlay.scoreStartIdx = textOverlay.vertex.vertices.size();

    // Score
    char buf[32];
    sprintf(buf, "Score %6u", score);
    x = -0.95f;
    y = -0.95f;
    createOverlayVertices(textOverlay.vertex.vertices, x, y, buf);

    uint32_t numBytes = textOverlay.vertex.vertices.size() *
                        sizeof(textOverlay.vertex.vertices.front());
    const VkBufferUsageFlags vertexUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    if (!createBuffer(&textOverlay.vertex.buffer, &textOverlay.vertex.memory,
                      numBytes, vertexUsage, memFlags, false)) {
        return false;
    }
    vkMapMemory(device, textOverlay.vertex.memory, 0, numBytes, 0,
                &textOverlay.vertexData);
    memcpy(textOverlay.vertexData, textOverlay.vertex.vertices.data(),
           numBytes);

    // Unlike other vertices, we do not unmap it so we can update it

    return true;
}

void VulkanApp::changeHealth(uint32_t sub)
{
    if (health > sub)
        health -= sub;
    else
        health = 0;

    //hud.vertex.vertices.resize(6);
    float vv = float(health) / 100.f;
    constexpr float z = 0.0f;
    constexpr MyPoint red(1.0f, 0.0f, 0.0f);

    hud.vertex.vertices.resize(12);
    auto *v = &hud.vertex.vertices.front();
    float right = hud.left + hud.fullWidth * vv;
    float end = hud.left + hud.fullWidth;
    *(v++) = {MyPoint{hud.left, hud.bottom, z}, red, 0.0f, vv};
    *(v++) = {MyPoint{right,    hud.bottom, z}, red, 1.0f, vv};
    *(v++) = {MyPoint{right,    hud.top, z}, red, 1.0f, 0.0f};
    // 2nd triangle
    *(v++) = {MyPoint{hud.left, hud.bottom, z}, red, 0.0f, vv};
    *(v++) = {MyPoint{right,    hud.top,    z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{hud.left, hud.top,    z}, red, 0.0f, 0.0f};

    constexpr MyPoint gray(0.2f, 0.2f, 0.2f);
    *(v++) = {MyPoint{right, hud.bottom, z}, gray, 0.0f, vv};
    *(v++) = {MyPoint{end,    hud.bottom, z}, gray, 1.0f, vv};
    *(v++) = {MyPoint{end,  hud.top, z}, gray, 1.0f, 0.0f};
    // 2nd triangle
    *(v++) = {MyPoint{right, hud.bottom, z}, gray, 0.0f, vv};
    *(v++) = {MyPoint{end,    hud.top,    z}, gray, 1.0f, 0.0f};
    *(v++) = {MyPoint{right, hud.top,    z}, gray, 0.0f, 0.0f};
    uint32_t numBytes = hud.vertex.vertices.size() *
                        sizeof(hud.vertex.vertices.front());
    memcpy(hud.vertexData, hud.vertex.vertices.data(), numBytes);
}

bool VulkanApp::createHUDVertex()
{
    const float xscale = -2.0f * getMinX() / float(devInfo.extent.width);
    const float yscale = -2.0f * getMinY() / float(devInfo.extent.height);
    const Texture *t = &hud.health;
    const float spriteScale = 3.0f;
    hud.left = float(t->width) / -2.0f * xscale * spriteScale;
    float right = -hud.left;
    hud.fullWidth = right - hud.left;
    float height = yscale * float(t->height) * spriteScale;
    hud.top = -0.930f;
    hud.bottom = hud.top + height;
    const float z = 0.0f;
    constexpr MyPoint red(1.0f, 0.0f, 0.0f);

    hud.vertex.vertices.resize(6);
    auto *v = &hud.vertex.vertices.front();
    *(v++) = {MyPoint{hud.left, hud.bottom, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{right,    hud.bottom, z}, red, 1.0f, 1.0f};
    *(v++) = {MyPoint{right,    hud.top, z}, red, 1.0f, 0.0f};
    // 2nd triangle
    *(v++) = {MyPoint{hud.left, hud.bottom, z}, red, 0.0f, 1.0f};
    *(v++) = {MyPoint{right,    hud.top,    z}, red, 1.0f, 0.0f};
    *(v++) = {MyPoint{hud.left, hud.top,    z}, red, 0.0f, 0.0f};

    uint32_t numBytes = hud.vertex.vertices.size() *
                        sizeof(hud.vertex.vertices.front());
    const VkBufferUsageFlags vertexUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    const VkMemoryPropertyFlags memFlags =
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    if (!createBuffer(&hud.vertex.buffer, &hud.vertex.memory,
                      numBytes, vertexUsage, memFlags, false)) {
        return false;
    }
    vkMapMemory(device, hud.vertex.memory, 0, numBytes, 0, &hud.vertexData);
    memcpy(hud.vertexData, hud.vertex.vertices.data(), numBytes);

    // Don't unmap

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
    vp.proj = ortho(getMinY(), -getMinY(), getMinX(), -getMinX(), -1.0f, 1.0f);
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
    poolSizes[0].descriptorCount = 1;
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

    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = backgroundDescriptor.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &backgroundDescriptorLayout;

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
    imageInfo.imageView = backgroundTexture.view;
    imageInfo.sampler = backgroundSampler;

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

bool VulkanApp::createHUDDescriptor()
{
    VkDescriptorPoolSize poolSizes[1];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = 2;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &hud.desc.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = hud.desc.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &hud.layout;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo, &hud.desc.set);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateDescriptorSets failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorImageInfo imageInfo[1];
    memset(imageInfo, 0, sizeof(imageInfo));
    imageInfo[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo[0].imageView = hud.health.view;
    imageInfo[0].sampler = hud.healthSampler;

    VkWriteDescriptorSet descriptorWrite[1];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = hud.desc.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType =
                                     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pImageInfo = imageInfo;

    vkUpdateDescriptorSets(device,
                           sizeof(descriptorWrite)/sizeof(*descriptorWrite),
                           descriptorWrite, 0, nullptr);
    return true;
}

bool VulkanApp::createOverlayDescriptor()
{
    VkDescriptorPoolSize poolSizes[1];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = 2;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &textOverlay.desc.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = textOverlay.desc.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &textOverlay.layout;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo, &textOverlay.desc.set);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateDescriptorSets failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorImageInfo imageInfo[1];
    memset(imageInfo, 0, sizeof(imageInfo));
    imageInfo[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo[0].imageView = textOverlay.font.view;
    imageInfo[0].sampler = textOverlay.fontSampler;

    VkWriteDescriptorSet descriptorWrite[1];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = textOverlay.desc.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType =
                                     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pImageInfo = imageInfo;

    vkUpdateDescriptorSets(device,
                           sizeof(descriptorWrite)/sizeof(*descriptorWrite),
                           descriptorWrite, 0, nullptr);
    return true;
}

bool VulkanApp::createShipDescriptor()
{
    VkDescriptorPoolSize poolSizes[3];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_SAMPLER;
    poolSizes[1].descriptorCount = 1;
    poolSizes[2].type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    poolSizes[2].descriptorCount = sprites.textures.size();

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &spritesDescriptor.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = spritesDescriptor.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &spritesDescriptorLayout;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo, &spritesDescriptor.set);
    if (vkRet != VK_SUCCESS) {
        printf("vkAllocateDescriptorSets failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorBufferInfo bufferInfo[1];
    memset(bufferInfo, 0, sizeof(bufferInfo));
    bufferInfo[0].buffer = vpUniformBuffer;
    bufferInfo[0].offset = 0;
    bufferInfo[0].range = sizeof(vp);

    VkDescriptorImageInfo samplerImageInfo = {};
    samplerImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    samplerImageInfo.imageView = nullptr;
    samplerImageInfo.sampler = spritesSampler;

    VkDescriptorImageInfo textureImageInfo[sprites.textures.size()];
    memset(textureImageInfo, 0, sizeof(textureImageInfo));
    for (uint32_t i = 0; i < sprites.textures.size(); ++i) {
        textureImageInfo[i].imageLayout =
                                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        textureImageInfo[i].imageView = sprites.textures[i].view;
        textureImageInfo[i].sampler = nullptr;
    }

    VkWriteDescriptorSet descriptorWrite[3];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = spritesDescriptor.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pBufferInfo = bufferInfo;

    descriptorWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[1].dstSet = spritesDescriptor.set;
    descriptorWrite[1].dstBinding = 1;
    descriptorWrite[1].dstArrayElement = 0;
    descriptorWrite[1].descriptorType =
                                    //VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                                    VK_DESCRIPTOR_TYPE_SAMPLER;
    descriptorWrite[1].descriptorCount = 1;
    descriptorWrite[1].pImageInfo = &samplerImageInfo;

    descriptorWrite[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[2].dstSet = spritesDescriptor.set;
    descriptorWrite[2].dstBinding = 2;
    descriptorWrite[2].dstArrayElement = 0;
    descriptorWrite[2].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    descriptorWrite[2].descriptorCount = sprites.textures.size();
    descriptorWrite[2].pImageInfo = textureImageInfo;

    vkUpdateDescriptorSets(device,
                           sizeof(descriptorWrite)/sizeof(*descriptorWrite),
                           descriptorWrite, 0, nullptr);

    return true;
}

bool VulkanApp::createExplosionDescriptor()
{
    VkDescriptorPoolSize poolSizes[2];
    memset(&poolSizes, 0, sizeof(poolSizes));
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = sizeof(poolSizes)/sizeof(*poolSizes);
    poolInfo.pPoolSizes = poolSizes;
    poolInfo.maxSets = 1;

    VkResult vkRet = vkCreateDescriptorPool(device, &poolInfo, nullptr,
                                            &explosions.desc.pool);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateDescriptorPool failed with %d\n", vkRet);
        return false;
    }

    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = explosions.desc.pool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &explosions.layout;

    vkRet = vkAllocateDescriptorSets(device, &allocInfo, &explosions.desc.set);
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
    imageInfo.imageView = explosions.atlas.view;
    imageInfo.sampler = explosions.sampler;

    VkWriteDescriptorSet descriptorWrite[2];
    memset(descriptorWrite, 0, sizeof(descriptorWrite));
    descriptorWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[0].dstSet = explosions.desc.set;
    descriptorWrite[0].dstBinding = 0;
    descriptorWrite[0].dstArrayElement = 0;
    descriptorWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite[0].descriptorCount = 1;
    descriptorWrite[0].pBufferInfo = bufferInfo;
    descriptorWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite[1].dstSet = explosions.desc.set;
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
     || !createShipDescriptor()
     || !createExplosionDescriptor()
     || !createOverlayDescriptor()
     || !createHUDDescriptor()) {
        return false;
    }
    return true;
}

bool VulkanApp::setupCommandBuffers()
{
    for (uint32_t i = 0; i < commandBuffers.size(); ++i) {
        if (!resetCommandBuffer(i, 0.0))
             return false;
    }
    return true;
}

bool VulkanApp::resetCommandBuffer(uint32_t i, double currentTime)
{
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

    VkClearValue clearColor[1] = {};
    memset(clearColor, 0, sizeof(clearColor));
    clearColor[0].color.float32[0] = 0.0f;
    clearColor[0].color.float32[1] = 0.0f;
    clearColor[0].color.float32[2] = 0.0f;
    clearColor[0].color.float32[3] = 1.0f;
    //clearColor[1] is for the resolve attachment
    //clearColor[2].depthStencil = {1.0f, 0};
    renderPassInfo.clearValueCount = 1;
    renderPassInfo.pClearValues = clearColor;

    // Background
    vkCmdBeginRenderPass(b, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                      backgroundPipeline);

    VkBuffer buffers[] = {backgroundVertex.buffer};
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
    vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            backgroundPipelineLayout, 0, 1,
                            &backgroundDescriptor.set, 0, nullptr);

    vkCmdDraw(b, backgroundVertex.vertices.size(), 1, 0, 0);

    // Sprites
    float noEasingFactor[MAX_SPRITES_PER_DRAW];
    for (uint32_t i = 0; i < MAX_SPRITES_PER_DRAW; ++i)
        noEasingFactor[i] = 1.0f;
    vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                      spritesPipeline);
    MyMatrix shipTransform = shipState.getTransform();
    vkCmdPushConstants(b, spritesPipelineLayout,
                       VK_SHADER_STAGE_VERTEX_BIT, 0,
                       sizeof(shipTransform), &shipTransform);
    uint32_t offset = vertexPushConstantsSize;
    uint32_t idx = sprites.shipTextureIndex;
    vkCmdPushConstants(b, spritesPipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT,
                       offset,
                       sizeof(idx), &idx);
    offset += MAX_SPRITES_PER_DRAW * sizeof(int);
    vkCmdPushConstants(b, spritesPipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT,
                       offset,
                       sizeof(noEasingFactor), noEasingFactor);
    offset += sizeof(noEasingFactor);

    // Apply a change in color in the fragment shader.  Used to make the ship
    // red when hit.
    float colorFix = 0.0f;
    if (shipState.inHitAnim) {
        const double diff = currentTime - shipState.startHitAnim;
        if (diff > 2.0) {
            shipState.inHitAnim = false;
        }
        else {
            colorFix = 1.0f - diff / 2.0f;
        }
    }
    vkCmdPushConstants(b, spritesPipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT, offset,
                       sizeof(float), &colorFix);

    buffers[0] = {spriteVertex.buffer};
    offsets[0] = {0};
    vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
    vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            spritesPipelineLayout, 0, 1,
                            &spritesDescriptor.set, 0, nullptr);

    vkCmdDraw(b, 6, 1, shipVertexIndex * 6, 0);
    float zero = 0.0f;
    vkCmdPushConstants(b, spritesPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT,
                       offset, sizeof(float), &zero);
    if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_UP)) {
        uint32_t idx = sprites.shipEngineTextureIndex;
        vkCmdPushConstants(b, spritesPipelineLayout,
                           VK_SHADER_STAGE_FRAGMENT_BIT,
                           vertexPushConstantsSize,
                           sizeof(idx), &idx);
        vkCmdDraw(b, 6, 1, shipEngineVertexIndex * 6, 0);
    }

    // Asteroids
    if (!asteroidStates.empty()) {
        uint32_t texIndices[asteroidStates.size()];
        MyMatrix transforms[asteroidStates.size()];
        float easing[asteroidStates.size()];
        for (uint32_t sz = 0; sz < 3; ++sz) {
            uint32_t numToDraw = 0;
            for (uint32_t i = 0 ; i < asteroidStates.size(); ++i) {
                if (asteroidStates[i].sizeType != sz) {
                    continue;
                }
                transforms[numToDraw] = asteroidStates[i].getTransform();
                texIndices[numToDraw] = asteroidStates[i].textureIndex;
                easing[numToDraw] = (currentTime - asteroidStates[i].birth);
                ++numToDraw;
            }
            if (numToDraw == 0) {
                continue;
            }

            const uint32_t maxDraw = MAX_SPRITES_PER_DRAW;
            const uint32_t numDrawCalls = numToDraw / maxDraw + 1;
            for (uint32_t it = 0; numToDraw && it < numDrawCalls; ++it) {
                vkCmdPushConstants(b, spritesPipelineLayout,
                                   VK_SHADER_STAGE_VERTEX_BIT, 0,
                                   sizeof(*transforms) * maxDraw,
                                   transforms + it * maxDraw);
                vkCmdPushConstants(b, spritesPipelineLayout,
                                   VK_SHADER_STAGE_FRAGMENT_BIT,
                                   vertexPushConstantsSize,
                                   sizeof(*texIndices) * maxDraw,
                                   texIndices + it * maxDraw);
                uint32_t off = vertexPushConstantsSize +
                               sizeof(*texIndices) * maxDraw;
                vkCmdPushConstants(b, spritesPipelineLayout,
                                   VK_SHADER_STAGE_FRAGMENT_BIT,
                                   off,
                                   sizeof(*easing) * maxDraw,
                                   easing + it * maxDraw);
                vkCmdDraw(b, 6, min(maxDraw, numToDraw),
                          (asteroidVertexIndexBig + sz) * 6, 0);
                numToDraw -= maxDraw;
            }
        }
        offset = vertexPushConstantsSize;
        offset += MAX_SPRITES_PER_DRAW * sizeof(int);
        vkCmdPushConstants(b, spritesPipelineLayout,
                           VK_SHADER_STAGE_FRAGMENT_BIT,
                           offset, sizeof(noEasingFactor), noEasingFactor);
    }

    // Bullet
    if (bulletState.live) {
        MyMatrix bulletTransform = bulletState.getTransform();
        vkCmdPushConstants(b, spritesPipelineLayout,
                           VK_SHADER_STAGE_VERTEX_BIT, 0,
                           sizeof(bulletTransform), &bulletTransform);
        uint32_t idx = sprites.shipBulletTextureIndex;
        vkCmdPushConstants(b, spritesPipelineLayout,
                           VK_SHADER_STAGE_FRAGMENT_BIT,
                           vertexPushConstantsSize,
                           sizeof(idx), &idx);
        vkCmdDraw(b, 6, 1, shipBulletVertexIndex * 6, 0);
    }

    // Explosions
    vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                      explosions.pipeline);
    buffers[0] = {explosions.vertex.buffer};
    offsets[0] = {0};
    vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
    vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            explosions.pipelineLayout, 0, 1,
                            &explosions.desc.set, 0, nullptr);
    auto it = explosions.actives.begin();
    while (it != explosions.actives.end()) {
        if (it->start == 0.0) {
            it->start = currentTime;
        }
        if (currentTime - it->start > 2.0) {
            it = explosions.actives.erase(it);
            continue;
        }
        MyMatrix transform;
        transform.set(0, 3, it->center.x);
        transform.set(1, 3, it->center.y);
        transform.set(2, 3, it->center.z);
        vkCmdPushConstants(b, explosions.pipelineLayout,
                           VK_SHADER_STAGE_VERTEX_BIT, 0,
                           sizeof(transform), &transform);
        uint32_t idx = rint((currentTime - it->start) * 32);
        vkCmdPushConstants(b, explosions.pipelineLayout,
                           VK_SHADER_STAGE_FRAGMENT_BIT,
                           sizeof(transform),
                           sizeof(idx), &idx);
        vkCmdDraw(b, explosions.vertex.vertices.size(), 1, 0, 0);
        ++it;
    }

    // Overlay
    vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                      textOverlay.pipeline);
    buffers[0] = {textOverlay.vertex.buffer};
    offsets[0] = {0};
    vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
    vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            textOverlay.pipelineLayout, 0, 1,
                            &textOverlay.desc.set, 0, nullptr);
    vkCmdDraw(b, textOverlay.vertex.vertices.size(), 1, 0, 0);

    // HUD
    vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS,
                      hud.pipeline);
    buffers[0] = {hud.vertex.buffer};
    offsets[0] = {0};
    vkCmdBindVertexBuffers(b, 0, 1, buffers, offsets);
    vkCmdBindDescriptorSets(b, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            hud.pipelineLayout, 0, 1,
                            &hud.desc.set, 0, nullptr);
    float textureBypass = 1.0f;
    vkCmdPushConstants(b, hud.pipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT,
                       0, sizeof(float), &textureBypass);
    vkCmdDraw(b, 6, 1, 0, 0);
    if (hud.vertex.vertices.size() > 6) {
        float textureBypass = 0.0f;
        vkCmdPushConstants(b, hud.pipelineLayout,
                           VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(float), &textureBypass);
        vkCmdDraw(b, hud.vertex.vertices.size(), 1, 6, 0);
    }

    vkCmdEndRenderPass(b);

    VkResult vkRet = vkEndCommandBuffer(b);
    if (vkRet != VK_SUCCESS) {
        printf("vkEndCommandBuffer failed with %d\n", vkRet);
        return false;
    }
    return true;
}

void VulkanApp::spawnNewAsteroid(double currentTime)
{
    const float halfXSize = bigAsteroidSize[0] / 2.0f;
    const float halfYSize = bigAsteroidSize[1] / 2.0f;
    static uniform_real_distribution<float> disX(getMinX() + halfXSize,
                                                 -getMinX() + halfXSize);
    static uniform_real_distribution<float> disY(getMinY() + halfYSize,
                                                 -getMinY() + halfYSize);
    static uniform_real_distribution<int> spawnSide(0, 1);
    MyPoint shipVertices[6];
    const uint32_t startIdx = 6 * shipVertexIndex;
    for (uint32_t i = 0; i < 6; ++i) {
        const Vertex v = spriteVertex.vertices[startIdx + i];
        shipVertices[i] = v.pos.transform(shipState.orientation);
        shipVertices[i] += shipState.position;
    }
    MyAABB2 shipAABB;
    shipAABB.min = {FLT_MAX, FLT_MAX};
    shipAABB.max = { -FLT_MAX, -FLT_MAX};
    for (uint32_t i = 0; i < 6; ++i) {
        shipAABB.min.x = min(shipAABB.min.x, shipVertices[i].x);
        shipAABB.min.y = min(shipAABB.min.y, shipVertices[i].y);

        shipAABB.max.x = max(shipAABB.max.x, shipVertices[i].x);
        shipAABB.max.y = max(shipAABB.max.y, shipVertices[i].y);
    }
    MyAABB2 asteroidsAABB[asteroidStates.size()];
    for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
        float sz[2];
        getAsteroidSize(sz, asteroidStates[i]);
        asteroidsAABB[i] = getSphereAABB(sz, asteroidStates[i].position);
    }
    while (1) {
        AsteroidState newAsteroid;
        if (spawnSide(randomGen)) {
            newAsteroid.position = {disX(randomGen),
                                    -getMinY() - bigAsteroidSize[1] / 2.0f,
                                    0.0f};
            if (spawnSide(randomGen)) {
                newAsteroid.position.y *= -1.0f;
            }
        }
        else {
            newAsteroid.position = {-getMinX() - bigAsteroidSize[0] / 2.0f,
                                    disY(randomGen), 0.0f};
            if (spawnSide(randomGen)) {
                newAsteroid.position.y *= -1.0f;
            }
        }

        MyAABB2 newAsteroidAABB = getSphereAABB(bigAsteroidSize,
                                                newAsteroid.position);

        if (newAsteroidAABB.overlap(shipAABB)) {
            continue;
        }
        bool tryagain = false;
        for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
            if (newAsteroidAABB.overlap(asteroidsAABB[i])) {
                tryagain = true;
                break;
            }
        }
        if (tryagain) {
            continue;
        }

        // Generate velocity
        MyPoint direction(0.0f, 1.0f, 0.0f);
        static uniform_real_distribution<float> angle(-M_PI, M_PI);

        MyQuaternion rot;
        rot.rotateZ(angle(randomGen));
        MyPoint v = direction.transform(rot);
        v.normalize();

        static uniform_real_distribution<float> velocityFactor(1e-1f,2e-1f);
        v *= velocityFactor(randomGen);
        newAsteroid.velocity = v;

        uniform_int_distribution<uint32_t> asteroidTextures{
                                   sprites.asteroidTextureStartIndex,
                                   sprites.asteroidTextureEndIndex};
        newAsteroid.textureIndex = asteroidTextures(randomGen);

        // radius of the collision sphere is the size of the y axis
        // this is completely based on the assset we're using
        newAsteroid.radius = bigAsteroidSize[1] / 2.0f * 0.98f;

        newAsteroid.generateTensor();
        newAsteroid.rotStartTime = currentTime;
        newAsteroid.birth = currentTime;

        asteroidStates.push_back(newAsteroid);
        //puts("spawned");
        break;
    }
}

bool VulkanApp::renderFrame(uint32_t renderCount, double currentTime,
                            double dt)
{
    (void) currentTime; // remove when we actually render stuff
    // Draw
    const uint32_t idx = renderCount % swapChain.size();

    //vkWaitForFences(device, 1, &swapChain[idx].fence, VK_TRUE, UINT64_MAX);
    //vkResetFences(device, 1, &swapChain[idx].fence);

#ifndef TEST_COLLISIONS
    if (asteroidStates.size() < maxNumAsteroids) {
        if (currentTime > (lastSpawnRandCheck + 1.0)) {
            lastSpawnRandCheck = currentTime;
            if (asteroidStates.empty() || asteroidSpawnRand(randomGen) == 1)
                spawnNewAsteroid(currentTime);
        }
    }
#else
    if (asteroidStates.size() < 2) {
        spawnNewAsteroid(currentTime);
    }
#endif
    if (bulletState.live) {
        bulletState.update(dt, this);
    }
    else if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_SPACE)) {
        // We should start with the same orientation as the ship but the
        // model is not facing the same way.  XXX maybe flip the vertices
        MyQuaternion oneMore;
        oneMore.rotateZ(M_PI);
        bulletState.orientation = shipState.orientation * oneMore;

        MyPoint firingDirection(0.0f, -1.0f, 0.0f);
        firingDirection = firingDirection.transform(shipState.orientation);
        firingDirection.normalize();
        const float disp = shipSize[1] / 2.0f + bulletSize[1] / 2.0f;

        bulletState.position = shipState.position + firingDirection * disp;
        bulletState.velocity = firingDirection * 0.5f;
        bulletState.live = true;
    }

    for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
        for (uint32_t j = (i+1) ; j < asteroidStates.size(); ++j) {
            processCollisions(asteroidStates[i], asteroidStates[j], i, j);
        }
    }
    for (AsteroidState& a : asteroidStates)
        a.update(currentTime, dt, this);

    for (uint32_t i = 0; i < asteroidStates.size(); ++i) {
        for (uint32_t j = (i+1) ; j < asteroidStates.size(); ++j) {
            resolveAsteroidCollisions(asteroidStates[i], asteroidStates[j],
                                      true, i, j);
        }
    }
    if (bulletState.live && !asteroidStates.empty()) {
        checkForBulletHit();
    }
    if (!asteroidStates.empty()) {
        checkForAsteroidHit(currentTime);
    }

    shipState.update(currentTime, dt, this);

    resetCommandBuffer(idx, currentTime);

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

    vkRet = vkQueueSubmit(graphicsQueue, 1, &submitInfo, nullptr); //swapChain[idx].fence);
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
    waitForIdle();

    return true;
}

bool VulkanApp::recreateSwapChain()
{
    // XXX FIXME
    waitForIdle();

    updateExtent();

    cleanupSwapChain();
    if (!createSwapChain()
     //|| !createDepthResources()
     || !loadShaders()
     || !createRenderPass()
     || !createPipelines()
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
    vkDestroyPipeline(device, backgroundPipeline, nullptr);
    vkDestroyPipeline(device, spritesPipeline, nullptr);
    vkDestroyPipeline(device, explosions.pipeline, nullptr);
    vkDestroyPipeline(device, textOverlay.pipeline, nullptr);
    vkDestroyPipeline(device, hud.pipeline, nullptr);
    vkDestroyPipelineLayout(device, backgroundPipelineLayout, nullptr);
    vkDestroyPipelineLayout(device, spritesPipelineLayout, nullptr);
    vkDestroyPipelineLayout(device, explosions.pipelineLayout, nullptr);
    vkDestroyPipelineLayout(device, textOverlay.pipelineLayout, nullptr);
    vkDestroyPipelineLayout(device, hud.pipelineLayout, nullptr);
    vkDestroyRenderPass(device, renderPass, nullptr);
    vkDestroyShaderModule(device, backgroundVertexShader, nullptr);
    vkDestroyShaderModule(device, backgroundFragmentShader, nullptr);
    vkDestroyShaderModule(device, spriteVertexShader, nullptr);
    vkDestroyShaderModule(device, spritesFragmentShader, nullptr);
    vkDestroyShaderModule(device, explosions.frag, nullptr);
    vkDestroyShaderModule(device, explosions.vert, nullptr);
    vkDestroyShaderModule(device, textOverlay.frag, nullptr);
    vkDestroyShaderModule(device, textOverlay.vert, nullptr);
    vkDestroyShaderModule(device, hud.frag, nullptr);
    vkDestroyShaderModule(device, hud.vert, nullptr);
    for (auto& swpe : swapChain) {
        vkDestroyFence(device, swpe.fence, nullptr);
        vkDestroySemaphore(device, swpe.imageAvailableSem, nullptr);
        vkDestroySemaphore(device, swpe.renderFinishedSem, nullptr);
        vkDestroyImageView(device, swpe.view, nullptr);
        // Note that the swpe.image is owned by and will be deallocated
        // through vkSwapChain
#ifdef MSAA
        vkDestroyImageView(device, swpe.msaaView, nullptr);
        vkFreeMemory(device, swpe.msaaMemory, nullptr);
        vkDestroyImage(device, swpe.msaaImage, nullptr);
#endif
        //vkDestroyImageView(device, swpe.depthImageView, nullptr);
        //vkFreeMemory(device, swpe.depthImageMemory, nullptr);
        //vkDestroyImage(device, swpe.depthImage, nullptr);
    }
    backgroundTexture.cleanup(device);
    vkDestroySampler(device, backgroundSampler, nullptr);
    for (uint32_t i = 0 ; i < sprites.textures.size(); ++i) {
        sprites.textures[i].cleanup(device);
    }
    explosions.atlas.cleanup(device);
    textOverlay.font.cleanup(device);
    hud.health.cleanup(device);
    vkDestroySampler(device, spritesSampler, nullptr);
    vkDestroySampler(device, explosions.sampler, nullptr);
    vkDestroySampler(device, textOverlay.fontSampler, nullptr);
    vkDestroySampler(device, hud.healthSampler, nullptr);

    vkDestroySwapchainKHR(device, vkSwapChain, nullptr);
}

void VulkanApp::cleanup()
{
    cleanupSwapChain();
    vkDestroyCommandPool(device, commandPool, nullptr);

    backgroundVertex.cleanup(device);
    spriteVertex.cleanup(device);
    explosions.vertex.cleanup(device);
    textOverlay.vertex.cleanup(device);
    hud.vertex.cleanup(device);

    //vkDestroyBuffer(device, cubeTransformsUniformBuffer, nullptr);
    //vkUnmapMemory(device, cubeTransformsUniformBufferMemory);
    //vkFreeMemory(device, cubeTransformsUniformBufferMemory, nullptr);

    vkDestroyBuffer(device, vpUniformBuffer, nullptr);
    vkUnmapMemory(device, vpUniformMemory);
    vkFreeMemory(device, vpUniformMemory, nullptr);

    vkDestroyDescriptorPool(device, backgroundDescriptor.pool, nullptr);
    vkDestroyDescriptorPool(device, spritesDescriptor.pool, nullptr);
    vkDestroyDescriptorPool(device, explosions.desc.pool, nullptr);
    vkDestroyDescriptorPool(device, textOverlay.desc.pool, nullptr);
    vkDestroyDescriptorPool(device, hud.desc.pool, nullptr);
    vkDestroyDescriptorSetLayout(device, backgroundDescriptorLayout, nullptr);
    vkDestroyDescriptorSetLayout(device, spritesDescriptorLayout, nullptr);
    vkDestroyDescriptorSetLayout(device, explosions.layout, nullptr);
    vkDestroyDescriptorSetLayout(device, textOverlay.layout, nullptr);
    vkDestroyDescriptorSetLayout(device, hud.layout, nullptr);

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
    region.imageExtent = { width, height, 1 };

    vkCmdCopyBufferToImage(commandBuffer, buffer, image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    endSingleTimeCommands(commandBuffer);
}

bool VulkanApp::loadFont(struct Texture *texture,
                         stbtt_bakedchar *fontData,
                         int firstChar,
                         const char *file)
{
    vector<char> ttf;
    if (!readFile(&ttf, file)) {
        return false;
    }
    unsigned char font[512 * 512];
    stbtt_BakeFontBitmap((const unsigned char*)ttf.data(),0, 24.0,
                         font, 512, 512, firstChar, 126 - firstChar, fontData);
    VkDeviceSize imageSize = sizeof(font);
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer(&stagingBuffer, &stagingBufferMemory,
                 imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                 VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 false);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
    memcpy(data, font, static_cast<size_t>(imageSize));
    vkUnmapMemory(device, stagingBufferMemory);
    texture->width = 512;
    texture->height = 512;

    VkImageCreateInfo imageInfo = {};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = texture->width;
    imageInfo.extent.height = texture->height;
    imageInfo.extent.depth = 1; // required for 2D image
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = VK_FORMAT_R8_UNORM;
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

    transitionImageLayout(texture->image, VK_FORMAT_R8_UNORM,
                          VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    copyBufferToImage(stagingBuffer, texture->image, texture->width,
                      texture->height);
    transitionImageLayout(texture->image, VK_FORMAT_R8_UNORM,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);

    VkImageViewCreateInfo viewInfo = {};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = texture->image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8_UNORM;
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

    return true;
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

    return true;
}

bool VulkanApp::createTextures() {
    uint32_t shipIdx = sprites.shipTextureIndex;
    uint32_t engineIdx = sprites.shipEngineTextureIndex;
    uint32_t bulletIdx = sprites.shipBulletTextureIndex;
    uint32_t asteroidStart = sprites.asteroidTextureStartIndex;
    if (!createTexture(&backgroundTexture, "assets/background.png")
     || !createTexture(&sprites.textures[shipIdx], "assets/ship.png")
     || !createTexture(&sprites.textures[engineIdx], "assets/engine1.png")
     || !createTexture(&sprites.textures[bulletIdx], "assets/bullet.png")
     || !createTexture(&sprites.textures[asteroidStart],
                       "assets/asteroid_v1.png")
     || !createTexture(&sprites.textures[asteroidStart + 1],
                       "assets/asteroid_v2.png")
     || !createTexture(&sprites.textures[asteroidStart + 2],
                       "assets/asteroid_v3.png")
     || !createTexture(&sprites.textures[asteroidStart + 3],
                       "assets/asteroid_v4.png")
     || !createTexture(&explosions.atlas, "assets/explosion_atlas.png")
     || !loadFont(&textOverlay.font, textOverlay.fontData,
                  textOverlay.fontFirstChar, "assets/kenvector_future.ttf")
     || !createTexture(&hud.health, "assets/health_bar.png")) {
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
    samplerInfo.maxAnisotropy = 8.0f;
    samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = 0.0f;

    VkResult vkRet;
    vkRet = vkCreateSampler(device, &samplerInfo, nullptr, &backgroundSampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }
    vkRet = vkCreateSampler(device, &samplerInfo, nullptr, &spritesSampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }
    vkRet = vkCreateSampler(device, &samplerInfo, nullptr, &explosions.sampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }
    vkRet = vkCreateSampler(device, &samplerInfo, nullptr, &hud.healthSampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }
    samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    vkRet = vkCreateSampler(device, &samplerInfo, nullptr,
                            &textOverlay.fontSampler);
    if (vkRet != VK_SUCCESS) {
        printf("vkCreateSampler failed with %d\n", vkRet);
        return false;
    }
    return true;
}

int main(void)
{
    VulkanApp app;
    app.run();

    return 0;
}
