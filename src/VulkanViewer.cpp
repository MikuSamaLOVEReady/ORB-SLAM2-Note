//
// Created by mikusama on 2/6/25.
//

#pragma GCC optimize ("O0")

#include "VulkanViewer.h"
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <future>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>



namespace ORB_SLAM2{

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
        void* pUserData)
    {
        //std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
        /// 严重程度判定
        if ( messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT ) {
            std::cerr << "warning: validation layer: " << pCallbackData->pMessage << std::endl;
        }
        return VK_FALSE;
    }

    void VulkanViewer::Run()
    {
        mbFinished = false;

        /// vulkan
        InitWindow();

        InitVulkan();

        InitImGui();
        /// main Loop
        mainLoop();
        mbFinished = true;
    }

    VulkanViewer::VulkanViewer(System* pSystem , MapDrawer* mpdrawer): mpSystem(pSystem)
    ,mbFinished(true),mbFinishRequested(false),mpMapDrawer(mpdrawer)
    {
        mpcamera = new Camera(glm::vec3(2.0f ,2.0f, 2.0f));
    }


    bool VulkanViewer::Stop()
    {
        unique_lock<mutex> lock(mMutexFinish);
        if (mbFinishRequested)
            return false;

        return false;
    }

    void VulkanViewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    ///
    bool VulkanViewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }


    void VulkanViewer::InitWindow()
    {
        glfwInit();

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        //glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        if (!glfwVulkanSupported())
        {
            printf("GLFW: Vulkan Not Supported\n");
        }

        mpWindow = glfwCreateWindow(WIDTH, HEIGHT, "Rulomi VK", nullptr, nullptr);
        glfwSetWindowUserPointer( mpWindow , this); /// 这里注册后 glfwGetWindowUserPointer 就可以重新获取VulkanViewer了
        glfwSetCursorPosCallback(mpWindow , mouse_callback);
        glfwSetMouseButtonCallback(mpWindow , mouse_button_callback);
        glfwSetFramebufferSizeCallback(mpWindow, framebufferResizeCallback);

        glfwSwapInterval(0); /// 看真实帧率
    }

    /// init vulkan本质是都是做一些格式设定，空间开辟的工作
    /// 真正填充数据，在DrawCall阶段
    bool VulkanViewer::InitVulkan(){
        createInstance();
        setupDebugMessenger();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
        createSwapChain();
        createImageViews();
        createRenderPass();
        createDescriptorSetLayout();    /// 设置Uniform Buffer 格式
        createGraphicsPipline();        /// 设置 pipline
        createFramebuffers();
        createCommandPool();
        createVertexBuffer();           /// 提供静态数据
        /// TODO：Index buffer
        createIndexBuffer();
        createUniformBuffers();         /// 提供UBO独立输入  创建Uniform buffer空间

        createDescriptorPool();
        createDescriptorSets();

        createCommandBuffers();         /// 每一帧独立占有的指令队列
        createSyncObjects();
    }

    bool VulkanViewer::createInstance()
    {
        if ( enableValidationLayers && !checkValidationLayerSupport() ) {
            throw std::runtime_error("Validation layers requested, but not available!");
        }
        
        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "Hello Triangle";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.pEngineName = "No Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;

        auto Extensions = getRequiredExtensions();
        createInfo.enabledExtensionCount = Extensions.size();
        createInfo.ppEnabledExtensionNames = Extensions.data();

        VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};
        if (enableValidationLayers)
        {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationlayers.size());
            createInfo.ppEnabledLayerNames = validationlayers.data();

            /// 关键点，能在crete instance的时候检查error
            populateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*) &debugCreateInfo;
        }
        else
        {
            createInfo.enabledLayerCount = 0;
            createInfo.pNext = nullptr;
        }

        VkResult result = vkCreateInstance(&createInfo, nullptr, &instance);
        if ( result != VK_SUCCESS) {
            std::cerr << "vkCreateInstance failed!: " << result << " VS " << VK_SUCCESS << std::endl;
            throw std::runtime_error("failed to create instance!");
        }
        return true;
    }

    void VulkanViewer::setupDebugMessenger()
    {
        if(!enableValidationLayers) return;

        VkDebugUtilsMessengerCreateInfoEXT createInfo;
        populateDebugMessengerCreateInfo(createInfo);

        if ( CreateDebugUtilsMessengerEXT(instance, &createInfo , nullptr, &debugMessenger) != VK_SUCCESS ) {
            throw std::runtime_error("failed to set up debug messenger");
        }
        return;
    }


    void VulkanViewer::processInput(GLFWwindow *window) {
        float camera_speed = 2.5f * detalTime;
        if(glfwGetKey(window , GLFW_KEY_SPACE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        if( glfwGetKey(window , GLFW_KEY_W) == GLFW_PRESS )
            mpcamera->cameraPos += mpcamera->cameraFront * camera_speed;
        if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            mpcamera->cameraPos -= mpcamera->cameraFront * camera_speed;
        if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)            /// 这里的左右轴 都是相对于相机自身的
            mpcamera->cameraPos -= glm::normalize( glm::cross(mpcamera->cameraFront , mpcamera->cameraUp) ) * camera_speed;
        if(glfwGetKey(window , GLFW_KEY_D ) == GLFW_PRESS)
            mpcamera->cameraPos += glm::normalize( glm::cross(mpcamera->cameraFront , mpcamera->cameraUp) ) * camera_speed;
    }

    static float fpsAccumulator = 0.0f;
    static int frameCount = 0;
    static float fps = 0.0f;
    static float lastTime = 0.0f;

    void VulkanViewer::mainLoop()
    {
        while (!glfwWindowShouldClose(mpWindow)) {
            float currentFrame = glfwGetTime();
            detalTime =  currentFrame - lastFrame;
            lastFrame = currentFrame;
            processInput(mpWindow);
            glfwPollEvents();

            /// Imgui UI渲染
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();


            renderPerformance();


            drawFrame();

            /// 轮寻退出
            if(CheckFinish())
                return;
        }
        vkDeviceWaitIdle(device);
    }

    void VulkanViewer::cleanup()
    {
        cleanupSwapChain();

        for ( size_t i = 0; i < swapChainExtent.width; i++ ) {
            vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
            vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
            vkDestroyFence( device , inFlightFences[i] , nullptr);
        }

        for ( size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++ ) {
            vkDestroyBuffer(device , uniformBuffers[i], nullptr);
            vkFreeMemory(device , uniformBuffersMemory[i], nullptr);
        }

        vkDestroyDescriptorPool(device, descriptorPool, nullptr);       ///  descriptorSet 不需要手动释放，把池子卸掉就自动free了
        vkDestroyDescriptorSetLayout( device , descriptorSetLayout, nullptr);
        vkDestroyCommandPool(device, commandPool, nullptr);                 /// 自动能将command pool释放
        vkDestroyBuffer(device, vertexBuffer, nullptr);
        vkFreeMemory(device, vertexBufferMemory, nullptr);
        vkDestroyBuffer(device , indexBuffer, nullptr);
        vkFreeMemory(device, indexBufferMemory, nullptr);
        vkDestroyPipeline(device , graphicsPipeline , nullptr);
        vkDestroyPipelineLayout(device , pipelineLayout , nullptr);
        vkDestroyRenderPass(device, renderPass, nullptr);
        vkDestroyDevice(device , nullptr);
        vkDestroySurfaceKHR( instance , surface , nullptr);
        vkDestroyInstance(instance, nullptr);

        if (enableValidationLayers) {
            DestoryDebugUtilsMessengerEXT(instance,debugMessenger,nullptr);
        }

        glfwDestroyWindow(mpWindow);
        glfwTerminate();
    }

    bool VulkanViewer::checkValidationLayerSupport()
    {
        uint32_t layerCount;
        vkEnumerateInstanceLayerProperties(&layerCount , nullptr );
        std::vector<VkLayerProperties> availableLayers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount , availableLayers.data() );

        for ( const char* layerName : validationlayers ) {
            bool layerFound = false;
            for ( const auto& layerProperties : availableLayers ) {
                if ( std::strcmp( layerProperties.layerName , layerName ) == 0 ) {
                    layerFound = true;
                    break;
                }
            }
            if ( !layerFound ) {
                return false;
            }
        }
        return true;
    }

    std::vector<const char*> VulkanViewer::getRequiredExtensions()
    {
        uint32_t glfwExtensionCount = 0;
        const char** glfwExtenstions;

        glfwExtenstions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        /// 这里是iterator的移动
        std::vector<const char*> extensions(glfwExtenstions , glfwExtenstions+glfwExtensionCount);

        if (enableValidationLayers) {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }
        return extensions;
    }

    void VulkanViewer::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo)
    {
        createInfo = {};
        /// 这个结构体 用于配置 Messenger ，本质上就像一个配置表。
        createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
        | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = debugCallback;             // 产生事件后，触发的callback
        createInfo.pUserData = nullptr;                         // 给回调传参数
    }

    VkResult VulkanViewer::CreateDebugUtilsMessengerEXT(VkInstance instance,
        const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator,
        VkDebugUtilsMessengerEXT* pDebugMessenger)
    {
        auto func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance , "vkCreateDebugUtilsMessengerEXT");

        if (func != nullptr) {
            return func(instance , pCreateInfo, pAllocator, pDebugMessenger);
        }else {
            return VK_ERROR_EXTENSION_NOT_PRESENT;
        }
    }

    void VulkanViewer::DestoryDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger,
        const VkAllocationCallbacks* pAllocator)
    {
         auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr( instance , "vkDestroyDebugUtilsMessengerEXT" );

        if ( func != nullptr ) {
            func(instance , debugMessenger, pAllocator );
        }
    }

    void VulkanViewer::pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
        if (deviceCount == 0) {
            throw std::runtime_error("failed to find GPUs with Vulkan support!");
        }
        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

        for (const auto& device : devices) {
            if ( isDeviceSuitable(device) ) {
                physicalDevice = device;
                break;
            }
        }

        if (physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    bool VulkanViewer::isDeviceSuitable(VkPhysicalDevice device) {
        /// 检查GPU特性：
        VkPhysicalDeviceProperties deviceProperties;
        vkGetPhysicalDeviceProperties(device, &deviceProperties);

        VkPhysicalDeviceFeatures deviceFeatures;
        vkGetPhysicalDeviceFeatures(device, &deviceFeatures);   /// 特性： geometry shader ，texture compression, 64 bit floats and multi viewport rendering
        /// 开启非Solid填充模式
        if (!deviceFeatures.fillModeNonSolid) {
            deviceFeatures.fillModeNonSolid = VK_TRUE;
            throw std::runtime_error("GPU does not support fillModeNonSolid! Cannot use VK_POLYGON_MODE_POINT.");
        }

        /// 检查GPU queue 功能，目前只检测 图形性能支持 + 能输出到window
        QueueFamilyIndices indices = findQueueFamilies(device);

        /// 检查SwapChain 硬件支持.
        bool extensionsSupport = checkDeviceExtensionSupport(device);

        /// 硬件+surface，vulkan与窗口无关，需要在连同surface做一次充分性检查。
        bool swapChainAdequate = false;
        if (extensionsSupport) {
            SwapChainSupportDetails swapChainSupport   = querySwapChainSupport(device);
            swapChainAdequate = !swapChainSupport.formates.empty() && !swapChainSupport.presentModes.empty();
        }

        return deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU && deviceFeatures.geometryShader && indices.isComplete();
    }

    VulkanViewer::QueueFamilyIndices VulkanViewer::findQueueFamilies(VkPhysicalDevice device) {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount , nullptr);
        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount , queueFamilies.data());

        int i = 0;
        for ( const auto& queueFamily : queueFamilies ) {
            /// 有些种类的queue会有功能重叠
            /// 硬件设计决定的：许多现代 GPU 的硬件架构中，可能存在多个独立的队列家族，
            /// 它们各自提供相似或重叠的功能支持（例如，都支持图形和传输）。
            /// 这为不同类型的任务提供了灵活性，并允许同时处理多种任务而不产生瓶颈。   graphicsFamily 很可能与 presentFamily 是同一个index
            if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                indices.graphicsFamily = i;
            }

            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i , surface , &presentSupport);
            if (presentSupport) {
                indices.presentFamily = i;
            }

            if ( indices.isComplete() ) {
                break;
            }

            i++;
        }
        return indices;
    }

    void VulkanViewer::createSurface() {
        if(glfwCreateWindowSurface(instance, mpWindow , nullptr , &surface) != VK_SUCCESS){
            throw std::runtime_error("fail to creat surface");
        }
    }

    /// 仅仅是在
    bool VulkanViewer::checkDeviceExtensionSupport(VkPhysicalDevice device) {
        uint32_t extensionCount = 0;
        vkEnumerateDeviceExtensionProperties(device , nullptr, &extensionCount, nullptr);
        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(device , nullptr, &extensionCount, availableExtensions.data());

        /// TODO: 容器之间直接相互构造
        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());
        for ( const auto& extension : availableExtensions ) {
            requiredExtensions.erase(extension.extensionName); // 删除成功返回1 删除失败返回0
        }

        return requiredExtensions.empty();
    }

    /// 这里已经传入surface了，surface是通过窗口接口找到的。因此携带了像素信息
    SwapChainSupportDetails VulkanViewer::querySwapChainSupport(VkPhysicalDevice device) {
        SwapChainSupportDetails details;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);

        /// surfaces
        uint32_t formatCount;
        vkGetPhysicalDeviceSurfaceFormatsKHR(device , surface , &formatCount , nullptr);
        if ( formatCount!=0 ) {
            details.formates.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(device , surface , &formatCount ,  details.formates.data());
        }

        /// Swap-Chain present状态设置
        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device,surface,&presentModeCount,nullptr);

        if ( presentModeCount != 0 ) {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(device , surface , &presentModeCount , details.presentModes.data());
        }

        return details;
    }

    void VulkanViewer::createLogicalDevice() {
        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        /// 在这里把合适的 物理GPU的队列 抽离出作为构成logical 设备的素材 ,不止一个
        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        std::set<uint32_t> uniqueQueueFamilies = {indices.graphicsFamily.value(), indices.presentFamily.value()};

        float queuePriority = 1.0f;
        for ( uint32_t queueFamily : uniqueQueueFamilies ) {
            VkDeviceQueueCreateInfo queueCreateInfo = {};
            queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueCreateInfo.queueFamilyIndex = queueFamily;      /// 获得GPU的queue Index 【在find physical device的时候就已经筛选完队列】
            queueCreateInfo.queueCount = 1;                      /// 从对应的QueueFamily中只构造一个 【】
            queueCreateInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueCreateInfo);
        }

        VkPhysicalDeviceFeatures deviceFeatures{};                              /// TODO: 这里还可以筛选一些特性

        /// 逻辑设备 =  A队列INFO(render) + B队列INFO(graphics)  + ...
        VkDeviceCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.pQueueCreateInfos = queueCreateInfos.data();                 /// 指向第一个v
        // 各个物理队列抽离出来构成 一个完整的logical队列
        createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
        createInfo.pEnabledFeatures = &deviceFeatures;

        /// 逻辑设备需要指定 特定的扩展(swapChain,将device的输出显示在window中) 以及 验证层
        /// Validataion
        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationlayers.size());
            createInfo.ppEnabledLayerNames = validationlayers.data();
        }else {
            createInfo.enabledLayerCount = 0;
        }

        /// 给逻辑设备开启swapchain，前提是硬件检查通过
        createInfo.enabledExtensionCount = deviceExtensions.size();
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();


        if(vkCreateDevice(physicalDevice , &createInfo , nullptr , &device) != VK_SUCCESS) {
            throw std::runtime_error("failed to create logical device!");
        }

        vkGetDeviceQueue( device , indices.graphicsFamily.value() , 0 , &graphicsQueue);
        vkGetDeviceQueue( device , indices.presentFamily.value() , 0 , &presentQueue);
    }

    void VulkanViewer::createSwapChain() {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);
        /// TODO: 从swap-chain支持的功能中筛选 , RT色彩格式+色彩空间+长宽【exten】
        VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formates);
        /// 选择缓冲模式
        VkPresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
        /// 获取Canvas尺寸
        VkExtent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

        /// 交换链的队列size？ 指硬件所能支持的RT个数
        uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
        /// 这里if只是检查，万一多余最大RT上限，就把值限制下去
        if ( swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount ) {
            imageCount = swapChainSupport.capabilities.maxImageCount;
        }

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = surface;
        createInfo.minImageCount = imageCount;                          /// 交换链中至少有多少个图像，3重缓冲就设定为3
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;    /// TODO： 这个是干什么的？ 如果是双目的图像【vision pro？ 可以同时输出俩个RT】
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;    /// 决定交换链中的 image 状态（资源状态设置）
        createInfo.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;         /// 解锁帧数
        /// VK_IMAGE_USAGE_TRANSFER_DST_BIT 【把输出的RT重新 transfer 成输入的RT】
        //createInfo.oldSwapchain = VK_NULL_HANDLE; ///  传入old_SwapChain就可以在 swap-chain 运行中改变所使用的swap-chain

        /// 再次拿出 硬件队列
        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        uint32_t queueFamilyIndices[] = { indices.graphicsFamily.value( ) , indices.presentFamily.value( ) };

        ///  设定swap-chain中的image是否被 渲染queue 或 显示queue 独占。
        if ( indices.graphicsFamily != indices.presentFamily ) {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;   ///  Images can be used across multiple queue families without explicit ownership transfers.
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = queueFamilyIndices;
        }else {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
            createInfo.queueFamilyIndexCount = 0;      /// 如果image是队列独占，则不用设置与之共享的另一个queue的index
            createInfo.pQueueFamilyIndices = nullptr;
        }

        /// 这俩个设置比较迷
        createInfo.preTransform = swapChainSupport.capabilities.currentTransform;   /// 预变换，在最终输出到屏幕的时候会在做最后一次transform
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;              /// 如果有多的window{不是RT?}，他们之间是否用alpha通道值进行blend

        //createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;                           /// 如果有其他窗口系统遮挡pixel，则
        createInfo.oldSwapchain = VK_NULL_HANDLE;               /// 交换链 在窗口变化的时候会发生销毁与重建

        if ( vkCreateSwapchainKHR(device , &createInfo , nullptr , &swapChain) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create swapchain!");
        }

        vkGetSwapchainImagesKHR(device , swapChain , &imageCount ,  nullptr);
        swapChainImages.resize(imageCount);         /// imageCount = 3 [RT的个数]
        vkGetSwapchainImagesKHR(device , swapChain , &imageCount , swapChainImages.data());

        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;
    }

    VkSurfaceFormatKHR VulkanViewer::chooseSwapSurfaceFormat(const vector<VkSurfaceFormatKHR> &availableFormats) {
        for ( const auto& availableFormat : availableFormats ) {
            if ( availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR ) {
                return availableFormat;
            }
        }
        return availableFormats[0];
    }

    /// 选择三重缓冲类型
    VkPresentModeKHR VulkanViewer::chooseSwapPresentMode(const vector<VkPresentModeKHR> &availablePresentModes) {
        for ( const auto& availablePresentMode: availablePresentModes ) {
            if ( availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR ) {        /// 三重缓冲
                return availablePresentMode;
            }
        }
        return VK_PRESENT_MODE_FIFO_KHR;
    }

    VkExtent2D VulkanViewer::chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities) {
        if ( capabilities.currentExtent.width != UINT32_MAX ) {
            return capabilities.currentExtent;
        }else {
            int width, height;
            glfwGetFramebufferSize(mpWindow, &width, &height);      /// 通过glfw获取真实 窗口Size

            VkExtent2D actualExtent = {
                    static_cast<uint32_t>(width),
                    static_cast<uint32_t>(height)
            };

            actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
            return actualExtent;
        }
    }

    void VulkanViewer::createImageViews() {

        swapChainImageViews.resize(swapChainImages.size());
        for ( size_t i = 0 ; i<swapChainImages.size() ;i++) {
            VkImageViewCreateInfo createInfo{};
            createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            createInfo.image = swapChainImages[i];
            createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            createInfo.format = swapChainImageFormat;
            createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
            createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; /// 都只作为 color RT
            createInfo.subresourceRange.baseMipLevel = 0;           /// MipMap
            createInfo.subresourceRange.levelCount = 1;             ///
            createInfo.subresourceRange.baseArrayLayer = 0;         /// layer 双目设置
            createInfo.subresourceRange.layerCount = 1;

            if ( vkCreateImageView( device , &createInfo , nullptr
                                    , &swapChainImageViews[i]) != VK_SUCCESS ) {
                throw std::runtime_error("failed to create image views");
            }
        }
    }

    void VulkanViewer::createRenderPass() {

        // 【创建attachments】
        VkAttachmentDescription colorAttachments{};                 /// 创建一张RT{没有调用什么create？？？这是为何？}
        colorAttachments.format = swapChainImageFormat;             /// 设置RT的format，这个是人为控制的
        colorAttachments.samples = VK_SAMPLE_COUNT_1_BIT;           /// 暂时没有多重采样
        colorAttachments.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;      /// 在渲染之前对这个attachment做的操作
        colorAttachments.storeOp = VK_ATTACHMENT_STORE_OP_STORE;    /// 在渲染之后
        /// 【创建attachment】 attachment stencil设置
        colorAttachments.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachments.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachments.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;         ///  在Pass启动前，layout的布局如何
        colorAttachments.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;     ///  在Pass结束后，我们希望产生的image能够被交换链输出到窗口

        ///  【创建attachment Ref】 pass引用之前的attachment 、这里的description
        ///   仅仅使用来为 SwapChain中真是attachment设定操作的功能，不是attachment的对象
        ///   相当是对RT的引用， 可以创建多个Ref引用统一张attachment
        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0;                                             /// **最重要！这是一个index ，定位到他是哪一张RT。layout(location = 0) out vec4 outColor
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;          /// 描述上面这个attachment的布局(a color buffer)
        ///  【创建attachment】 subPass1 ，填装之前做好的attachment。 表示这个pass用的attachment
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;

        /// subPass dependency 【定义不同subpass之间的依赖关系】
        /// Render Dependency Graph {}
        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;            /// 本subPass所依赖的Pass，来自于外部，并不是本RenderPass中的东西。 srcSubpass 表示前置Pass是谁
        dependency.dstSubpass = 0;                              /// 哪些其他subpass在依赖本subpass {存储的是index}
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;        /// 等待SC把上一帧render到image之后
        dependency.srcAccessMask = 0;                                       ///Mask表示对资源的访问类型
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;        /// Stage阶段是什么意思
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;    ///Mask表示对资源的访问类型【这里感觉是对，依赖本subPass的对象设置的】

        /// RenderPass 构造
        VkRenderPassCreateInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = 1;                                             /// RT的个数在这里设置 与 RT类型必须强制匹配
        renderPassInfo.pAttachments = &colorAttachments;                                /// 无论几个Pass ， 他们进来与离开的时候操作均统一
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = 1;                                             /// 不同subpass之间切换需求
        renderPassInfo.pDependencies = &dependency;


        if ( vkCreateRenderPass(device , &renderPassInfo , nullptr , &renderPass) != VK_SUCCESS) {
            throw std::runtime_error("failed to create render pass");
        }
    }

    void VulkanViewer::createGraphicsPipline() {
        /// pipline装备 1.编译shader
        /// std::vector<char> vertexCode = readFile("/home/mikusama/Desktop/SLAM/ORB-SLAM2-Note/Shaders/VertexBufferUBO.spv");
        std::vector<char> vertexCode = readFile("/home/mikusama/Desktop/SLAM/ORB-SLAM2-Note/Shaders/VertexBufferUBO3D.spv");
        std::vector<char> fragCode   = readFile("/home/mikusama/Desktop/SLAM/ORB-SLAM2-Note/Shaders/frag.spv");
        VkShaderModule vertShaderModule = createShaderModule(vertexCode);
        VkShaderModule fragShaderModule = createShaderModule(fragCode);

        /// vertex shader创建
        VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
        vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertShaderStageInfo.module = vertShaderModule;
        vertShaderStageInfo.pName = "main";                     /// function to invoke

        /// fragment shader
        VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
        fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragShaderStageInfo.module = fragShaderModule;
        fragShaderStageInfo.pName = "main";

        /// 【Shader stages】
        VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo , fragShaderStageInfo};

        /// per-vertex  ||  per-instance 【 经典每个vertex携带的信息大小 + attribute pointer{属性偏移} 】
        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        auto bindingDescription =  Vertex_3D::getBindingDescription();
        auto attributeDescriptions = Vertex_3D::getAttributeDescriptions();

        vertexInputInfo.vertexBindingDescriptionCount = 1;              /// 只有一个绑定点[ 接受Position + Color 全部数据]
        vertexInputInfo.vertexAttributeDescriptionCount =  static_cast<uint32_t>( attributeDescriptions.size() ); /// 属性种类个数
        vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

        /// Input Assembly。 图元装配阶段
        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        //inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;   /// every 3 vertices without reuse
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        /// Viewport
        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = (float)swapChainExtent.width;
        viewport.height = (float)swapChainExtent.height;
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;

        /// Scissor
        VkRect2D scissor{};
        scissor.offset = { 0 ,0};
        scissor.extent = swapChainExtent;


        /// 【Fixed-function state】 FIX stages ,
        /// Dynamic阶段的目的是为了 可以在每个pass中单独设定确保
        std::vector<VkDynamicState> dynamicStates = {
                VK_DYNAMIC_STATE_VIEWPORT,
                VK_DYNAMIC_STATE_SCISSOR
        };
        /// 管线中动态可变的部分
        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        /// 多重RT的开启发生在这里
        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.pViewports = &viewport;
        viewportState.scissorCount = 1;
        viewportState.pScissors = &scissor;

        /// Rasterizer _ 三角片
        /*
        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;                 /// 如果为true则会将超过近远平面的 fragments 固定到近远平面
        rasterizer.rasterizerDiscardEnable = VK_FALSE;          /// 如果设为true 则所有输出都不会显示在 framebuffer中
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;          /// VK_POLYGON_MODE_FILL  || VK_POLYGON_MODE_LINE || VK_POLYGON_MODE_POINT
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;       /// 逆时针为正    /// 顺时针连接的vertex为正面
        rasterizer.depthBiasEnable = VK_FALSE;                  /// 深度偏移主要 用于shadow mapping
        rasterizer.depthBiasConstantFactor = 0.0f;
        rasterizer.depthBiasClamp = 0.0f;
        rasterizer.depthBiasSlopeFactor = 0.0f;
         */

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_POINT;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_FALSE;

        /// Multisampling 【抗锯齿设置】
        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_FALSE;
        /// TODO:
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        multisampling.minSampleShading = 1.0f; // Optional
        multisampling.pSampleMask = nullptr; // Optional
        multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
        multisampling.alphaToOneEnable = VK_FALSE; // Optional

        /// Depth & Stencil Test


        /// Color Blending 【一定会与FrameBuffer中的color做合并 , 1.按比例叠加或者按位运算组合】
        /// 可做成数组的形式 提供多个blend配置
        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT
                                              | VK_COLOR_COMPONENT_A_BIT;
        /// 分成alpha通道和color通道即可
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        /// Color Blending
        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;                     /// 是否启用第二个bitwise操作  MIX color + bitwise operation
        colorBlending.attachmentCount = 1;                          /// 如果有多个RT，这里blend的个数也需要给定
        colorBlending.pAttachments = &colorBlendAttachment;

        /// 【Layout , 描述shader的参数 uniforms】
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;                  /// 通过SetLayout 【设置全局变量 uniforms ，为vertex shader 也为 framentshader】
        pipelineLayoutInfo.pushConstantRangeCount = 0;
        pipelineLayoutInfo.pPushConstantRanges = nullptr;

        if ( vkCreatePipelineLayout(device , &pipelineLayoutInfo , nullptr , &pipelineLayout) ) {
            throw std::runtime_error("failed to create pipeline layout!");
        }

        /// 真正创建Pipline的地方
        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;      /// VBO 和 IBO设置
        pipelineInfo.pInputAssemblyState = &inputAssembly;      /// 图元装配
        pipelineInfo.pViewportState = &viewportState;           /// RT
        pipelineInfo.pRasterizationState = &rasterizer;         /// 光栅化器
        pipelineInfo.pMultisampleState = &multisampling;        /// 采样器设置
        pipelineInfo.pDepthStencilState = nullptr;              /// alpha test -> stencil test -> depth test
        pipelineInfo.pColorBlendState = &colorBlending;         /// 管线最终blend 设置
        pipelineInfo.pDynamicState = &dynamicState;             /// 动态状态设定
        pipelineInfo.layout = pipelineLayout;                   /// shader参数装配
        pipelineInfo.renderPass = renderPass;                   /// 本管线所需要
        pipelineInfo.subpass = 0;                               /// 子pass的index
        pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;       /// pipline继承
        pipelineInfo.basePipelineIndex = -1;

        if ( vkCreateGraphicsPipelines(device , VK_NULL_HANDLE , 1 ,&pipelineInfo , nullptr , &graphicsPipeline) ) {
            throw std::runtime_error("failed to create graphics pipeline!");
        }


        /// End of pipline
        vkDestroyShaderModule(device,vertShaderModule,nullptr);
        vkDestroyShaderModule(device,fragShaderModule,nullptr);
        return;
    }

    VkShaderModule VulkanViewer::createShaderModule(const vector<char> &code) {
        VkShaderModuleCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());  /// SPIR-V是一个32位对其的格式。

        VkShaderModule shaderModule;
        if ( vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create shader module");
        }
        return shaderModule;
    }

    /// 设置shader参数描述符。
    void VulkanViewer::createDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding uboLayoutBinding{};
        uboLayoutBinding.binding = 0;
        uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;            // 这里的 描述符layout 只是给uniform buffer用
        uboLayoutBinding.descriptorCount = 1;           /// Transform矩阵不只，只有一个。descriptors不只一个，可以是个数组 每一个骨骼动画的Transform都是单独的，所以需要提交的Translate也是不同 ，
        uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        uboLayoutBinding.pImmutableSamplers = nullptr;

        VkDescriptorSetLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &uboLayoutBinding;
        if ( vkCreateDescriptorSetLayout(device , &layoutInfo , nullptr , &descriptorSetLayout ) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create descriptor set layout");
        }
    }

    void VulkanViewer::createFramebuffers() {

        swapChainFramebuffers.resize(swapChainImageViews.size());
        for ( size_t i = 0; i<swapChainImageViews.size() ; i++) {
            /// 一个frame buffer可以同时有多个 attachments{本质上是image view}
            VkImageView attachments[] = {
                    swapChainImageViews[i]
            };

            VkFramebufferCreateInfo framebufferInfo{};
            framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            framebufferInfo.renderPass = renderPass;                ///  same number and type of attachments.
            framebufferInfo.attachmentCount = 1;                    ///  对应上面attachment的个数
            framebufferInfo.pAttachments = attachments;             ///
            framebufferInfo.width = swapChainExtent.width;
            framebufferInfo.height = swapChainExtent.height;
            framebufferInfo.layers = 1;

            if ( vkCreateFramebuffer(device , &framebufferInfo , nullptr , &swapChainFramebuffers[i] ) != VK_SUCCESS ) {
                throw std::runtime_error("failed to create framebuffer");
            }
        }

    }

    void VulkanViewer::createCommandPool() {
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);
        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;   /// CMD buffer都被单独记录，并且单独被重置(每一帧 = 一个command buffer ，每一帧都需要被重置)
        poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

        if (  vkCreateCommandPool(device , &poolInfo , nullptr , &commandPool) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create command pool!");
        }
    }

    void VulkanViewer::createVertexBuffer() {
        /// TODO： 这里设置缓冲区，用于batch Rendering
        VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

        VkBuffer stagingBuffer;
        VkDeviceMemory StagingBufferMemory;
        createBuffer( bufferSize , VK_BUFFER_USAGE_TRANSFER_SRC_BIT ,       /// 这里关键词是Source
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      stagingBuffer , StagingBufferMemory );

        void* data; ///  直接创建 Transfer 专用buffer , 在CPU上的staging buffer 映射到 CPU 上...？ 逆天
        vkMapMemory(device , StagingBufferMemory , 0 , bufferSize , 0 , &data );  /// flags没作用
        memcpy(data , vertices.data() , (size_t)bufferSize);
        vkUnmapMemory(device , StagingBufferMemory);

        createBuffer( bufferSize , VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
                , VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT  , vertexBuffer , vertexBufferMemory  );

        copyBuffer( stagingBuffer , vertexBuffer , bufferSize );        ///[这个操作发生在GPU上] 在GPU中的data之间传递数据

        /// 清理staging buffer
        vkDestroyBuffer(device , stagingBuffer , nullptr);              /// 这里如果要反复使用，就做成全局变量
        vkFreeMemory(device , StagingBufferMemory , nullptr);
    }


    /// 动态更新 VBO [从drawer中提取需要操作的data ] 、 不是增长而是每次都一整个操纵。每次重新创建就好...
    void VulkanViewer::updateMapPointStagingBuffer_V1(const std::vector<Vertex_3D>& newVertices) {
        VkDeviceSize newSize = sizeof(Vertex) * newVertices.size();

        if( newSize <= currentBufferSize ){
            void* data;
            /// TODO: 这里是直接映射出 GPU_local_Device??
            /// 还是先映射到staging上吧
            vkMapMemory(device , MapPointStagingBufferMemory , 0 , newSize,0,&data);
            memcpy( data , newVertices.data() , (size_t)newSize);
            vkUnmapMemory(device , MapPointStagingBufferMemory);
        }
        else{  /// 扩容 以及 第一次创建staging buffer
            currentBufferSize = newSize * 1.5;
            /// 清理VBO+暂存区
            if( vertexBuffer != VK_NULL_HANDLE ){
                //vkDeviceWaitIdle(device);
                vkQueueWaitIdle(graphicsQueue);  // ✅ 只等待 `graphicsQueue`，避免阻塞整个设备
                vkDestroyBuffer(device,vertexBuffer , nullptr);
                vkFreeMemory(device,vertexBufferMemory, nullptr);
            }

            if( MapPointStagingBuffer != VK_NULL_HANDLE ){
                vkDestroyBuffer(device,MapPointStagingBuffer , nullptr);
                vkFreeMemory(device,MapPointStagingBufferMemory, nullptr);
            }

            /// 创建新的 大容量暂存区
            createBuffer(currentBufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                         MapPointStagingBuffer, MapPointStagingBufferMemory);

            /// 复制data到新staging
            void* data;
            vkMapMemory(device , MapPointStagingBufferMemory , 0 , newSize,0,&data);
            memcpy( data , newVertices.data() , (size_t)newSize);
            vkUnmapMemory(device , MapPointStagingBufferMemory);
        }

        /// 创建新VBO
        createBuffer( currentBufferSize , VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
                , VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT  , vertexBuffer , vertexBufferMemory  );

        /// 将暂存区 -> 复制到真实VBO中
        copyBuffer(  MapPointStagingBuffer , vertexBuffer , currentBufferSize);
    }




    void VulkanViewer::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties,
                                    VkBuffer &buffer, VkDeviceMemory &bufferMemory) {
        /// 为GPU创建内存的信息 ，目的是获取VkMemoryRequirements
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if ( vkCreateBuffer(device , &bufferInfo , nullptr , &buffer) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create buffer");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device , buffer , &memRequirements);  /// memRequirements 所分配内存应具备的GPU属性[]

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType( memRequirements.memoryTypeBits , properties );      /// 获取查找的结果

        if ( vkAllocateMemory(device , &allocInfo , nullptr , &bufferMemory) != VK_SUCCESS ) {
            throw std::runtime_error("failed to allocate buffer memory");
        }

        vkBindBufferMemory(device , buffer , bufferMemory , 0); /// CPU侧 handle 与 真实GPU显存绑定  第四个参数需要设置内存对齐
        /// vkBindBufferMemory(device, newBuffer, previouslyAllocatedMemory, 0);
        // 动态资源分配的场景中，比如逐帧更新的统一缓冲区，你可以复用一块显存，而不是每次都重新分配。 重新绑定到新缓冲区
    }

    void VulkanViewer::copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(device , &allocInfo , &commandBuffer );
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;      ///命令缓冲区只会被提交一次到队列中，而不再被重新提交或重用

        vkBeginCommandBuffer(commandBuffer , &beginInfo);
        VkBufferCopy copyRegion{};
        copyRegion.srcOffset = 0;
        copyRegion.dstOffset = 0;
        copyRegion.size = size;
        vkCmdCopyBuffer( commandBuffer , srcBuffer , dstBuffer , 1 , &copyRegion ); /// 插入一条指令。 用于转移数据
        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit( graphicsQueue , 1 ,&submitInfo , VK_NULL_HANDLE );   /// 如果用fence, 就支持多个queue transfer
        vkQueueWaitIdle( graphicsQueue );   /// 登等待同步唤醒
        vkFreeCommandBuffers(device , commandPool , 1 , &commandBuffer );
    }

    uint32_t VulkanViewer::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        /// 内存类型1 内存类型2 .... 直接与count 相绑定关联 ,
        /// 1 << 0 = 0b0001，1 << 1 = 0b0010，1 << 2 = 0b0100 每次移动一位便可以查遍所有内存类型，index就是内存的ID
        for ( uint32_t i = 0 ; i<memProperties.memoryTypeCount; i++) {
            /// typeFilter & (1<<i) 筛选能支持 VBO 的GPU内存类型 { 硬件层面的支持 }

            /// 确保属性匹配 (memProperties.memoryTypes[i].propertyFlags & properties) == properties 。 该index 的 GPU内存的flag 是否满足 heap and properties of each type of memory
            /// 更进一步说的是 内存符合：TODO：VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT（GPU本地内存，性能高适合GPU直接访问）
            /// TODO：VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT（ CPU可访问 , 但是往往这种空间不是GPU读写时最优的选择）
            /// TODO：VK_MEMORY_PROPERTY_HOST_COHERENT_BIT（ CPU 写入数据后立即可见，无需手动刷新缓存 ）
            if ( typeFilter & (1<<i) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties ) {
                return i;
            }
        }

        throw std::runtime_error("failed to find memory type");
    }

    void VulkanViewer::createIndexBuffer() {

        VkDeviceSize  bufferSize = sizeof(indices[0]) * indices.size();
        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;
        createBuffer( bufferSize , VK_BUFFER_USAGE_TRANSFER_SRC_BIT ,
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      stagingBuffer, stagingBufferMemory );

        void* data;
        vkMapMemory(device , stagingBufferMemory , 0 , bufferSize , 0 , &data );    /// 到GPU上的data copy CPU上Buffer
        memcpy( data , indices.data() , (size_t)bufferSize );
        vkUnmapMemory(device , stagingBufferMemory);

        createBuffer( bufferSize , VK_BUFFER_USAGE_TRANSFER_DST_BIT| VK_BUFFER_USAGE_INDEX_BUFFER_BIT ,
                      VK_MEMORY_HEAP_DEVICE_LOCAL_BIT , indexBuffer , indexBufferMemory );
        copyBuffer( stagingBuffer , indexBuffer , bufferSize );

        vkDestroyBuffer(device , stagingBuffer , nullptr);
        vkFreeMemory(device , stagingBufferMemory , nullptr);

    }


    /// 同时多frame，分别在GPU中不同的位置,这里仅是开辟空间，并不填充数据
    void VulkanViewer::createUniformBuffers() {
        VkDeviceSize bufferSize = sizeof(UniformBufferObject);

        uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
        uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
        uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);

        for ( size_t i = 0; i < MAX_FRAMES_IN_FLIGHT ; i++ ) {
            createBuffer(bufferSize , VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT ,
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT|VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT
                    , uniformBuffers[i] ,  uniformBuffersMemory[i]);
            /// 映射出去的时候 直接用raw data完成
            vkMapMemory(device , uniformBuffersMemory[i], 0 , bufferSize , 0 , &uniformBuffersMapped[i] );
        }
    }

    uint32_t currentFrame = 0;
    void VulkanViewer::drawFrame() {
        ImGui::Render();
        /// 等待GPU处理完成(CPU GPU同步)  Hint：同步完成后才可以acquireImage
        vkWaitForFences( device , 1 , &inFlightFences[currentFrame] , VK_TRUE , UINT64_MAX ); /// 监听到已经发出

        uint32_t imageIndex = 0;
        VkResult result = vkAcquireNextImageKHR( device , swapChain , UINT64_MAX ,
                                                 imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE
                                                 , &imageIndex);
        if ( result == VK_ERROR_OUT_OF_DATE_KHR  ) {
            recreateSwapChain();
            return;
        }else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR ) {
            throw std::runtime_error("failed to acquire swap chain image");
        }
        vkResetFences( device , 1 , &inFlightFences[currentFrame]); ///重置为 unsigned ，表示CPU正在等待GPU返回。 TODO: 放在交换链重置之后？

        /// Uniform Buffer 提交
        updateUniformBuffer(currentFrame);

        /// 指令录制
        vkResetCommandBuffer( commandBuffers[currentFrame] , 0 );
        recordCommandBuffer( commandBuffers[currentFrame] , imageIndex); /// **** 上传在这

        ///指令提交
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        /// before execution begins 需要等待的东西
        VkSemaphore waitSemaphores[] = { imageAvailableSemaphores[currentFrame] };      /// 再次获取交换链图像资源锁
        VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };  ///[细节到某一个阶段的就绪状态] pipeline stage【index与需要等待的信号量一致】
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = waitSemaphores;
        submitInfo.pWaitDstStageMask = waitStages;      /// 颜色输出阶段，需要等待imageAvailable，也就是等待swapchain中有可用image之后，才可以继续向它draw
        submitInfo.commandBufferCount = 1 ;
        submitInfo.pCommandBuffers = &(commandBuffers[currentFrame]) ;   /// 具体提交的指令参数

        VkSemaphore signalSemaphores[] = { renderFinishedSemaphores[currentFrame] };       /// 渲染指令执行完成后，触发的信号量
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = signalSemaphores;

        ///  command buffer 完成输出后 inFlightFence 标志将被激活 ， 表示drawcall调用完毕，绘制结束。
        ///  CPU没有阻塞，而是在本函数开头位置才Acquire的时候才会。
        if ( vkQueueSubmit(graphicsQueue ,  1 , &submitInfo , inFlightFences[currentFrame] ) != VK_SUCCESS ) {     /// 提交
            throw std::runtime_error("failed to submit command buffer draw command buffer");
        }

        /// 显示阶段 ，
        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = signalSemaphores;     /// 这里指定在提交给显示队列之前，等待哪个渲染信号的结束


        VkSwapchainKHR swapChains[] = { swapChain };        /// 比如vison pro这种东西，它一次性出现左右两个图像
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = swapChains;
        presentInfo.pImageIndices = &imageIndex;            /// 指定显示队列+需要显示的图片
        presentInfo.pResults = nullptr;

        result = vkQueuePresentKHR(presentQueue, &presentInfo);      ///  error handling
        if ( result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized ) {
            framebufferResized = false;
            recreateSwapChain();
        }else if ( result != VK_SUCCESS ) {
            throw std::runtime_error("failed to present swap chain image!");
        }

        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;

    }

    void VulkanViewer::recreateSwapChain() {
        /// 最小化窗口情况时，一直卡住轮寻。
        int width = 0 , height = 0;
        glfwGetFramebufferSize( mpWindow , &width , &height );
        while ( width == 0 || height == 0) {
            glfwGetFramebufferSize(mpWindow , &width , &height );
            glfwWaitEvents();
        }

        /// 等待所有队列全部操作完成。 确保设备空闲
        /// 设备进行修改、销毁资源或其他可能影响设备状态的操作时使用
        vkDeviceWaitIdle(device);
        cleanupSwapChain();
        /// 重新构造 SwapChain
        createSwapChain();      ///自动匹配新窗口Size
        createImageViews();
        createFramebuffers();
    }

    /// TODO: 转移为操作摄像机
    void VulkanViewer::updateUniformBuffer(uint32_t currentImage) {
        static auto startTime = std::chrono::high_resolution_clock::now();
        /// 以秒为单位
        auto currentTime = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration<float , std::chrono::seconds::period>(currentTime - startTime).count();


        UniformBufferObject ubo{};
        //螺旋观察
        //ubo.model = glm::rotate( glm::mat4(1.0f) , time*glm::radians(90.0f)*0.01f , glm::vec3(0.0f , 0.0f, 1.0f) );
        ubo.model = glm::mat4(1.0f);

        /// 获取摄像机参数， 修改View矩阵
        glm::mat4 cameraView(1.0f);
        cameraView = mpcamera->GetViewMatrix();
        //ubo.view = glm::lookAt( glm::vec3( 2.0f , 2.0f ,2.0f ) ,glm::vec3(0.0f , 0.0f , 0.0f) ,glm::vec3(0.0f, 0.0f, 1.0f));
        ubo.view = cameraView;

        ubo.proj = glm::perspective(glm::radians(45.0f) ,
                                    swapChainExtent.width/(float)swapChainExtent.height , 0.1f , 100.0f);

        ubo.proj[1][1] *= -1;           /// scaleing Y轴缩放-1 颠倒图形

        /// Inverse? TODO: 为什么需要？
        memcpy(uniformBuffersMapped[currentImage] , &ubo, sizeof(ubo));
    }

    void VulkanViewer::createCommandBuffers() {

        commandBuffers.resize( MAX_FRAMES_IN_FLIGHT );
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = commandPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = (uint32_t)commandBuffers.size();

        if ( vkAllocateCommandBuffers(device , &allocInfo , commandBuffers.data() ) != VK_SUCCESS ) {
            throw std::runtime_error("failed to allocate command buffers");
        }

    }


    /// 创建GPU之间 、 GPU与CPU之间的同步操作
    void VulkanViewer::createSyncObjects() {

        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

        VkSemaphoreCreateInfo semaphoreInfo{};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        /// CPU 与 GPU同步
        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;     /// 第一次创建时，保证fence不影响CPU同步, 即让CPU确认已经进入就绪状态，可以开始提交指令给GPU

        for ( int i = 0 ; i<MAX_FRAMES_IN_FLIGHT ; ++i) {
            if ( vkCreateSemaphore(device , &semaphoreInfo , nullptr , &imageAvailableSemaphores[i] ) != VK_SUCCESS
                 || vkCreateSemaphore(device , &semaphoreInfo , nullptr , &renderFinishedSemaphores[i] ) != VK_SUCCESS
                 || vkCreateFence(device , &fenceInfo , nullptr , &inFlightFences[i] ) != VK_SUCCESS ) {
                throw std::runtime_error("failed to create semaphores");
            }
        }

    }

    void VulkanViewer::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex) {
        /// createInfo 主要用于设置 buffer的作用
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = 0;
        beginInfo.pInheritanceInfo = nullptr;

        /// vkBeginCommandBuffer：每次调用时，都会自动重置commandBuffer
        if ( vkBeginCommandBuffer(commandBuffer , &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording command buffer!");
        }

        /// renderPassBegin
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = renderPass;                             /// 对哪个RnederPass操作
        renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];     /// renderPass中的哪个 FrameBuffer, FrameBuffer中又包含多个RT[attachment] 我该如何
        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = swapChainExtent;

        VkClearValue clearColor = { {{0.0f, 0.0f, 0.0f, 1.0f}} };
        renderPassInfo.clearValueCount = 1; /// 为attachment clear and load阶段设置操作 VK_ATTACHMENT_LOAD_OP_CLEAR
        renderPassInfo.pClearValues = &clearColor;

        /// 开启录制
        vkCmdBeginRenderPass( commandBuffer , &renderPassInfo , VK_SUBPASS_CONTENTS_INLINE );       ///设定fragment shader的Attachment

            // 之前创建（设定好）好的管线，接受的指令是哪些
            vkCmdBindPipeline( commandBuffer,  VK_PIPELINE_BIND_POINT_GRAPHICS , graphicsPipeline );    /// pipeline中 viewport以及scissor

            VkViewport viewport{};
            viewport.x = 0.0f;
            viewport.y = 0.0f;
            viewport.width = (float)swapChainExtent.width;
            viewport.height = (float)swapChainExtent.height;
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;
            vkCmdSetViewport(commandBuffer, 0, 1, &viewport);   /// 第二个参数表示RT的索引

            /// 像是二次裁切
            VkRect2D scissor{};
            scissor.offset = {0, 0};
            scissor.extent = swapChainExtent;
            vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

            /// Draw call 准备VBO
            std::vector<Vertex_3D> vertices;
            ConstructVertex(mpMapDrawer , vertices);
            updateMapPointStagingBuffer_V1(vertices); ///


            VkBuffer vertexBuffers[] = {vertexBuffer};      /// TODO：这里如果有新的帧加入，我得更新才是，一直用init的时候的数据，就没有
            VkDeviceSize offsets[] = {0};

            /// 绑定静态数据（共享）
            vkCmdBindVertexBuffers( commandBuffer , 0 , 1 , vertexBuffers , offsets );      /// 第二个参数，本质上也就是个offset，如果有额外的buffer，他才会需要
            //vkCmdBindIndexBuffer( commandBuffer , indexBuffer , 0, VK_INDEX_TYPE_UINT16 );
            /// 绑定具体的UBO 状态 到pipline对应 layout上
            /// 需要槽位对其
            vkCmdBindDescriptorSets( commandBuffer  ,  VK_PIPELINE_BIND_POINT_GRAPHICS ,
                                     pipelineLayout , 0 ,
                                     1 , &descriptorSets[currentFrame] ,
                                     0 , nullptr);

            /// 绑定动态数据（每一帧都需要更新， UBO中的data，全局变量）
            vkCmdDraw(commandBuffer , static_cast<uint32_t>(vertices.size()), 1, 0, 0);
            //vkCmdDrawIndexed( commandBuffer , static_cast<uint32_t>(indices.size()) , 1, 0, 0 ,0);
            ImDrawData* draw_data = ImGui::GetDrawData();
            ImGui_ImplVulkan_RenderDrawData(draw_data , commandBuffer);
        vkCmdEndRenderPass(commandBuffer);

        ///TODO：还可额外增加Pass

        if ( vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to record command buffer!");
        }
    }

    void VulkanViewer::createDescriptorPool() {
        VkDescriptorPoolSize poolSize{};
        poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        poolSize.descriptorCount = static_cast<uint32_t>( MAX_FRAMES_IN_FLIGHT );        /// 每一并行帧 都是独立的descriptor

        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;                                   ///
        poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

        if( vkCreateDescriptorPool(device , &poolInfo , nullptr , &descriptorPool ) != VK_SUCCESS ) {
            throw std::runtime_error("failed to create descriptor pool");
        }
    }

    void VulkanViewer::createDescriptorSets() {

        /// 1. 在pool中分配Sets [descriptorSetLayout 在初始化的时候就已经确定，本质上是为了与pipline的设定一致，所以直接用现成的]
        std::vector<VkDescriptorSetLayout> Layouts(MAX_FRAMES_IN_FLIGHT , descriptorSetLayout);
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = descriptorPool;
        allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        allocInfo.pSetLayouts = Layouts.data();         /// 需要与std::vector<VkDescriptorSet> descriptorSets; Size一致

        /// 这里初始化的 set是空白的，没有被设定
        descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
        if ( vkAllocateDescriptorSets(device , &allocInfo , descriptorSets.data()) != VK_SUCCESS ) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }

        /// 必须利用VkWriteDescriptorSet 来写入格式
        for ( size_t i = 0 ; i<MAX_FRAMES_IN_FLIGHT ; ++i ) {
            VkDescriptorBufferInfo bufferInfo{};
            bufferInfo.buffer = uniformBuffers[i];              /// 指定数据大小
            bufferInfo.offset = 0;
            bufferInfo.range = sizeof(UniformBufferObject);

            VkWriteDescriptorSet descriptorWrite{};             /// VkWriteDescriptorSet
            descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrite.dstSet = descriptorSets[i];         /// 把uniformBuffers【i】的data存入dstSet中 ！！！最后update更新的东西就是他

            descriptorWrite.dstBinding = 0;                     /// shader中 layout(binding  = 0) 的序号
            descriptorWrite.dstArrayElement = 0;                /// descriptors can be arrays，这里给出array的起点index
            descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            descriptorWrite.descriptorCount = 1;                ///  根据刚刚给出的开始index，设置update几个descriptor{骨骼动画存在多个 Transform}
            descriptorWrite.pBufferInfo = &bufferInfo;
            descriptorWrite.pImageInfo = nullptr;           /// descriptors that refer to image data [shader参数是Iamge data，我可以理解为RT吗]
            descriptorWrite.pTexelBufferView = nullptr;     /// descriptors that refer to buffer views [ shader 参数是 其他buffer ]
            /// 描述符 只有read & write 本质上用一个接口就能搞定
            vkUpdateDescriptorSets(device,  1 , &descriptorWrite , 0 , nullptr );
        }

    }

    void VulkanViewer::cleanupSwapChain() {
        for ( auto framebuffer : swapChainFramebuffers ) {
            vkDestroyFramebuffer(device, framebuffer, nullptr);
        }

        for ( auto imageView : swapChainImageViews ) {
            vkDestroyImageView(device ,imageView , nullptr);
        }

        vkDestroySwapchainKHR(device , swapChain ,nullptr);
    }

    ///TODO
    void VulkanViewer::ConstructVertex(MapDrawer *Drawer, vector<Vertex_3D> &dst) {

        /// 获取数量，reserve dst
        const vector<MapPoint*> &vpMPs = Drawer->mpMap->GetAllMapPoints();
        /// Ref查重
        const vector<MapPoint*> &vpRefMPs = Drawer->mpMap->GetReferenceMapPoints();
        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty())
            return;

        /// 构造所有地图点
        //dst.resize(vpMPs.size()+spRefMPs.size() );
        dst.resize(vpMPs.size());

        size_t i = 0;
        for( ; i<vpMPs.size() ; ++i){
            //if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            //    continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            dst[i] = Vertex_3D{
                glm::vec3( pos.at<float>(0) , pos.at<float>(1) , pos.at<float>(2)),
                glm::vec3(1.0f, 0.0f, 0.0f)
            };
        }

        /*
        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            dst[i] = Vertex_3D{
                    glm::vec3( pos.at<float>(0) , pos.at<float>(1) , pos.at<float>(2)),
                    glm::vec3(1.0f, 0.0f, 0.0f)
            };
        }*/

    }

    void VulkanViewer::InitImGui() {

        VkDescriptorPoolSize pool_sizes[] =
                {
                        { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
                        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
                        { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
                        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
                        { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
                        { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
                        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
                        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
                        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
                        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
                        { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
                };

        VkDescriptorPoolCreateInfo pool_info = {};
        pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        pool_info.maxSets = 1000;
        pool_info.poolSizeCount = std::size(pool_sizes);
        pool_info.pPoolSizes = pool_sizes;

        VkDescriptorPool imguiPool;

        if(vkCreateDescriptorPool(device, &pool_info, nullptr, &imguiPool)){
            throw std::runtime_error("failed to create descriptor pool");
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        ImGui_ImplGlfw_InitForVulkan(mpWindow, true);
        ImGui_ImplVulkan_InitInfo init_info = {};
        init_info.Instance = instance;
        init_info.PhysicalDevice = physicalDevice;
        init_info.Device = device;
        init_info.Queue = graphicsQueue;
        init_info.DescriptorPool = imguiPool;
        init_info.RenderPass = renderPass;
        init_info.Subpass = 0;
        init_info.MinImageCount = 3;
        init_info.ImageCount = 3;
        init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        ImGui_ImplVulkan_Init(&init_info);

    }

    void VulkanViewer::HandleMouse(double xposIn, double yposIn) {
        if (!rightMousePressed) return;

        float xpos = static_cast<float> (xposIn);
        float ypos = static_cast<float> (yposIn);
        if(firstMouse)
        {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;       ///  y从上到下减少

        lastX = xpos;
        lastY = ypos;
        mpcamera->processRoate( xoffset, yoffset );
    }

    void VulkanViewer::HandleMouseButton(int button, int action) {
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (action == GLFW_PRESS) {
                rightMousePressed = true;
                firstMouse= true;
            } else if (action == GLFW_RELEASE) {
                rightMousePressed = false;
            }
        }
    }

    bool VulkanViewer::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }


    void VulkanViewer::DrawFPSGraph(float deltaTime) {
        static bool animate = true;
        ImGui::Checkbox("Animate", &animate);

        static float values[90] = {};
        static int values_offset = 0;
        static double refresh_time = 0.0;
        if (!animate || refresh_time == 0.0)
            refresh_time = ImGui::GetTime();
        while (refresh_time < ImGui::GetTime()) // Create data at fixed 60 Hz rate for the demo
        {
            //static float phase = 0.0f;
            values[values_offset] = deltaTime*1000.0f;
            values_offset = (values_offset + 1) % IM_ARRAYSIZE(values);
            //phase += 0.10f * values_offset;
            refresh_time += 1.0f / 60.0f;
        }

        // Plots can display overlay texts
        // (in this example, we will display an average value)
        {
            char overlay[32];
            ImGuiIO& io = ImGui::GetIO();
            sprintf(overlay, "avg Frame %f", io.Framerate);
            ImGui::PlotLines("Lines", values, IM_ARRAYSIZE(values), values_offset, overlay,
                             0.0f, 20.0f, ImVec2(0, 80.0f));
        }
    }


    void VulkanViewer::renderPerformance() {
        /// 准备
        ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 800), ImGuiCond_FirstUseEver);
        ImGui::Begin("Performance & MapInfo");
        ImGuiIO& io = ImGui::GetIO();
        float currentTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentTime - lastTime;
        frameCount++;
        fpsAccumulator += 1.0f / deltaTime; // 计算当前帧的FPS

        if (frameCount >= 500) { // 每 500 帧更新一次 FPS 统计过去500帧的操作间隔
            fps = fpsAccumulator / frameCount; // 计算平均 FPS
            fpsAccumulator = 0.0f; // 归零
            frameCount = 0;
        }

        // fps = 1.0f / (currentTime - lastTime);
        lastTime = currentTime;
        ImGui::SetWindowFontScale(1.5f);  // 放大 1.5 倍
        ImGui::Text("last 500 Frames AVG FPS  : %.1f", fps);
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);


        ImGui::Text("Dataset: HM-03");
        ImGui::Text("MapPoints Count: %.1zu m", mpMapDrawer->mpMap->GetAllMapPoints().size());
        ImGui::Text("Key Frames Count: %lu",  mpMapDrawer->mpMap->KeyFramesInMap());

        DrawFPSGraph(deltaTime);

        ImGui::SetWindowFontScale(1.0f);  // 恢复默认缩放
        ImGui::End();
    }


}

