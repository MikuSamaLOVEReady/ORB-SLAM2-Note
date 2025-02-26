//
// Created by mikusama on 2/6/25.
//

#ifndef VULKANVIEWER_H
#define VULKANVIEWER_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"
#include "VulkanCamera.h"

#include "System.h"
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

#include <mutex>

const int MAX_FRAMES_IN_FLIGHT = 2; /// 最高可以同时多少个帧在渲染

namespace ORB_SLAM2
{

  class System;
  class MapDrawer;

/// 对物理设备扩展检查，SwapChain
    const std::vector<const char*> deviceExtensions = {
            VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

/// 交换链
    struct SwapChainSupportDetails {
        VkSurfaceCapabilitiesKHR capabilities;
        std::vector<VkSurfaceFormatKHR> formates;
        std::vector<VkPresentModeKHR> presentModes;
    };

    /// 读取编译
    static std::vector<char> readFile(const char* filename) {
        /// 表示以二进制模式打开文件，并且文件指针定位到文件的末尾。
        /// 使用ate的好处就在于，能够直接获取文件size
        std::ifstream file(filename , std::ios::ate | std::ios::binary);
        if ( !file.is_open() ) {
            throw std::runtime_error("faile to open file");
        }
        size_t fileSize = (size_t)file.tellg(); /// 【文件开头】到【当前文件指针位置】的字节数 1char =  1byte { std::byte }
        std::vector<char> buffer(fileSize);
        /// 再从头读取
        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();

        return std::move(buffer);
    }


    /// VertexBuffer Object
    struct Vertex {
        glm::vec2 Position;
        glm::vec3 Color;

        /// VkVertexInputBindingDescription： 定义一片在GPU上的内存
        static VkVertexInputBindingDescription getBindingDescription() {
            /// 描述GPU如何理解CPU端传来的data
            VkVertexInputBindingDescription bindingDescription{};
            bindingDescription.binding = 0;  ///但它 主要与 layout(location = X) 搭配使用，以正确匹配 CPU 端的顶点数据和 GPU 端的着色器输入。 指定CPU端的资源绑定点{GPU上内存的index}
            bindingDescription.stride = sizeof(Vertex); /// 绑定点处数据的大小
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            return bindingDescription;
        }

        /// 解释 VertexBuffer 布局。 传入的是一片data，其中同时包含了色彩和位置，需要在该内存中分别解释
        static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions() {
            std::array<VkVertexInputAttributeDescription , 2> attributeDescriptions{};
            attributeDescriptions[0].binding = 0;                               // 选择解释哪一片缓冲区{在刚才binding Description的位置定义的一个index}
            attributeDescriptions[0].location = 0;                              /// 与 shader中的 location  = 0 完全对应
            attributeDescriptions[0].format = VK_FORMAT_R32G32_SFLOAT;          /// 32+32
            attributeDescriptions[0].offset = offsetof(Vertex, Position);       /// 位置偏移

            /// 设定shader location节点 +
            attributeDescriptions[1].binding = 0;
            attributeDescriptions[1].location = 1;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(Vertex, Color);
            return attributeDescriptions;
        };
    };

    /// VertexBuffer Object 3D
    struct Vertex_3D {
        glm::vec3 Position;
        glm::vec3 Color;

        /// VkVertexInputBindingDescription： 定义一片在GPU上的内存
        static VkVertexInputBindingDescription getBindingDescription() {
            /// 描述GPU如何理解CPU端传来的data
            VkVertexInputBindingDescription bindingDescription{};
            bindingDescription.binding = 0;  ///但它 主要与 layout(location = X) 搭配使用，以正确匹配 CPU 端的顶点数据和 GPU 端的着色器输入。 指定CPU端的资源绑定点{GPU上内存的index}
            bindingDescription.stride = sizeof(Vertex_3D); /// 绑定点处数据的大小
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            return bindingDescription;
        }

        /// 解释 VertexBuffer 布局。 传入的是一片data，其中同时包含了色彩和位置，需要在该内存中分别解释
        static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions() {
            std::array<VkVertexInputAttributeDescription , 2> attributeDescriptions{};
            attributeDescriptions[0].binding = 0;                               // 选择解释哪一片缓冲区{在刚才binding Description的位置定义的一个index}
            attributeDescriptions[0].location = 0;                              /// 与 shader中的 location  = 0 完全对应
            attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;          /// 32+32
            attributeDescriptions[0].offset = offsetof(Vertex_3D, Position);       /// 位置偏移

            /// 设定shader location节点 +
            attributeDescriptions[1].binding = 0;
            attributeDescriptions[1].location = 1;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(Vertex_3D, Color);
            return attributeDescriptions;
        };
    };

    //TODO: 静态 Vertices
    const std::vector<Vertex> vertices = {
            {{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
            {{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
            {{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
            {{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}}
    };


    // TODO: 静态 indices //SLAM 不一定需要
    const std::vector<uint16_t> indices = {
            0,1,2,2,3,0
    };

    struct UniformBufferObject {
        glm::mat4 model;
        glm::mat4 view;
        glm::mat4 proj;
    };


    class VulkanViewer{
    public:
      VulkanViewer(System *pSystem , MapDrawer* mpdrawer);
      void Run();

      void RequestFinish();
      bool CheckFinish();
      bool isFinished();

      uint32_t WIDTH = 800;
      uint32_t HEIGHT = 600;
      bool firstMouse = true;
      float lastX =  WIDTH / 2.0f;
      float lastY = HEIGHT / 2.0f;
      void HandleMouse(double xposIn, double yposIn);

      /// 窗口回调
      static void mouse_callback(GLFWwindow* window , double xposIn , double yposIn)
      {
          VulkanViewer* viewer = static_cast<VulkanViewer*>(glfwGetWindowUserPointer(window)); // ✅ 取回 VulkanViewer 实例
          if (viewer) {
              viewer->HandleMouse(xposIn ,yposIn); // ✅ 调用成员函数
          }
      };

      static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
      {
          VulkanViewer* viewer = static_cast<VulkanViewer*>(glfwGetWindowUserPointer(window));
          if (viewer) {
              viewer->HandleMouseButton(button, action);
          }
      }


    protected:

    private:
      bool Stop();
      void InitWindow();

      bool InitVulkan();
        bool createInstance();
        void createSurface();
        void pickPhysicalDevice();
        /// 根据GPU特性选择 device
        bool isDeviceSuitable(VkPhysicalDevice device);
        /// 物理设备支持的queue family与
        struct QueueFamilyIndices {
            std::optional<uint32_t> graphicsFamily;
            std::optional<uint32_t> presentFamily;
            bool isComplete() {
                return graphicsFamily.has_value() && presentFamily.has_value();
            }
        };
        QueueFamilyIndices findQueueFamilies( VkPhysicalDevice device );
        bool checkDeviceExtensionSupport(VkPhysicalDevice device);
        SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);
        void createLogicalDevice();
        /// 交换链
        void createSwapChain();
        /// RT色彩格式+色彩空间+
        VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
        /// 选择缓冲模式
        VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
        ///
        VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
        void createImageViews(); /// 抽象交换链中的Image对象
        void createRenderPass();
         /// TODO： void createDescriptorSetLayout(); unnifom buffer 为Shader传递火速据
        void createDescriptorSetLayout();       /// 在
        void createGraphicsPipline();
        VkShaderModule createShaderModule(const std::vector<char>& code);
        void createFramebuffers();
        void createCommandPool();
        void createVertexBuffer();      /// TODO： 仅做静态操作使用
        void createBuffer( VkDeviceSize size , VkBufferUsageFlags usage,
                           VkMemoryPropertyFlags properties , VkBuffer& buffer , VkDeviceMemory& bufferMemory );

        void copyBuffer( VkBuffer srcBuffer , VkBuffer dstBuffer ,  VkDeviceSize size );
        uint32_t findMemoryType(uint32_t typeFilter , VkMemoryPropertyFlags properties);
        void createIndexBuffer();       /// TODO：idnex buffer 不确定要不要
        void createUniformBuffers();
        void createDescriptorPool();
        void createDescriptorSets();
        /// 指令缓冲区构建
        void createCommandBuffers();
        void createSyncObjects();

        /// 窗体大小变化，交换链重构
        void recreateSwapChain();
        void cleanupSwapChain();        /// 交换链重构

        /// vulkan缓冲区

      void InitImGui();             /// UI初始化 TODO:

      void mainLoop();
          void drawFrame();
              void updateUniformBuffer( uint32_t currentImage ); /// 处理输入
              void recordCommandBuffer( VkCommandBuffer commandBuffer , uint32_t imageIndex);
              /// 地图点 vertex staging buffer。 需要预先分配
              void updateMapPointStagingBuffer_V1(const std::vector<Vertex_3D>& newVertices);
              void ConstructVertex( MapDrawer* Drawer , std::vector<Vertex_3D>& dst);



      void cleanup();

      //// SLAM变量
      System* mpSystem;
      std::mutex mMutexFinish;      /// TODO: init
      bool mbFinishRequested;       /// 由逻辑线程发出请求，渲染线程
      bool mbFinished;              /// 表示当前VkThread的状态 TODO
      /// 获取Drawer中保存的data
      MapDrawer* mpMapDrawer;
      Camera* mpcamera;

      /// GLFW 窗口系统
      GLFWwindow* mpWindow;
      bool framebufferResized = false;
      /// 窗口输入操作
      float detalTime = 0.0f;
      float lastFrame = 0.0f;


      /// Vulkan变量
      VkInstance instance ;
      VkDebugUtilsMessengerEXT debugMessenger;
      VkSurfaceKHR surface = 0;
      VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
      VkDevice device ;            /// logical device
      VkQueue graphicsQueue ;      /// 为后续
      VkQueue presentQueue ;
      VkSwapchainKHR swapChain ;
      std::vector<VkImage> swapChainImages;   /// 交换链提供画布，并给予这些对象对应属性
      VkFormat swapChainImageFormat;          /// 交换链根据 GPU 与 窗口缓冲筛选后，所确定的格式
      VkExtent2D swapChainExtent;             /// 尺寸
      std::vector<VkImageView> swapChainImageViews;   /// 像是交换链的读取器
      VkRenderPass renderPass;
      VkDescriptorSetLayout descriptorSetLayout;        /// 在init的时候 UBO的格式就已经可以确定 。这个Layout是用于pipline的绑定 后续的descriptor set 使用与运行时承载数据
      VkPipelineLayout pipelineLayout;
      VkPipeline graphicsPipeline;
      std::vector<VkFramebuffer> swapChainFramebuffers;
      VkCommandPool commandPool;
      VkBuffer vertexBuffer = VK_NULL_HANDLE;                /// 类似一个hanlder，本质是底层真实的引用
      VkDeviceMemory vertexBufferMemory = VK_NULL_HANDLE;    /// 真正创建的GPU显存
      /// Index Buffer
      VkBuffer indexBuffer;
      VkDeviceMemory indexBufferMemory;
      /// 运行时 uniform buffer
      std::vector<VkBuffer> uniformBuffers;               /// handle
      std::vector<VkDeviceMemory> uniformBuffersMemory;   /// 实际内存
      std::vector<void*> uniformBuffersMapped;            /// CPU端可以操纵的DATA. 能直接写入的部分
      VkDescriptorPool descriptorPool;

        /// 同步使用
      std::vector<VkFence> inFlightFences;                  /// Fence用于 CPU与GPU的同步
      std::vector<VkSemaphore> imageAvailableSemaphores;    /// Semaphores 用于GPU队列之间的同步
      /// 多帧command buffer
      std::vector<VkCommandBuffer> commandBuffers;          /// 每个独立帧，拥有完整一套command buffer
      std::vector<VkSemaphore> renderFinishedSemaphores;    /// 在graphics队列中操作完成后，触发这里的Finish信号量，将结果传递给显示队列
      /// 这个？干啥？
      /// 这里与pipline中设定的格式其实是一致的。只不过通过写入的方式（构建VkDescriptorBufferInfo对象）
      /// 与 vkUpdateDescriptorSets函数调用， 将descriptorSets中的内容填充（本质与piplline中设定的值一样）
      std::vector<VkDescriptorSet> descriptorSets;
      /// 每一帧更新所需 顶点GPU存储，暂存区对象不会每次消失，而是单独扩容
      VkBuffer MapPointStagingBuffer = VK_NULL_HANDLE;               /// 地图点 暂存区 引用
      VkDeviceMemory MapPointStagingBufferMemory = VK_NULL_HANDLE;   /// 暂存区实际大小 [成员变量一定要初始化啊！！！]
      VkDeviceSize currentBufferSize = 0;       ///


      /// 相机操控系统

      static void framebufferResizeCallback( GLFWwindow* window ,  int width ,  int height ) {
        auto app = reinterpret_cast<VulkanViewer*>(glfwGetWindowUserPointer(window));
        app->framebufferResized = true;
      }

      /// Vulkan验证层信息
      const std::vector<const char*> validationlayers ={
        "VK_LAYER_KHRONOS_validation"
      };

      #ifdef NDEBUG
        const bool enableValidationLayers = false;
      #else
        const bool enableValidationLayers = true;
      #endif

      bool checkValidationLayerSupport();
      std::vector<const char*> getRequiredExtensions();
      void populateDebugMessengerCreateInfo (VkDebugUtilsMessengerCreateInfoEXT& createInfo);
      void setupDebugMessenger();
      VkResult CreateDebugUtilsMessengerEXT( VkInstance instance , const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo ,
        const VkAllocationCallbacks* pAllocator , VkDebugUtilsMessengerEXT* pDebugMessenger );

      void DestoryDebugUtilsMessengerEXT( VkInstance instance , VkDebugUtilsMessengerEXT debugMessenger , const VkAllocationCallbacks* pAllocator );

      /// 处理input
      void processInput(GLFWwindow* window);
      void HandleMouseButton(int button, int action);
      bool rightMousePressed = false;

      /// UI绘制
      void DrawFPSGraph(float deltaTime);
      void renderPerformance();

    };


}

#endif //VULKANVIEWER_H
