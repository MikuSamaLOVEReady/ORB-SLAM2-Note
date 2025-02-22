//
// Created by mikusama on 16/02/25.
//

#ifndef ORB_SLAM2_VULKANCAMERA_H
#define ORB_SLAM2_VULKANCAMERA_H

#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace ORB_SLAM2{

    class Camera{
        public:
            Camera() = delete;
            Camera(glm::vec3 pos = glm::vec3(2.0f ,2.0f ,2.0f) ,
                   glm::vec3 front = glm::vec3(0.0f , 0.0f , -1.0f),
                   glm::vec3 up = glm::vec3(0.0f , 0.0f , 1.0f)
                   ): cameraPos(pos) , cameraFront(front) , cameraUp(up),
                   yaw(0) , pitch(0), WorldUp(0.0f, 1.0f, 0.0f)
                   {
                        UpdateCameraView();
                   }


            glm::mat4 GetViewMatrix();

            /// model矩阵
            glm::vec3 cameraPos;
            glm::vec3 cameraFront;
            glm::vec3 cameraUp;
            glm::vec3 cameraRight;

            /// rotation
            float yaw;          /// 左右晃动
            float pitch;        /// 上下仰视

            void UpdateCameraView();
            void processRoate(float xoffset , float yoffset);

    private:
            /// 需要主定义世界朝向向上
            glm::vec3 WorldUp;



    };
}

#endif //ORB_SLAM2_VULKANCAMERA_H
