//
// Created by mikusama on 16/02/25.
//

#include "VulkanCamera.h"

namespace ORB_SLAM2 {


    void ORB_SLAM2::Camera::UpdateCameraView() {
        glm::vec3 front;
        /// yaw pitch roll 都是球坐标上的的东西
        front.x = cos(glm::radians(pitch)) * cos(glm::radians(yaw));
        front.y = sin(glm::radians(pitch));
        front.z = cos(glm::radians(pitch)) * sin(glm::radians(yaw));

        cameraFront = glm::normalize(front);
        cameraRight = glm::cross(cameraFront, WorldUp);
        cameraUp = glm::cross(cameraRight, cameraFront);
    }

    glm::mat4 Camera::GetViewMatrix() {
        return glm::lookAt(cameraPos ,
                           cameraPos+cameraFront ,
                           cameraUp);
    }

    void Camera::processRoate(float xoffset, float yoffset) {
        float sensitivity = 0.05f;
        yoffset *= sensitivity;
        xoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if( pitch > 89.0f)
            pitch = 89.0f;
        if( pitch < -89.0f)
             pitch = -89.0f;
        UpdateCameraView();
    }

}