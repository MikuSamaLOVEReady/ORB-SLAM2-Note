/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    // 将uv值 -> grid x y
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    int GetFrameMaxVisibility();

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight; ///

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters. FINISH
    cv::Mat mK;                 // TODO:可以优化一下
    static float fx;            /// 内参 只在第一个frame 出现时被给值，
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;          /// 相机的畸变矫正参数

    // Stereo baseline multiplied by fx. 基线乘以相机的焦距
    float mbf;      ///  b * f 本质上是为了方便求深度

    // Stereo baseline in meters.
    /// mb = mbf/fx
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;  ///  当前帧中的特征点个数

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;     /// [矫正前]这些vector的下标 =「特征点ID，也是mappoint的ID」 // mvKeys左目， mvKeysRight右目   //存储ORB提取出得特征点uv【位置】
    std::vector<cv::KeyPoint> mvKeysUn;                /// 矫正后的关键点uv坐标

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    /// 没有 只有u没有v 是因为纵坐标 与mvKeys[i] 的纵坐标相同
    std::vector<float> mvuRight; /// （左图）关键点【i】的在 右目相机的【水平】像素坐标 ， 纵坐标默认一直是对齐的// 只用水平距离可以计算深度 ： 只在双目时期有用、 RGB-D呢？
    std::vector<float> mvDepth;  ///  （双目可以估计出、深度）特征点对应的深度？我可以理解为相机空间下的z坐标吗？

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    // 【将特征点的位置编码成 二进制描述子】
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints; ///  _Frame中的特征点 能关联到 地图中的landmark（地图点）的个数

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;       /// 是否是外点

    // Keypoints are assigned to cells in a grid to reduce matching
    // complexity when projecting MapPoints.
    // 特征点ID 被放在一个grand中 64 x 48
    static float mfGridElementWidthInv;   // 每一个grid的宽度
    static float mfGridElementHeightInv;  //
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;                                   ///最关键的矩阵，！ World->Camera camera Pos

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;                         /// 当前KF的ID

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;                         /// 与Cur_Frame 拥有最多相似部分的Frame

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;    // 去畸变后图像中特征点的最小 x 坐标。
    static float mnMaxX;    // 去畸变后图像中特征点的最大 x 坐标。
    static float mnMinY;    // 去畸变后图像中特征点的最小 y 坐标。
    static float mnMaxY;    // 去畸变后图像中特征点的最大 y 坐标。

    static bool mbInitialComputations; //只为第一个出现的frame 做操作？


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;           /// 什么时候被确定的？ 是在解出 Tcw的时候吗？
    cv::Mat mOw; //==mtwc

    int maxVisibility = 0;
};

}// namespace ORB_SLAM

#endif // FRAME_H
