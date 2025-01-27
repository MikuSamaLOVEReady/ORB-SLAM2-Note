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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include <spdlog/logger.h>
#include <cmath>

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,        /// 配置 + 词典文件准备
        NO_IMAGES_YET=0,            /// 没有图像输入
        NOT_INITIALIZED=1,          /// 至少需要一个图像
        OK=2,
        LOST=3
    };

    eTrackingState mState;               /// tracking 线程当前Frame的跟踪状态
    eTrackingState mLastProcessedState;  /// 上一frame的跟踪状态

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;            /// 通过grabImag【传感器】 产生一个对象赋值给currentFrame
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    /// min_cost_max_flow 算法组成

    /// 1. Maximum point visibility:
    int func_point( int n  , int m ){
        if( m == n) return 1;
        return (n+1)/(n-1) * func_point(n+1 , m);
    }

    int cal_Point_visibility( MapPoint* mp  , Frame* current_frame ){
        int m  = mp->Observations();
        int n = current_frame->GetFrameMaxVisibility();
        return func_point(m , n);
    }

    /// 2. Maximum spatial diversity:
    float calculateMinCircleRadius( const int width , const int height ){
        double diagonal = sqrt(height * height + width * width);
        // 最小圆的半径是对角线长度的一半
        return diagonal / 2;
    }

    int cal_pointNeibor( MapPoint* mp , KeyFrame* current_frame ) {
        vector<size_t> indices(0);
        if( mp->mbTrackInView){
            const int windowSizeX = 64 / 2; // 半宽度
            const int windowSizeY = 48 / 2;
            float u =  mp->mTrackProjX;
            float v =  mp->mTrackProjX;
            //int nLastOctave = current_frame.mvKeys[i].octave;
            /// 获取当前 金字塔层级
            int index = mp->GetIndexInKeyFrame(current_frame);
            if( index == -1 ) return -1;    ///  无法计算
            int cur_level =  current_frame->mvKeysUn[index].octave;
            /// TOOD: 这里参考ORBmatcher.cc 1479行， 基础阈值可微调
            /// 将阈值设置为 覆盖矩形的最小圆的半径
            float th = calculateMinCircleRadius(windowSizeX , windowSizeY);
            float radius = th*current_frame->mvScaleFactors[cur_level];
            indices = current_frame->GetFeaturesInArea(u, v, radius);
        }
        return indices.size();
    }

    int calculate_Point_diversity( MapPoint* mp , KeyFrame* frame_i , KeyFrame* frame_j ){
            int N1 = cal_pointNeibor(mp , frame_i);
            int N2 = cal_pointNeibor(mp , frame_j);
            // 计算 并向上取整
            double result = log( N1 * N2 + 1);
            int ceilResult = static_cast<int>(ceil(result));
            return ceilResult;
    }

    /// 3. Maximum frame baseline length: 最大基线计算
    double calculate_baseLineDis( KeyFrame* frame_i  , KeyFrame* frame_k  ) {
            auto m1 = frame_i->GetCameraCenter();
            auto m2 = frame_k->GetCameraCenter();
            cv::Mat diff = m1 - m2;
            double distance = cv::norm(diff);
            double result = 10 / (0.1 * distance + 1);
            return result;
    }


    /// 4. 深度补偿/惩罚函数
    double cal_penalty ( MapPoint* mp , KeyFrame* current_frame ) {

        ///获取当前地图点深度
            cv::Mat world_pos = mp->GetWorldPos();
            cv::Mat Tcw = current_frame->GetPose();
            cv::Mat MP_Camera_pos = Tcw.rowRange(0,3).colRange(0,3) * world_pos + Tcw.rowRange(0,3).col(3);     ///
            ///  MP_Camera_pos 获取Z坐标
            float depth = MP_Camera_pos.at<float>(2);
            /// TODO：惩罚值计算，需要实验验证
            double avg_depth = current_frame->GetDepthAVG();
            double delta_depth = depth - avg_depth;
            double k = 1.0f;
            double penalty_value = k * delta_depth * delta_depth * (delta_depth >= 0 ? 1 : -1);
            return penalty_value;
    }


    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    /// 当没有足够的地图点用于匹配时，系统可能会使用一些临时点（temporal points）进行跟踪，执行一种类似于视觉里程计的操作。这种方式允许系统在没有足够地图信息时仍能保持对相机位置的估计。
    /// 只在定位状态工作。

    bool mbVO;   // 需要与地图中的point对比？ 如果地图中没有，但仍然

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;                            /// 与Cur_Frame 拥有最多相似部分的Frame
    std::vector<KeyFrame*> mvpLocalKeyFrames;           /// 但凡能与当前Frame 有相同的Mppoint的KF都会被加入             「用于构成局部地图」
    std::vector<MapPoint*> mvpLocalMapPoints;           /// 当前local地图的地图点（把所有local_KF能观察到的都加入）       「用于构成局部地图」
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;      /// 它用于确定在SLAM过程中哪些点被认为是"近点"或"远点"。近点和远点通常有不同的处理方式，因为远点的深度估计不够精确

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;           ///  是否构造KF的关键 、 如果

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;                        /// 上一个grab出来的关键帧
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;        /// 上一次的重定位Frame 记录了上一次执行重定位的帧的ID。当系统尝试执行重定位时，它会记录当前的帧ID到这个变量。

    //Motion Model
    cv::Mat mVelocity;                      /// 仅仅为

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    /// LOG 日志系统
    std::shared_ptr<spdlog::logger> logger;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
