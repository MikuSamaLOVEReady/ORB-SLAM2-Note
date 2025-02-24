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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);   /// 通过加锁 控制访问的一致性
    cv::Mat GetWorldPos();  /// 通过加锁 控制访问的一致性

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx); /// 添加当前mappoint 到对应的关键帧中，并给定对应index，建立两者关系
    void EraseObservation(KeyFrame* pKF);           /// 删除地图点在 指定keyframe中的联系。

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;                                               /// 单目观察到本mappoint+1 ， 双目、RGBD + 2 、 用于判断地图点的质量

    // Variables used by the tracking
    float mTrackProjX;                                      /// 这两个变量表示地图点在当前帧中的投影坐标（像素坐标）。
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;                                     /// 一个布尔变量，表示地图点是否在当前帧的视野内。
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;             ///
    long unsigned int mnLastFrameSeen;                      /// 最后看到该Mappoint的 Frame ID

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;                 /// 世界坐标系中的 世界坐标

     // Keyframes observing the point and associated index in keyframe
     /// 能观察到当前 mappoint的 keyframe 以及在 这一帧中[本地图点]的index【索引对应在帧中图像特征index】
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;       /// 地图点的特征描述子，用于后续与新观测到frame中的描述子做比对，
     /// 找到最相近的， 图像中的2D点代表的就是该3D点

     // Reference KeyFrame
     KeyFrame* mpRefKF;         /// 参考帧是什么意思？ 当构建出【三角化出】当前地图点时的对应keyframe

     // Tracking counters
     int mnVisible; /// 理论（从距离的）
     int mnFound;   /// 实际 观察到本MP的 frame

     // Bad flag (we do not currently erase MapPoint from memory)
     /// 例如由于视觉上的失效、被检测为外点等。 先标记、后删除
     bool mbBad;
     MapPoint* mpReplaced; /// 如果被替换，则其他thread看到这个指针不是null，则会重新访问新point

     // Scale invariance distances
     //// 由参考帧来决定
     float mfMinDistance;   /// 最近的相机观测距离 ， 如果相机到该特征点对应的地图点的距离？
     float mfMaxDistance;   ///

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
