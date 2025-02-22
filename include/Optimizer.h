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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);

    void static SparseOptimization(Frame* pFrame , Map* pMap , KeyFrameDatabase *pKFDB );


    /// 稀疏化优化器
    static float cal_Point_visibility( MapPoint* mp  , Frame* current_frame );
    static float func_point( int n  , int m );

    /// 2.1 Maximum spatial diversity:
    static float calculateMinCircleRadius( const int width , const int height );

    /// 2.2 运行时： 地图点映射回Frame中，查看该地图点对应特征点，周围有哪些特征点
    static int cal_pointNeibor( MapPoint* mp , KeyFrame* current_frame );

    /// 2.3 任意两帧之间, 同一个特征点在 [i] 、 [j] 两帧中一个Circle中特征点个数
    static int calculate_Point_diversity( MapPoint* mp , KeyFrame* frame_i , KeyFrame* frame_j );

    static cv::Point2f getProjection(MapPoint* mp, KeyFrame* current_frame);

    /// 3.1 Frame[i] 与 Frame[k] 之间的基线长计算
    static double calculate_baseLineDis( KeyFrame* frame_i  , KeyFrame* frame_k );

    /// EXT 4.深度补偿函数
    static double cal_penalty ( MapPoint* mp , KeyFrame* current_frame );

};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
