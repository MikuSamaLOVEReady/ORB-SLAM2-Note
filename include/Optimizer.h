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

    /// 稀疏化优化器 --- By 罗瑞笛
    void static SparseOptimization(Frame* pFrame );

    void static SparseOptimizationLocalBA(KeyFrame* pFrame , Map* pMap ,
                                          list<MapPoint*>& lLocalMapPoints ,
                                          set<KeyFrame*>& lLocalKeyFrames);

    static float cal_Point_visibility( MapPoint* mp  , Frame* current_frame );     /// 单帧优化
    static float cal_Point_visibility( MapPoint* mp  ,  int maxVis);  /// 多帧局部

    static float func_point( int n  , int m );

    /// 2.1 Maximum spatial diversity:
    static float calculateMinCircleRadius( const int width , const int height );

    /// 2.2 运行时： 地图点映射回Frame中，查看该地图点对应特征点，周围有哪些特征点
    static int cal_pointNeibor( MapPoint* mp , KeyFrame* current_frame );

    /// 2.3 任意两帧之间, 同一个特征点在 [i] 、 [j] 两帧中一个Circle中特征点个数
    static long long calculate_Point_diversity( MapPoint* mp , KeyFrame* frame_i , KeyFrame* frame_j );

    static cv::Point2f getProjection(MapPoint* mp, KeyFrame* current_frame);

    /// 3.1 Frame[i] 与 Frame[k] 之间的基线长计算
    static double calculate_baseLineDis( KeyFrame* frame_i  , KeyFrame* frame_k );

    /// EXT 4.深度补偿函数
    static double cal_penalty ( MapPoint* mp , KeyFrame* current_frame );

    /// 单帧优化
    static long long singleOptimize( MapPoint* pMP , Frame *pFrame , int index);
    //static long long singleOptimize( MapPoint* pMP , )

    ///  针对单次位姿优化 ～ 仅仅从特征点稀疏程度来看
    //   static void MapPointsDensity();

    void static LocalBundleAdjustmentLRD( list<MapPoint*> lLocalMapPoints,
                                       list<KeyFrame*> lLocalKeyFrames,
                                        bool *pbStopFlag, Map *pMap ,KeyFrame *pKF);


};

    struct Edge{
        int to{};
        int capacity{};
        int cost{};
        int rev{};

        Edge(int to , int capacity , int cost , int rev):
         to(to),capacity(capacity),cost(cost),rev(rev){}
    };

    class Graph{

    public:
        Graph( size_t mapCount , size_t covisFrameCount):
                mapPointCount(mapCount) , covisKeyFrameCount(covisFrameCount) ,
                adj(mapCount+covisFrameCount+2){};  /// 0 与 size-1 分别source 与 sink

        /// adj[index] index表示图定点的ID
        /// mapCount --> [ index从 1 开始到 mapCount ]
        /// [ index从 mapCount+1 -> mapCount+viscount ]
        void addEdege(int from , int to , int cap , int cost);

        /// 顶点的index情况
        pair<int , int> minCostMaxFlow(int source, int sink , int flowLimit = std::numeric_limits<int>::max() );

        /// 笛杰斯特拉（无法处理负边, 舍弃）
        std::vector<int> dijkstra(int source) {
            const int INF = std::numeric_limits<int>::max();
            int sumSize = mapPointCount + covisKeyFrameCount;
            std::vector<int> dist(sumSize, INF); // 距离数组，初始化为无穷大
            dist[source] = 0;              // 源节点到自身的距离为 0

            // 优先队列：存储 pair<距离, 节点>，使用最小堆
            using P = std::pair<int, int>;
            std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
            pq.push({0, source}); // 初始化：将源节点加入队列

            while (!pq.empty()) {
                int cost = pq.top().first;  // 当前节点的距离
                int u = pq.top().second;    // 当前节点
                pq.pop();

                // 如果当前距离大于已知的最短距离，跳过（优化）
                if (cost > dist[u]) continue;

                // 遍历所有邻接节点
                for (const Edge& e : adj[u]) {
                    int v = e.to;                // 邻接节点
                    int nextCost = cost + e.cost; // 通过 u 到 v 的路径距离

                    // 如果找到更短的路径，更新距离并加入队列
                    if (nextCost < dist[v]) {
                        dist[v] = nextCost;
                        pq.push({nextCost, v});
                    }
                }
            }

            return dist; // 返回从源节点到所有节点的最短距离
        }

        size_t GetGraphsize();
        vector<int> GetSelectedMpKF();
    private:
        size_t mapPointCount;
        size_t covisKeyFrameCount;

        vector<vector<Edge>> adj;
        vector<int> mapSelectedIndex{};

    };


} //namespace ORB_SLAM

#endif // OPTIMIZER_H
