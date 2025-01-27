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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

// stero双目
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info 设置ORB 特征提取的操作
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size(); //提取出了多少个特征点

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();   //双目畸变矫正

    ComputeStereoMatches(); // 双目左右特征点匹配->

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));       /// 一个特征
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    /// TODO：只会初始化一次
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv= static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid(); /// 将特征点分配到 grid中（用于加速）
}


//RGB-D
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints(); //RGB-D 畸变矫正

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    // 关键是计算没个grid的宽度和高度
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        /// 相机内参
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


//mono
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);          // 默认每个grid初始容量最多一半 左图 特征点个数

    for(int i=0;i<N;i++)                                ///  遍历所有特征点
    {
        const cv::KeyPoint &kp = mvKeysUn[i];           /// 特征点的UV（位置）
        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);  /// grid的每个格子中是一个 vector里面存储 特征点ID
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    //提取的 特征点uv存在vector中 , 描述符也是自动cv产生
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

///
void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();                                                            /// 旋转Matrix的逆 就是转置
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;                                                       /// 相机在世界坐标系中的Pos[推导见Notion]
                                                                                /// 逆向减去之前的平移，并且从Camera旋转回world坐标
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();                     /// Mappoint 世界坐标

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;                     /// 转移到->Camera空间下的Pos
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    /// 投影到 相机 背后了 ， 那就不在当前观察的范围内
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    /// Projection Matrix ， 判定是否投影离开了成像平面
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    /// 地图点与相机之间的距离，【世界坐标差】
    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;                            /// 统一坐标系（世界坐标系） 地图点 - 相机光心点【相机在世界坐标系中的Pos】
    const float dist = cv::norm(PO);                /// sqrt( x^2 + y^2 + z^2) 相机到Mappoint的距离

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist; /// 从地图点指向当前相机位置的向量。 Pn【单位向量，指向同一个位置】

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;                  /// 感觉仅仅是给 local mapping 用的变量
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;           /// 地图点在右目中的 u坐标
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;               // 视角余弦值，用于视角一致性检查

    return true;
}


/// 根据投影坐标、尺度因子、以及尺度金字塔层级、来定义一个F当前图像区域
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];     ///取出一个grid中之前存放的关键点们
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    /// 修正过后的点仍然可能不再frame内
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

//畸变修正
void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0) /// 双目相机的矫正发生在外部，这里直接把左特征点穿出去
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}


/// 关键就在于根据图像确定每个特征点对应的 mvuRight mvDepth
void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);                           /// 左图中的每个特征点都存储

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2; /// 判断后续是否匹配成功的条件

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;     /// [0]表示提取原始底层图像，像素最高的

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>()); ///

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);                                /// 给vector预留200个

    const int Nr = mvKeysRight.size();                                 /// 右侧图像中的 关键点

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];                       ///iR = 右侧特征值的index
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);                            /// 确定一个y值的范围（右侧，从距离上看，可能匹配的地方）
        const int minr = floor(kpY-r);

        for(int yi=minr; yi<=maxr; yi++)
            vRowIndices[yi].push_back(iR);                             ///  记录某一行 能在右图中找到
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    ///  < 最好 匹配的得分（小） ， 左图特征点ID >
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);


    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];   /// 矫正前 左侧图片特征点
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL]; /// 左图的y值 ， 取出可能匹配的右图特征点id

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        /// 1. 粗匹配 根据【描述子+金字塔层级】
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints （根据描述子来搜索、可能匹配的点）
        // best dipatcher
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];              /// 右特征点ID = iR
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)      /// 图像金字塔匹配
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);               /// 根据ID获取->描述符
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);  /// DescriptorDistance 计算汉明距离
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;              /// 右图像中最佳 特征点 index
                }
            }
        }

        // Subpixel match by correlation 表达的是 ORB 描述子之间的汉明距离（Hamming Distance）
        // 小于一个阈值 thOrbDist 时的条件。这是用于判断两个特征点的描述子是否足够相似，从而认为它们可能匹配的条件。
        // 也就是之前的 候选node 只有一个会被选出来
        /// 3. 再在它周围做 像素级别特征匹配 【精匹配、 在有匹配点周围5x5的方格子内再比较】
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);                 /// 特征点u,v -> 图像金字塔上相应尺度的坐标
            const float scaledvL = round(kpL.pt.y*scaleFactor);                 ///
            const float scaleduR0 = round(uR0*scaleFactor);                     ///

            // sliding window search
            ///  提取左图像的窗口
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].
                    rowRange(scaledvL-w,scaledvL+w+1).
                    colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);                                                   ///图像通常以整数形式表示，而在计算中使用浮点数类型可以更好地处理图像差异
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);     ///减少光照变化对图像的影响

            /// 计算右图像窗口
            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;                       /// 11记录 右侧备选特征点周围11个像素的（距离）得分，越小越接近
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;           /// 以初始右图 scaleduR0 为中心 左右各扩展L各像素
            const float endu = scaleduR0+L+w+1;         ///
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)           /// 像素级别特征匹配 -5 -> 5 范围内遍历可能的偏移值
            {
                /// 提取右图与左特征点对应的子图窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].
                        rowRange(scaledvL-w,scaledvL+w+1).
                        colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);


                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);    ///取消亮度影响

                float dist = cv::norm(IL,IR,cv::NORM_L1);  /// 两个矩阵同位置上直接做差，再求和，dist越小越相似
                if(dist<bestDist)
                {
                    bestDist =  dist;           ///  相似程度得分（int）
                    bestincR = incR;            /// 第几个偏移值最好
                }
                vDists[L+incR] = dist;          /// 记录所有的dist
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            /// 寻取最好匹配的周围2个像素点，一共三个点构成一个曲线（拟合抛物线）
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));     //

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            /// 与当前关键点 最佳对应的 像素点的x坐标
            float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0+(float)bestincR+deltaR); /// 金字塔缩放值 * （坐标中心 + 窗口中最好的偏移 + 旋转预估量）

            float disparity = (uL-bestuR);                                  /// 同一个点在 左特征值 与 右特征值的 u像素差 【视差】
                                                                            /// （disparity）视差越大 ，物体越近

            if(disparity>=minD && disparity<maxD)                           /// 视差需要被限定在一定范围内
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]= mbf/disparity;                                 ///  为左特征点  u坐标和 depth赋值
                mvuRight[iL] = bestuR;                                      ///  右图对应点特征u坐标（x坐标   SLAM14讲 P104
                vDistIdx.push_back(pair<int,int>(bestDist,iL));      ///  < 最好匹配的得分（小） ， 左图特征点ID >
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());                      /// 从最相似 -> 最不同排序【一共N的特征点,从小到大排序，最相似的在最前面】
    const float median = vDistIdx[vDistIdx.size()/2].first;               /// 选取一个 --- 相似度的中间值
    const float thDist = 1.5f*1.4f*median;                                /// 选择中间值的一定倍数作为匹配成功的标志

    for(int i=vDistIdx.size()-1;i>=0;i--)                                 /// 从差异最大的开始
    {
        if(vDistIdx[i].first<thDist)                                      /// 只有特征点中 相似度够高（分数够低）才不会被设置
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;                              ///  其他相似程度低的右图点，就全部设置无用值
            mvDepth[vDistIdx[i].second]=-1;                               ///  vDistIdx[i].second -> 表示左图特征ID是多少？
        }
    }

}


//从RGB-D 计算到  right img
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    //N 是关键点个数 ，
    mvuRight = vector<float>(N,-1); //
    mvDepth = vector<float>(N,-1);  //存储对应水平像素坐标的下的深度

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];     //取 关键点的 在左目上的data
        const cv::KeyPoint &kpU = mvKeysUn[i];  //取 去畸变后的关键点 u

        const float &v = kp.pt.y;               //根据关键点uv 从depth图中读出深度
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;                  //关键点的深度值 【深度不需要畸变矫正吧】
            mvuRight[i] = kpU.pt.x-mbf/d;    //关键点的在 右目相机的水平像素坐标
        }
    }
}


/// 初始化的时候调用，通过把特征点直接投影到世界空间，构造mappoint
/// TrackWithModel 时候也调用。
cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];                 /// attention： 只有能提供深度的才可以直接反向投影

    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;       /// 特征点矫正后的坐标
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;         /// 逆向 使用 projection matrix获取，Camera Space下的坐标。  投影矩阵逆*成像平面
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);       /// 得到相机空间下的Pos
        return mRwc*x3Dc+mOw;                              ///X3Dc已经是Camera 空间下的点了， mOw表示相机的平移     /// 返回特征点所在的世界坐标
    }
    else
        return cv::Mat();
}

/// 需不需要 mutex？
int Frame::GetFrameMaxVisibility() {
    /// init 当前最大可见值
    maxVisibility = 0;
    for( auto it = mvpMapPoints.begin() ; it!=mvpMapPoints.end() ; ++it ){
        MapPoint* pMP = *it;
        if( pMP ){
            maxVisibility =  maxVisibility > pMP->Observations() ? maxVisibility : pMP->Observations();
        }
    }
    return maxVisibility;
}



} //namespace ORB_SLAM
