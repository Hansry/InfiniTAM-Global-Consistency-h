// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMapGraphManager.h"

namespace ITMLib
{
	/** \brief active map manager
	*/
	class ITMActiveMapManager
	{
	public:
		typedef enum { PRIMARY_LOCAL_MAP, NEW_LOCAL_MAP, LOOP_CLOSURE, RELOCALISATION, LOST, LOST_NEW } LocalMapActivity;

	private:
		struct ActiveDataDescriptor 
		{
		        //localMapIndex应该是该局部地图在所有子地图容器allData中的id
			int localMapIndex;
			LocalMapActivity type;
			std::vector<Matrix4f> constraints;
			ORUtils::SE3Pose estimatedPose;
			int trackingAttempts;
		};

		//局部底部的管理器
		ITMMapGraphManager *localMapManager;
		
		//activeData存储当前处于活跃的子地图
		std::vector<ActiveDataDescriptor> activeData;

		int CheckSuccess_relocalisation(int dataID) const;
		int CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const;
		/// @brief 为主子地图和当前子地图俩个子地图添加relations
		void AcceptNewLink(int dataId, int primaryDataId, const ORUtils::SE3Pose & pose, int weight);

		float visibleOriginalBlocks(int dataID) const;
		bool shouldStartNewArea(void) const;
		
		bool shouldMovePrimaryLocalMap(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

	public:
	        /// @brief 创建新的local map
		int initiateNewLocalMap(bool isPrimaryLocalMap = false);
		
		/// @brief 在重定位或者发生回环的时候，处于inactive的localmap可能会变为active，这时候需要将该inactive localmap的Id存储到activeData中
                /// @param localMapId 发生回环或者重定位时候的inactive localmap的id (在存储了所有localmap的vector的index)
		/// @param isRelocalisation 如果跟踪失败且不插入关键帧(意味着当前帧与关键帧库中有相似的帧)，则为重定位，若跟踪成功且不插入关键帧，则为回环
                /// @return 返回该inactive localmap->active localmap之后存储在activeData中的index
		int initiateNewLink(int sceneID, const ORUtils::SE3Pose & pose, bool isRelocalisation);

		/// @brief 存储的俩个子地图之间的变换矩阵，Told_to_new指的是点云的变换那种，坐标系变换应该是Tnew_to_old，即当前子地图到primary子地图的位姿变换
		void recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess);
		
		/// @brief return whether or not the local map graph has changed
		/// @brief 如果子地图与主子地图相对变换关系发生了改变，则意味着local map graph也改变了，此时返回true.否则返回false
		bool maintainActiveData(void);

		///@brief 返回当前primary localmap的Id
		int findPrimaryDataIdx(void) const;
		int findPrimaryLocalMapIdx(void) const;

		int findBestVisualisationDataIdx(void) const;
		int findBestVisualisationLocalMapIdx(void) const;

		/// @brief 返回当前处于活跃状态的子地图的个数
		int numActiveLocalMaps(void) const { 
		  return static_cast<int>(activeData.size()); 
		}
		int getLocalMapIndex(int dataIdx) const { 
		  return activeData[dataIdx].localMapIndex; 
		}
		/// @brief 返回当前处于活跃状态的index为dataIdx的子地图的类型:NEW_LOCAL_MAP,LOOP_CLOSURE,RELOCALISATION
		LocalMapActivity getLocalMapType(int dataIdx) const { 
		  return activeData[dataIdx].type; 
		}

		ITMActiveMapManager(ITMMapGraphManager *localMapManager);
		~ITMActiveMapManager(void) {}
	};
}