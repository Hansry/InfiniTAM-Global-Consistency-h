// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../../Objects/Scene/ITMLocalMap.h"
#include "../../Core/ITMDenseMapper.h"
#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib
{
	/* This helpful abstract interface allows you to ignore the fact that
	scenes are templates.
	*/
	class ITMMapGraphManager
	{
	public:
		virtual ~ITMMapGraphManager(void) {}

		virtual int createNewLocalMap(void) = 0;
	        /// @brief 从allData移除id为localMapId的子地图
	        /// @param localMapId 需要移除的子地图的Id 
		virtual void removeLocalMap(int index) = 0;
		virtual size_t numLocalMaps(void) const = 0;

		/// @brief 从allData中找到id为fromLocalMap的子地图，得到该子地图的ConstraintList（map类型，key为与该子地图产生约束的子地图的id，
		///        val为俩个地图相对变换），若在key中找到了toLocalMap这个id，则返回俩个子地图之间的变换
		virtual const ITMPoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const = 0;
		virtual ITMPoseConstraint & getRelation(int fromLocalMap, int toLocalMap) = 0;
		virtual void eraseRelation(int fromLocalMap, int toLocalMap) = 0;
		
		/// @brief 获得当前Id的localMap与其他localMap之间的pose约束
		//         返回的是ConstranintList类型的数据,typedef std::map<int, ITMPoseConstraint> ConstraintList;
		virtual const ConstraintList & getConstraints(int localMapId) const = 0;

		/// @brief 将id为localMapId的子地图的GlobalPose设为pose,即Tc->w
		virtual void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) = 0;
		/// @brief 得到id为localMapId的子地图的位姿
		virtual const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const = 0;

		virtual bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose) = 0;

		virtual const ORUtils::SE3Pose* getTrackingPose(int localMapId) const = 0;
		
		/// @brief id为localMapId的局部地图的大小
		virtual int getLocalMapSize(int localMapId) const = 0;
		
		/// @brief 返回该局部地图可见block的数量
		virtual int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
	};

	template<class TVoxel, class TIndex>
	class ITMVoxelMapGraphManager : public ITMMapGraphManager
	{
	private:
		const ITMLibSettings *settings;
		const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
		const ITMDenseMapper<TVoxel, TIndex> *denseMapper;
		Vector2i trackedImageSize;

		/// 创建一个vector来维护ITMLocalMap
		std::vector<ITMLocalMap<TVoxel, TIndex>*> allData;

	public:
		ITMVoxelMapGraphManager(const ITMLibSettings *settings, const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine, const ITMDenseMapper<TVoxel, TIndex> *denseMapper, const Vector2i & trackedImageSize);
		~ITMVoxelMapGraphManager(void);

		int createNewLocalMap(void);
		void removeLocalMap(int index);
		size_t numLocalMaps(void) const { 
		  return allData.size(); 
		}

		const ITMLocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) const { 
		  return allData[localMapId]; 
		}

		/// @brief 得到Id为localMapId的子地图
		ITMLocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) { 
		  return allData[localMapId]; 
		}


		const ITMPoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const;
		ITMPoseConstraint & getRelation(int fromLocalMap, int toLocalMap);
		void eraseRelation(int fromLocalMap, int toLocalMap);
		
		const ConstraintList & getConstraints(int localMapId) const { 
		  return allData[localMapId]->relations; 
		}
		
		void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) { 
		  allData[localMapId]->estimatedGlobalPose = pose; 
		}
		
		
		const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const { 
		  return allData[localMapId]->estimatedGlobalPose; 
		}

		bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose);
		
		const ORUtils::SE3Pose* getTrackingPose(int localMapId) const { 
		  return getLocalMap(localMapId)->trackingState->pose_d; 
		}

		int getLocalMapSize(int localMapId) const;
		int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const;
		
		///  @brief 得到id为fromLocalMapId和toLocalMapId之间的变换矩阵
		///  T_{t,f} = T_{t,w}*T_{f,w}^{-1} = T_{t,w}*T_{w,f}
		ORUtils::SE3Pose findTransformation(int fromlocalMapId, int tolocalMapId) const;
	};
}