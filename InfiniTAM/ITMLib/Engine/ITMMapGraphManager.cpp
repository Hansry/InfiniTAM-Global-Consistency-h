// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMapGraphManager.h"

//#include <queue>

namespace ITMLib
{
  namespace Engine {
        //ITMVoxelMapGraphManager构造函数，初始化列表对private成员进行初始化
	ITMVoxelMapGraphManager::ITMVoxelMapGraphManager(const ITMLibSettings *_settings, const ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex> *_visualisationEngine, const ITMDenseMapper<ITMVoxel, ITMVoxelIndex> *_denseMapper, const Vector2i & _trackedImageSize)
		: settings(_settings), visualisationEngine(_visualisationEngine), denseMapper(_denseMapper), trackedImageSize(_trackedImageSize)
	{
	}

	//ITMVoxelMapGraphManager析构函数
	ITMVoxelMapGraphManager::~ITMVoxelMapGraphManager(void)
	{
		while (allData.size() > 0)
		{
		        //释放ITMLocalMap<TVoxel, TIndex>*指针所指的内存
			delete allData.back();
			//删除vector的最后一个元素
			allData.pop_back();
		}
	}

	int ITMVoxelMapGraphManager::createNewLocalMap(void)
	{
		int newIdx = (int)allData.size();
		allData.push_back(new ITMLocalMap(settings, visualisationEngine, trackedImageSize));

		//对于新分配的ITMLocalMap,对各种参数进行初始化
		denseMapper->ResetScene(allData[newIdx]->scene);
		return newIdx;
	}

	/// @brief 从allData移除id为localMapId的子地图
	/// @param localMapId 需要移除的子地图的Id 
	void ITMVoxelMapGraphManager::removeLocalMap(int localMapId)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return;

		// make sure there are no relations anywhere pointing to the local map
		// 对于需要移除的子地图，必须保证没有其他关系指向本子地图
		const ConstraintList & l = getConstraints(localMapId);
		for (ConstraintList::const_iterator it = l.begin(); it != l.end(); ++it) {
		  eraseRelation(it->first, localMapId);
		}
		// delete the local map
		delete allData[localMapId];
		// 删除的是迭代器的位置
		allData.erase(allData.begin() + localMapId);
	}

	/// @brief 返回id为fromLocalMap和toLocalMap子地图之间的关联
	/// @return 位姿之间的关联,数据类型为ITMPoseConstraint
	ITMPoseConstraint & ITMVoxelMapGraphManager::getRelation(int fromLocalMap, int toLocalMap)
	{
		ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		return m[toLocalMap];
	}

	static const ITMPoseConstraint invalidPoseConstraint;

	/// @brief 返回俩个子地图的relation
	const ITMPoseConstraint & ITMVoxelMapGraphManager::getRelation_const(int fromLocalMap, int toLocalMap) const
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) {
		  return invalidPoseConstraint;
		}
		const ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		ConstraintList::const_iterator it = m.find(toLocalMap);
		if (it == m.end()) {
		  return invalidPoseConstraint;
		}
		return it->second;
	}

	/// @brief 删除Id为fromLocalMap的子地图与Id为toLocalMap的子地图之间的关系
	/// @param toLocalMap 为当前需要移除的子地图的Id
	/// @param fromLocalMap 是与需要移除的子地图具有关联的地图的Id
	void ITMVoxelMapGraphManager::eraseRelation(int fromLocalMap, int toLocalMap)
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) return;

		//通过fromLocalMap得到与当前子地图具有关联的子地图，删除该关联的子地图中与当前的子地图的联系
		std::map<int, ITMPoseConstraint> & m = getLocalMap(fromLocalMap)->relations;
		m.erase(toLocalMap);
	}

	bool ITMVoxelMapGraphManager::resetTracking(int localMapId, const ITMLib::Objects::ITMPose & pose)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return false;
		allData[localMapId]->trackingState->pose_d->SetFrom(&pose);
		allData[localMapId]->trackingState->age_pointCloud = -1;
		return true;
	}

	/// @brief id为localMapId的局部地图的大小
	int ITMVoxelMapGraphManager::getLocalMapSize(int localMapId) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;

		ITMScene<ITMVoxel, ITMVoxelIndex> *scene = allData[localMapId]->scene;
		return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
	}

	/// @brief 返回该局部地图在minBlockId和maxBlockId之间可见的block数量
	int ITMVoxelMapGraphManager::countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIds) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;
		const ITMLocalMap *localMap = allData[localMapId];

		if (invertIds) 
		{
			int tmp = minBlockId;
			minBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - maxBlockId - 1;
			maxBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - tmp - 1;
		}

		return visualisationEngine->CountVisibleBlocks(localMap->scene, localMap->renderState, minBlockId, maxBlockId);
	}

	struct LinkPathComparison 
	{
		bool operator()(const std::vector<int> & a, const std::vector<int> & b) { return a.size() > b.size(); }
	};

	/// @brief 得到id为fromLocalMapId和toLocalMapId之间的变换矩阵
	/// T_{t,f} = T_{t,w}*T_{f,w}^{-1} = T_{t,w}*T_{w,f}
	ITMLib::Objects::ITMPose ITMVoxelMapGraphManager::findTransformation(int fromLocalMapId, int toLocalMapId) const
	{
		ITMLib::Objects::ITMPose fromLocalMapPose, toLocalMapPose;
		if ((fromLocalMapId >= 0) || ((size_t)fromLocalMapId < allData.size())) {
		  fromLocalMapPose = allData[fromLocalMapId]->estimatedGlobalPose;
		}
		if ((toLocalMapId >= 0) || ((size_t)toLocalMapId < allData.size())) {
		  toLocalMapPose = allData[toLocalMapId]->estimatedGlobalPose;
		}
		// toLocalMapPose.GetM()为T_{t,w}
		// fromLocalMapPose.GetInvM()为T_{w,f}
		return ITMLib::Objects::ITMPose(toLocalMapPose.GetM() * fromLocalMapPose.GetInvM());
	}
   } //namespace Engine
} //namespace ITMLib
