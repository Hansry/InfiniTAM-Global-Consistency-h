// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings)
{
	swappingEngine = NULL;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CPU<TVoxel,TIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<TVoxel,TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CUDA<TVoxel,TIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel, TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CPU<TVoxel,TIndex>();
#endif
		break;
	}
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete voxelBlockOpEngine;

	if (swappingEngine!=NULL) delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ResetScene(ITMScene<TVoxel,TIndex> *scene) const
{
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	// split and merge voxel blocks according to their complexity
	voxelBlockOpEngine->SplitAndMerge(scene, renderState);

	// allocation
	//步骤1：对于当前帧的深度图，遍历每一个像素，通过相机内参得到基于当前相机坐标系的三维空间点,并转换到世界坐标系下
	//步骤2:查找该三维空间点所属的voxel block的三维空间坐标（具体做法为将三维空间点以m为单位转换成voxel block为单位），通过哈希映射计算其在
	//     entriesAllocType，blockCoords（大小为：number of bucket + size of excess list）等的索引，并判断是否需要在voxelAllocationList
	//     （大小为SDF_LOCAL_BLOCK_NUM（voxel block的个数） * SDF_BLOCK_SIZE3（voxel block存储值的大小））给voxel block分配新的位置
	//步骤3:如果判断voxel block已存在，但是被swap out,则需要将其swap in, 如果不存在则需要在ordered list或者excess list进行分配
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState);

	// integration
	//步骤1:将当前坐标系对应的空间点转到世界坐标系，找到对应的voxel block
	//步骤2:在voxel block中，对当前的空间点找到对应的volume,对其中的值进行融合
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);

// 	if (swappingEngine != NULL) {
// 		// swapping: CPU -> GPU
// 		swappingEngine->IntegrateGlobalIntoLocal(scene, renderState);
// 		// swapping: GPU -> CPU
// 		swappingEngine->SaveToGlobalMemory(scene, renderState);
// 	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true);
}

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;

