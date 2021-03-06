// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CPU.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CPU(void) 
{
	int noTotalEntries = ITMVoxelBlockHash::noTotalEntries;
	entriesAllocType = new ORUtils::MemoryBlock<unsigned char>(noTotalEntries, MEMORYDEVICE_CPU);
	blockCoords = new ORUtils::MemoryBlock<Vector4s>(noTotalEntries, MEMORYDEVICE_CPU);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CPU(void) 
{
	delete entriesAllocType;
	delete blockCoords;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHashEntry));
	tmpEntry.ptr = -2;
	ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
	for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[i] = i;

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}


/// @brief 通过融合给定视角的深度和颜色信息来更新voxel blocks
template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

        //M_d为当前帧坐标系到世界坐标系下的变换矩阵Tcw,即深度图片的坐标系到世界坐标系下的变换矩阵Td,w
	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) {
	  //Note that:calib.trafo_rgb_to_depth指的是将RGB坐标系下的空间点转到Depth坐标系下，从坐标系转换角度来看，应该是Tdepth->rgb,即Depth坐标系到RGB坐标系的变换
	  //即calib.trafo_rgb_to_depth.calib_inv = Trgb,d (这里指坐标系的变换),因此Trgb,w = Trgb,d * Td,w
	  M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;
	}
	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noVisibleEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[visibleEntryIds[entryId]];
		
		//identifies an actual allocated entry in the voxel block array (hansry)
		if (currentHashEntry.ptr < 0) continue;

		//其中currentHashEntry.pos存储的xyz是以voxel block为计量单位，而不是m
		//实际上globalPos为voxel block的三维坐标，而不是空间点的坐标，通过读取voxel block可以得到存储在
		//voxel block中的空间点在sdf的表达（即通过得到voxel block,可以得到voxel block的网格中的sdf值）
		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		
		//乘上SDF_BLOCK_SIZE后将以voxel block为计量单位转换成以voxel为计量单位
		globalPos *= SDF_BLOCK_SIZE;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		//对voxel block里面的sdf值进行更新
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) 
		  for (int y = 0; y < SDF_BLOCK_SIZE; y++) 
		    for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;
			
                        //一个voxel block中存储了8x8x8 or 4x4x4个voxels，需要找到他们的locId
			locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			if (stopIntegratingAtMaxW) 
			  if (localVoxelBlock[locId].w_depth == maxW) 
			    continue;
			//if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;

			//乘上voxelSize之后将globalPose以voxel为计量单位转成了pt_model以m为计量单位
			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;

			ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, 
				projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
		}
	}
}

/// @brief 根据当前视角的深度图转成空间中的voxel block后，判断voxel block是否已经在voxelAllocationList或者excessAllocationList分配，
///        若已经分配，则更改其状态为当前可见，若还未分配，则进行分配。
template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool isDefusion)
{
	Vector2i depthImgSize = view->depth->noDims;
	//场景体素的大小
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	//invM_d为世界坐标系到当前帧坐标系的变换矩阵Twc (其中w为给定的子地图)
	//M_d为当前帧坐标系到世界坐标系下的变换矩阵Tcw
	M_d = trackingState->pose_d->GetM(); 
	M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	
	//返回的是存储hash entry的列表
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->useSwapping ? scene->globalCache->GetSwapStates(false) : 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
	Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noTotalEntries;

	bool useSwapping = scene->useSwapping;

	//1.0m下有多少个blocks
	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	int noVisibleEntries = 0;

	memset(entriesAllocType, 0, noTotalEntries);

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
		entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

	//build hashVisibility
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < depthImgSize.x*depthImgSize.y; locId++)
	{
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
			invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
			scene->sceneParams->viewFrustum_max);
	}

	if (onlyUpdateVisibleList) useSwapping = false;
	if (!onlyUpdateVisibleList)
	{
		//allocate
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx, exlIdx;
			unsigned char hashChangeType = entriesAllocType[targetIdx];

			switch (hashChangeType)
			{
			case 1: //needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					hashTable[targetIdx] = hashEntry;
				}

				break;
			case 2: //needs allocation in the excess list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

				if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					//由于通过hash function对voxel block的位置计算得到的hash entry已经被占用，这时候需要添加offerset以找到excess list对应的地方进行存储
					//需要强调下，offset的基准是SDF_BUCKET_NUM,即hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1
					hashTable[targetIdx].offset = exlOffset + 1; //connect to child

					//在excess list进行hash Entry的存储 
					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

					//设置为可见
					entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
				}

				break;
			}
		}
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];
		
		//hashVisibleType == 3是上一帧可见的标记,判断上一帧可见的voxel block在当前视角下是否依旧可见
		if (hashVisibleType == 3)
		{
			bool isVisibleEnlarged, isVisible;

			if (useSwapping)
			{
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisibleEnlarged) {
				  hashVisibleType = 0;
				}
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisible) { 
				  hashVisibleType = 0; 
				}
			}
			entriesVisibleType[targetIdx] = hashVisibleType;
		}

		if (useSwapping)
		{
		/* 
		  swapStates: 
		  0 - most recent data is on host, data not currently in active memory
	          1 - data both on host and in active memory, information has not yet been combined
		  2 - most recent data is in active memory, should save this data back to host at some point
		*/
		//如果当前voxel block可见，则将其状态设为“该数据存在于host和activate memory,等待融合"
		   if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) {
		     swapStates[targetIdx].state = 1;
		   }
		}

		if (hashVisibleType > 0)
		{	
		        //visibleEntryIDs存储了所有可见的hash entry的索引
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}

#if 0
		// "active list", currently disabled
		if (hashVisibleType == 1)
		{
			activeEntryIDs[noActiveEntries] = targetIdx;
			noActiveEntries++;
		}
#endif
	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping)
	{
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			//hash entry可见同时且被标记为swapped out (has been removed)，需要将其swapped in?
			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) 
			{
				vbaIdx = lastFreeVoxelBlockId; 
				lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) {
				  hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
				}
			}
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;

	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::Decay(
		ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		const ITMRenderState *renderState,
		int maxWeight,
		int minAge,
		bool forceAllVoxels)
{
	throw std::runtime_error("Voxel decay is not yet supported on the CPU.");
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::DecayDefusionPart(
		ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		const ITMRenderState *renderState,
		int maxWeight,
		int minAge,
		bool forceAllVoxels)
{
	throw std::runtime_error("Voxel decay is not yet supported on the CPU.");
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::SlideWindow(
		ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		const ITMRenderState *renderState,
                int maxAge)
{
	throw std::runtime_error("Voxel decay is not yet supported on the CPU.");
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::SlideWindowDefusionPart(
		ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		const ITMRenderState *renderState,
                int maxAge, int maxSize)
{
	throw std::runtime_error("Voxel decay is not yet supported on the CPU.");
}


template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::DeIntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState)
{
       throw std::runtime_error("DeIntegrateIntoScene is not yet supported on the CPU.");
}

template<class TVoxel>
size_t ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::GetDecayedBlockCount()
{
	throw std::runtime_error("Voxel decay is not yet supported on the CPU.");
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHHash>::ITMSceneReconstructionEngine_CPU(void) 
{
	int noTotalEntries = ITMVoxelBlockHHash::noTotalEntries;
	entriesAllocType = (uchar*)malloc(noTotalEntries);
	blockCoords = (Vector4s*)malloc(noTotalEntries * sizeof(Vector4s));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHHash>::~ITMSceneReconstructionEngine_CPU(void) 
{
	free(entriesAllocType);
	free(blockCoords);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHHashEntry));
	tmpEntry.ptr = -3;
	ITMHHashEntry *hashEntry_ptr = scene->index.GetEntries();
	for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	for (int listId = 0; listId < SDF_HASH_NO_H_LEVELS; listId++)
	{
		int startPoint = listId * SDF_EXCESS_LIST_SIZE;
		for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[startPoint + i] = i;
	}

	for (int i = 0; i < SDF_HASH_NO_H_LEVELS; i++) scene->index.SetLastFreeExcessListId(i, SDF_EXCESS_LIST_SIZE - 1);
}

/// @brief 通过融合给定视角的深度和颜色信息来更新voxel blocks
template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHHash>::IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float smallestVoxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) {
	  M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;
	}
	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int entryNo = 0; entryNo < noVisibleEntries; entryNo++)
	{
		Vector3i globalPos;
		int entryId = visibleEntryIDs[entryNo];
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		float localVoxelSize = smallestVoxelSize * (1 << ITMVoxelBlockHHash::GetLevelForEntry(entryId));
		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;

			locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;

			pt_model.x = (float)(globalPos.x + x) * localVoxelSize;
			pt_model.y = (float)(globalPos.y + y) * localVoxelSize;
			pt_model.z = (float)(globalPos.z + z) * localVoxelSize;
			pt_model.w = 1.0f;

			ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
		}
	}
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHHash>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, 
											 const ITMRenderState *renderState, 
											 bool onlyUpdateVisibleList, 
											 bool isDefusion)
{
	Vector2i depthImgSize = view->depth->noDims;
	float smallestVoxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM(); M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->useSwapping ? scene->globalCache->GetSwapStates(false) : 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	int noTotalEntries = scene->index.noTotalEntries;

	bool useSwapping = scene->useSwapping;
	if (onlyUpdateVisibleList) useSwapping = false;

	float oneOverSmallestBlockSize = 1.0f / (smallestVoxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	int noVisibleEntries = 0;

	memset(entriesAllocType, 0, noTotalEntries);

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++) {
		// visible at previous frame and unstreamed
		// but maybe not there anymore...
		int ptr = hashTable[visibleEntryIDs[i]].ptr;
		// blocks might have disappeared due to splitting and merging
		if (ptr < -1) entriesVisibleType[visibleEntryIDs[i]] = 0;
		else entriesVisibleType[visibleEntryIDs[i]] = 3;
	}

	//build hashVisibility
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < depthImgSize.x*depthImgSize.y; locId++)
	{
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		buildHHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
			invProjParams_d, mu, depthImgSize, oneOverSmallestBlockSize, hashTable, scene->sceneParams->viewFrustum_min,
			scene->sceneParams->viewFrustum_max);
	}

	//allocate
	if (!onlyUpdateVisibleList) for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		int vbaIdx, exlIdx;
		ITMHHashEntry hashEntry;

		switch (entriesAllocType[targetIdx])
		{
		case 1: //needs allocation, fits in the ordered list
		case 3: //needs allocation, reactivate old entry
			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				Vector4s pt_block_all = blockCoords[targetIdx];

				hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				if (entriesAllocType[targetIdx] == 1) hashEntry.offset = 0;
				else hashEntry.offset = hashTable[targetIdx].offset;

				hashTable[targetIdx] = hashEntry;
				entriesVisibleType[targetIdx] = 1; //make entry visible
			}
			break;

		case 2: //needs allocation in the excess list
			int level = ITMVoxelBlockHHash::GetLevelForEntry(targetIdx);

			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
			exlIdx = (lastFreeExcessListIds[level])--;

			if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				Vector4s pt_block_all = blockCoords[targetIdx];

				hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[level*SDF_EXCESS_LIST_SIZE + exlIdx];

				hashTable[targetIdx].offset = exlOffset + 1; //connect to child

				hashTable[level * ITMVoxelBlockHHash::noTotalEntriesPerLevel + SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

				entriesVisibleType[level * ITMVoxelBlockHHash::noTotalEntriesPerLevel + SDF_BUCKET_NUM + exlOffset] = 1; //make child visible
			}

			break;
		}
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		int level = ITMVoxelBlockHHash::GetLevelForEntry(targetIdx);
		float voxelSize = smallestVoxelSize * (1 << level);
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHHashEntry &hashEntry = hashTable[targetIdx];

		if (hashVisibleType >= 2)
		{
			bool isVisibleEnlarged, isVisible;

			if (hashVisibleType == 3) 
			{
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisible) { entriesVisibleType[targetIdx] = 0; hashVisibleType = 0; }
			}

			if (useSwapping && hashVisibleType == 2)
			{
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				entriesVisibleType[targetIdx] = isVisibleEnlarged; 
				hashVisibleType = isVisibleEnlarged;
			}
		}

		if (useSwapping)
		{
		 /* 
		 swapStates: 
		 0 - most recent data is on host, data not currently in active memory
	         1 - data both on host and in active memory, information has not yet been combined
		 2 - most recent data is in active memory, should save this data back to host at some point
		*/
		    
		     //如果当前voxel block可见，则将其状态设为“该数据存在于host和activate memory,等待融合"
		     if (entriesVisibleType[targetIdx] > 0 && swapStates[targetIdx].state != 2) {
			 swapStates[targetIdx].state = 1;
		     }
		}

		if (hashVisibleType > 0)
		{
		        //visibleEntryIDs存储了所有可见的hash entry的索引
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}

#if 0
		// "active list", currently disabled
		if (hashVisibleType == 1)
		{
			activeEntryIDs[noActiveEntries] = targetIdx;
			noActiveEntries++;
		}
#endif
	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping)
	{
	        //hash entry可见同时且被标记为swapped out (has been removed)，需要将其swapped in?
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) 
			{
				vbaIdx = lastFreeVoxelBlockId; 
				lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) {
				   hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
				}	  
			}
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;

	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool isDefusion)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < scene->index.getVolumeSize().x*scene->index.getVolumeSize().y*scene->index.getVolumeSize().z; ++locId)
	{
		int z = locId / (scene->index.getVolumeSize().x*scene->index.getVolumeSize().y);
		int tmp = locId - z * scene->index.getVolumeSize().x*scene->index.getVolumeSize().y;
		int y = tmp / scene->index.getVolumeSize().x;
		int x = tmp - y * scene->index.getVolumeSize().x;
		Vector4f pt_model;

		if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;
		//if (approximateIntegration) if (voxelArray[locId].w_depth != 0) continue;

		pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, 
			depth, depthImgSize, rgb, rgbImgSize);
	}
}

template class ITMLib::Engine::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
