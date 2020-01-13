// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

struct AllocationTempData {
	int noAllocatedVoxelEntries;
	int noAllocatedExcessEntries;
	int noVisibleEntries;
};

using namespace ITMLib::Engine;

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *noVisibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i imgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *voxelArray, const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

template<class TVoxel, bool stopMaxW>
__global__ void integrateIntoSceneH_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *liveEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb,
	Vector4f projParams_d, Vector4f projParams_rgb, const ITMVoxelBlockHHash::IndexData *indexData,float smallestVoxelSize, float mu, int maxW);

__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, ITMHashEntry *hashTable, float viewFrustum_min,
	float viewFrustrum_max);
__global__ void buildHHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float oneOverSmallestBlockSize, ITMHHashEntry *hashTable, float viewFrustum_min,
	float viewFrustrum_max);

__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords);
__global__ void allocateVoxelBlocksListHHash_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, int *noAllocatedExcessEntries, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords);

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesVisibleType);

__global__ void setToType3(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries);
__global__ void setToType3HH(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries, const ITMHashEntry *hashTable);

template<bool useSwapping>
__global__ void buildVisibleList_device(ITMHashEntry *hashTable, ITMHashSwapState *swapStates, int noTotalEntries,
	int *visibleEntryIDs, AllocationTempData *allocData, uchar *entriesVisibleType,
	Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize, int offsetToAdd);

// host methods
/*
/// @brief 用来在整个地图上进行voxel decay
/// 有点类似于'decay_device',但是decayFull_device用于所有的voxels而不是只作用在可视voxel block上
template<class TVoxel>
__global__ void decayFull_device(const Vector4s *useBlockPositions,
                                 TVoxel *localVBA,
				 ITMHashEntry *hashTable,
				 int minAge,
				 int *voxelAllocationList,
				 int *lastFreeBlockId,
				 int *locks,
				 int currentFrame,
				 uchar *entriesVisibleType);
*/
template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaMalloc((void**)&allocationTempData_device, sizeof(AllocationTempData)));
	ITMSafeCall(cudaMallocHost((void**)&allocationTempData_host, sizeof(AllocationTempData)));

	int noTotalEntries = ITMVoxelBlockHash::noTotalEntries;
	ITMSafeCall(cudaMalloc((void**)&entriesAllocType_device, noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&blockCoords_device, noTotalEntries * sizeof(Vector4s)));
	
	ITMSafeCall(cudaMalloc((void**)&lastFreeBlockId_device, 1 * sizeof(int)));
	ITMSafeCall(cudaMalloc(&locks_device, SDF_BUCKET_NUM * sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&allocatedBlockPositions_device, SDF_LOCAL_BLOCK_NUM * sizeof(Vector4s)));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaFreeHost(allocationTempData_host));
	ITMSafeCall(cudaFree(allocationTempData_device));
	ITMSafeCall(cudaFree(entriesAllocType_device));
	ITMSafeCall(cudaFree(blockCoords_device));
	
	ITMSafeCall(cudaFree(lastFreeBlockId_device));
	ITMSafeCall(cudaFree(locks_device));
	ITMSafeCall(cudaFree(allocatedBlockPositions_device));
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();
        
	totalDecayedBlockCount = 0;
	// Clean up the visible frame queue used in voxel decay.
	while (! frameVisibleBlocks.empty()) {
		delete frameVisibleBlocks.front().blockCoords;
		frameVisibleBlocks.pop();
	}
	
	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	memsetKernel<TVoxel>(voxelBlocks_ptr, TVoxel(), numBlocks * blockSize);
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	fillArrayKernel<int>(vbaAllocationList_ptr, numBlocks);
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHashEntry));
	tmpEntry.ptr = -2;
	ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
	memsetKernel<ITMHashEntry>(hashEntry_ptr, tmpEntry, scene->index.noTotalEntries);
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	fillArrayKernel<int>(excessList_ptr, SDF_EXCESS_LIST_SIZE);

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

/// @brief 根据当前视角的深度图转成空间中的voxel block后，判断voxel block是否已经在voxelAllocationList或者excessAllocationList分配，若已经分配，则更改其状态为当前可见，若还未分配，则进行分配。
template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, 
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList)
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

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	
	//voxelAllocationList为分配的存储sdf的内存，大小为:voxel block的数量*voxel block的大小
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	//为了哈希冲突而额外分配的内存，用来存储sdf的内存， excessALlocationList该内存的首地址
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	
	//返回的是存储hash entry的列表hashTable，得到哈希表，通过哈希映射函数建立空间点与哈希表中的entry（ITMHashEntry数据类型）之间的关系
	//其中blockPose为空间点，hashEntry为索引到VBA内存的数据类型
	//hashIdx = hashIndex(blockPose) 
	//ITMHashEntry hashEntry = hashTable[hashIdx]
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->useSwapping ? scene->globalCache->GetSwapStates(true) : 0;

	// noTotalEntries = SDF_BUCKET_NUM（大小需要大于Voxel Block的数量） + SDF_EXCESS_LIST_SIZE(防止哈希冲突)
	int noTotalEntries = scene->index.noTotalEntries;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();

	dim3 cudaBlockSizeHV(16, 16);
	dim3 gridSizeHV((int)ceil((float)depthImgSize.x / (float)cudaBlockSizeHV.x), (int)ceil((float)depthImgSize.y / (float)cudaBlockSizeHV.y));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));

	dim3 cudaBlockSizeVS(256, 1);
	dim3 gridSizeVS((int)ceil((float)renderState_vh->noVisibleEntries / (float)cudaBlockSizeVS.x));

	//1.0m下有多少个blocks
	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	AllocationTempData *tempData = (AllocationTempData*)allocationTempData_host;
	//返回localVBA和ExcessList中倆段内存中还可以分配的Entries的数量，也可以说是分配的内存中最新的还没有被用的内存的地址
	// _______________________________
	//|___________________|||||||||||||
	//  (这里分界线即lastFreeBlockId或者ExcessListId)
	tempData->noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;
	tempData->noAllocatedExcessEntries = scene->index.GetLastFreeExcessListId();
	tempData->noVisibleEntries = 0;
	ITMSafeCall(cudaMemcpyAsync(allocationTempData_device, tempData, sizeof(AllocationTempData), cudaMemcpyHostToDevice));

	// entriesAllocType_device存储着 对应着entriAllocType_device的某个元素 的空间点是否需要被分配或者swap in
	// 1表示在ordered part进行分配, 2表示需要在excess中进行分配
	ITMSafeCall(cudaMemsetAsync(entriesAllocType_device, 0, sizeof(unsigned char)* noTotalEntries));

	if (gridSizeVS.x > 0) {
	  setToType3 << <gridSizeVS, cudaBlockSizeVS >> > (entriesVisibleType, visibleEntryIDs, renderState_vh->noVisibleEntries);
	}
	
	/// 判断当前帧空间点是否为其分配内存或者已经被分配了，状态保存在entriesAlloType_device中
	buildHashAllocAndVisibleType_device << <gridSizeHV, cudaBlockSizeHV >> >(entriesAllocType_device, entriesVisibleType, 
		blockCoords_device, depth, invM_d, invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable,
		scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max);

	bool useSwapping = scene->useSwapping;
	if (onlyUpdateVisibleList) useSwapping = false;
	if (!onlyUpdateVisibleList)
	{
		allocateVoxelBlocksList_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, excessAllocationList, hashTable,
			noTotalEntries, (AllocationTempData*)allocationTempData_device, entriesAllocType_device, entriesVisibleType,
			blockCoords_device);
	}

	if (useSwapping)
		buildVisibleList_device<true> << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, swapStates, noTotalEntries, visibleEntryIDs,
			(AllocationTempData*)allocationTempData_device, entriesVisibleType, M_d, projParams_d, depthImgSize, voxelSize, 0);
	else
		buildVisibleList_device<false> << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, swapStates, noTotalEntries, visibleEntryIDs,
			(AllocationTempData*)allocationTempData_device, entriesVisibleType, M_d, projParams_d, depthImgSize, voxelSize, 0);

	if (useSwapping)
	{
		reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries, 
			(AllocationTempData*)allocationTempData_device, entriesVisibleType);
	}

	ITMSafeCall(cudaMemcpy(tempData, allocationTempData_device, sizeof(AllocationTempData), cudaMemcpyDeviceToHost));
	renderState_vh->noVisibleEntries = tempData->noVisibleEntries;
	scene->localVBA.lastFreeBlockId = tempData->noAllocatedVoxelEntries;
	scene->index.SetLastFreeExcessListId(tempData->noAllocatedExcessEntries);
	
	//为了进行decay
	int totalBlockCount = scene->index.getNumAllocatedVoxelBlocks();
	size_t visibleBlockCount = static_cast<size_t>(tempData->noVisibleEntries);
	size_t visibleBlockByteCount = visibleBlockCount * sizeof(int);
	
	//Keep track of the visible blocks, which will be used later by the voxel decay mechanism
	ORUtils::MemoryBlock<int> *visibleEntryIDsCopy = nullptr;
	if(visibleBlockByteCount > 0){
	  ///分配与当前可视的voxel block相同的CUDA内存
	  visibleEntryIDsCopy = new ORUtils::MemoryBlock<int>(visibleBlockByteCount, MEMORYDEVICE_CUDA);
	  ITMSafeCall(cudaMemcpy(visibleEntryIDsCopy->GetData(MEMORYDEVICE_CUDA),
	                         visibleEntryIDs,
			         visibleBlockByteCount,
			         cudaMemcpyDeviceToDevice));
	}
	
	VisibleBlockInfo visibleBlockInfo = {
	    visibleBlockCount, //count
	    frameIdx, //frameIdx
	    visibleEntryIDsCopy, //visibleEntry
	};
	
	frameVisibleBlocks.push(visibleBlockInfo);
	frameIdx ++;
	
	//This just returns the size of the pre-allocated buffer
	//返回预分配的voxel block的数量，即所有可以分配的voxel block的数量
	long allocatedBlocks = scene->index.getNumAllocatedVoxelBlocks();
	//This is the number of blocks we are using out of the chunk that was allocated initially on the GPU (for non swapping case).
	//返回已经分配了的voxel blocks的数量
	long usedBlocks = allocatedBlocks - scene->localVBA.lastFreeBlockId - 1;
	
	//返回所有分配的Excess Entries的大小，其中Excess Entries主要是为了预防哈希冲突
	long allocatedExcessEntries = SDF_EXCESS_LIST_SIZE;
	//返回已经使用的Excess Entries的大小
	long usedExcessEntries = allocatedExcessEntries - tempData->noAllocatedExcessEntries;
	
	if(usedBlocks > allocatedBlocks){
	   usedBlocks = allocatedBlocks;
	}
	
	if(usedExcessEntries > allocatedExcessEntries){
	   usedExcessEntries = allocatedExcessEntries;
	}
	
	//Display some memory status, useful for debugging mapping failures.
	float percentFree = 100.0f * (1.0f - static_cast<float>(usedBlocks)/allocatedBlocks);
	float allocatedSizeMB = scene->localVBA.allocatedSize * sizeof(ITMVoxel) / 1024.0f / 1024.0f;
	printf("[Visible: %6d | Used blocks (primary): %8ld/%ld (%.2f%% free)\n"
	       "Used excess list slots: %8ld/%ld | Total allocated size: %.2fMiB]\n",
	       tempData->noVisibleEntries,
	       usedBlocks,
	       allocatedBlocks,
	       percentFree,
	       usedExcessEntries,
	       allocatedExcessEntries,
	       allocatedSizeMB);
        if(scene->localVBA.lastFreeBlockId < 0){
	   throw std::runtime_error("Invalid free voxel block ID. InfiniTAM has run out of space in "
								 "the Voxel Block Array.");
	}
	if(scene->index.GetLastFreeExcessListId() < 0){
	   throw std::runtime_error("Invalid free excess list slot ID. InfiniTAM has run out of slots "
				    "in the hash table excess list. Consider increasing the size of "
				    "the excess list or the number of buckets.");
	}
}

/// @brief 通过融合给定视角的深度和颜色信息来更新voxel blocks
template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;
	
	//如果当前视角没有任何有用的数据，则没必要进行内存分配
	if(renderState_vh->noVisibleEntries == 0){
	  return;
	}

	///M_d为当前帧坐标系到世界坐标系下的变换矩阵Tcw,即深度图片的坐标系到世界坐标系下的变换矩阵Td,w
	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) {
	  //Note that:calib.trafo_rgb_to_depth指的是将RGB坐标系下的空间点转到Depth坐标系下，从坐标系转换角度来看，应该是Tdepth->rgb,即Depth坐标系到RGB坐标系的变换
	  //即calib.trafo_rgb_to_depth.calib_inv = Trgb,d (这里指坐标系的变换),因此Trgb,w = Trgb,d * Td,w
	  M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;
	}
	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(renderState_vh->noVisibleEntries);

	if (scene->sceneParams->stopIntegratingAtMaxW)
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device<TVoxel, true, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device<TVoxel, true, true> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
	else
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device<TVoxel, false, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device<TVoxel, false, true> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}

///@brief 从hash table中将该block进行删除，同时释放block对应的VBA entry
///@param hashTable 哈希表
///@param blockPos block在voxel grid的位置，其实就是hashTable的输入，得到的是在hash entries
///@param locks Array used for locking in order to prevent data races when
///                             attempting to delete multiple elements with the same key.
///@param voxelAllocationList 分配的列表
///@param lastFreeBlockId 最新的voxel allocation list空闲处的索引
///@param entriesVisibleType For every entry in the hash table, keeps track of whether it's visible in the last/current frame.

///@Note Does not support swapping
template<class TVoxel, bool paranoid=false>
__device__ void deleteBlock(ITMHashEntry *hashTable,
                            Vector3i blockGridPos,
                            int *locks,
                            int *voxelAllocationList,
                            int *lastFreeBlockId,
                            uchar *entriesVisibleType)
{
   int keyHash = hashIndex(blockGridPos);
   //Lock the bucket for the operation, to ensure the lists stay consistent
   int status = atomicExch(&locks[keyHash], BUCKET_LOCKED);
   if(status != BUCKET_UNLOCKED){
     printf("Contention on bucket of hash value %d. Not going further with deletion of block "
	     "(%d, %d, %d).\n", keyHash, blockGridPos.x, blockGridPos.y, blockGridPos.z);
     return;
  }
  
  bool isFound = false;
  int outBlockIdx = -1;
  int outPrevBlockIdx = -1;
  findVoxel(hashTable, blockGridPos, 0, isFound, outBlockIdx, outPrevBlockIdx);
  
  bool isExcess = (outBlockIdx >= SDF_BUCKET_NUM);
  
  //Paranoid sanity checks
  if(paranoid){
    if(outPrevBlockIdx == -1){
      if(isExcess){
	printf("\n[ERROR] Found entity in excess list with no previous element (%d, %d, %d)!\n",
	       blockGridPos.x,
	       blockGridPos.y,
	       blockGridPos.z);
      }
    }
    else{
      if(!isExcess){
	printf("\n[ERROR] Found entity in bucket list with a previous guy!\n");
      }
    }
    
    if(!isFound || outBlockIdx<0){
      if(blockGridPos.x % 10 == 3){
	printf("\n\nFATAL ERROR: sanity check failed in 'decay_device' voxel (block) "
							   "found = %d, outBlockIdx = %d (%d, %d, %d) ; %s.\n",
		static_cast<int>(isFound),
	        outBlockIdx,
	        blockGridPos.x,
	        blockGridPos.y,
	        blockGridPos.z,
	        isExcess ? "excess":"non-excess"
	      );
      }
      atomicExch(&locks[keyHash], BUCKET_UNLOCKED);
      return;
    }
  }
  
  //First, deallocate the VBA slot.
  int freeListIdx = atomicAdd(&lastFreeBlockId[0],1);
  voxelAllocationList[freeListIdx+1] = hashTable[outBlockIdx].ptr;
  // TODO:Update excess freelist! (should work without doing it but leak excess slots.)
  // If updating the excess free List, make sure you also sync back the proper 'last excess slot index'.
  
  //Second, clear out the hash table entry, and do bookkeeping for buckets with more than one element.
  if(outPrevBlockIdx == -1){
    //In the ordered list
    if(hashTable[outBlockIdx].offset >= 1){
      //In the ordered list, with a successor(继承者)，继承者就是hashTable【outBlockIdx】
      long nextIdx = SDF_BUCKET_NUM + hashTable[outBlockIdx].offset -1 ;
      hashTable[outBlockIdx] = hashTable[nextIdx];
      
      entriesVisibleType[outBlockIdx] = entriesVisibleType[nextIdx];
      entriesVisibleType[nextIdx] = 0;
      
       // Free up the slot we just copied into the main VBA, in case there's still pointers
       // to it in the visible list from some to-be-decayed frame.
       // [RIP] Not doing this can mean the zombie block gets detected as valid in the future,
       // even though it's in the excess area but nobody is pointing at it.
       hashTable[nextIdx].offset = 0;
       hashTable[nextIdx].ptr = -2;
    }
    else{
       //In the ordered list, and no successor(继承者)
      hashTable[outBlockIdx].ptr = -2;
      entriesVisibleType[outBlockIdx] = 0;
    }
  }
  else{
    	// In the excess list with a successor or not.
        hashTable[outPrevBlockIdx].offset = hashTable[outBlockIdx].offset;
	hashTable[outBlockIdx].offset = 0;
	hashTable[outBlockIdx].ptr = -2;
	
	entriesVisibleType[outPrevBlockIdx] = entriesVisibleType[outBlockIdx];
	entriesVisibleType[outBlockIdx] = 0;
  }
  //Release the lock
  atomicExch(&locks[keyHash], BUCKET_UNLOCKED);
}

template<class TVoxel>
__device__ void decayVoxel(
		Vector3i blockGridPos,
		int locId,
		TVoxel *localVBA,			// could wrap in HashMap struct
		ITMHashEntry *hashTable,		// could wrap
		int minAge,
		int maxWeight,
		int *voxelAllocationList,		// could wrap
		int *lastFreeBlockId,			// could wrap
		int *locks,
		int currentFrame,
		uchar *entriesVisibleType		// could wrap
) {
	bool isFound = false;
	int blockHashIdx = -1;
	int blockPrevHashIdx = -1;
        
	int voxelIdx = findVoxel(hashTable, blockGridPos, locId, isFound, blockHashIdx, blockPrevHashIdx);

	if (-1 == blockHashIdx) {
		if (locId == 0) {
		     printf("ERROR: could not find bucket for (%d, %d, %d) @ hash ID %d.\n",
				   blockGridPos.x, blockGridPos.y, blockGridPos.z, hashIndex(blockGridPos));
		}
		return;
	}

	bool emptyVoxel = false;
	bool safeToClear = true;
	int age = currentFrame - hashTable[blockHashIdx].allocatedTime;
	if (age < minAge) {
		// Important corner case: when we had a block in the visible list, but it got deleted in
		// a previous decay pass, and ended up also getting reallocated (and thus the old ID in
		// the visible list was pointing to the wrong thing).
		safeToClear = false;
	}

	if (safeToClear) {
		// The SDF limit it EXPERIMENTAL and enabling it may be to aggressive when applied on a per-voxel basis.;
	        // localVBA[voxelIdx]为要decay的voxel
	        bool isNoisy = (localVBA[voxelIdx].w_depth <= maxWeight);
		if (isNoisy && localVBA[voxelIdx].w_depth > 0) {
			localVBA[voxelIdx].reset();
			emptyVoxel = true;
		}

		if (localVBA[voxelIdx].w_depth == 0) {
			emptyVoxel = true;
		}
	}

	// Count the empty voxels in the block, to determine if it's empty
	// TODO(andrei): Try summing all the weights and empty == weightSum < k (==3-10).
	// voxelPerrBlock为每个block中包含的voxels
	static const int voxelsPerBlock = SDF_BLOCK_SIZE3;
	__shared__ int countBuffer[voxelsPerBlock];
	countBuffer[locId] = static_cast<int>(emptyVoxel);
	__syncthreads();

	// Block-level sum for counting non-empty voxels in this block.
	// 计算该block下的非空的voxels的数量
	blockReduce(countBuffer, voxelsPerBlock, locId);
	__syncthreads();

	int emptyVoxels = countBuffer[0];
	bool emptyBlock = (emptyVoxels == voxelsPerBlock);

	if (locId == 0 && emptyBlock && safeToClear) {
		deleteBlock<TVoxel>(hashTable,
				    blockGridPos,
				    locks,
				    voxelAllocationList,
				    lastFreeBlockId,
				    entriesVisibleType);
	}
}

/// @brief 清除权重小于maxWeight的voxel blocks,同时在‘outBlocksToDellocate’中将在进程中变为空的voxel blocks标记为等待释放(pending deallocation),
///        对可视voxel block表的每一个voxel block进行decay
/// @param localVBA voxel block存储的原始内存
/// @param hashTable 将hashIdx映射到localVBA中地址的哈希表
/// @param visibleBlockPositions 通过blockIdx.x得到hashIdx，即visibleBlockPositions存储了所有voxel block的hashIdx,通过hashTable映射后可得到HashEntry
/// @param minAge 当某帧的age大于minAge,则可以考虑将该帧对应的visible voxel进行decay
/// @param maxWeight 当其对某帧的visible voxel进行decay时，若对应的voxel其depth weight小于或等于maxWeight的时候，该voxel block将会被decay
/// @param voxelAllocationList 该列表包含在localVBA使用或未被使用的voxel blocks的索引
/// @param lastFreeBlockId 'voxelAllocationList'中最后空闲未被使用(last free)的blocks的索引
/// @param locks 当删除block的时候，用来锁住bucket
/// @param currentFrame 被SLAM system处理过的当前帧的索引
/// @param entriesVisibleType 将hash table indices映射到一个enum中，该enum表示特定的voxel block是否可见
template<class TVoxel>
__global__ void decay_device(TVoxel *localVBA,
                             ITMHashEntry *hashTable,
			     int *VisibleBlockPositions,
			     int minAge,
			     int maxWeight,
			     int *voxelAllocationList,
			     int *lastFreeBlockId,
			     int *locks,
			     int currentFrame,
			     uchar *entriesVisibleType)
{
         //The local offset of the voxel in the current block
         //体素在当前voxel block的位置
         int locId = threadIdx.x + threadIdx.y * SDF_BLOCK_SIZE + threadIdx.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	 int entryId = VisibleBlockPositions[blockIdx.x];
	 if (entryId < 0) return;
	 
	 const ITMHashEntry &currentHashEntry = hashTable[entryId];
	 if (currentHashEntry.ptr < 0) return;
	
	///因为blockGridPos是以SDF_BLOCK_SIZE为单位的，简单的说就是其计算方法是以一个block一个block来算的，乘上SDF_BLOCK_SIZE后其单位就是1了
	Vector3i blockGridPos = currentHashEntry.pos.toInt();
	decayVoxel<TVoxel>(blockGridPos, 
	                   locId, 
	                   localVBA, 
	                   hashTable, 
	                   minAge, 
	                   maxWeight, 
	                   voxelAllocationList, 
	                   lastFreeBlockId, 
	                   locks, 
	                   currentFrame,
	                   entriesVisibleType);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash>::PartialDecay(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, 
										const ITMRenderState *renderState,
										const VisibleBlockInfo &visibleBlockInfo,
										int minAge,
										int maxWeight){
  
        ///创建每个scene时分配的这块内存中每个block的索引
        int *voxelAllocationList = scene->localVBA.GetAllocationList();
	///创建一个scene时用来存储voxel block时分配的内存大小，其中voxel block还存储了每个voxel的sdf值
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	
	/// cudaMemeset为local_device分配SDF_BUCKET_NUM个0
	ITMSafeCall(cudaMemset(locks_device, 0, SDF_BUCKET_NUM*sizeof(int)));
	
	dim3 voxelBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(static_cast<uint32_t>(visibleBlockInfo.count));
	decay_device<TVoxel> <<< gridSize, voxelBlockSize >>> (
	         localVBA,
		 hashTable,
		 visibleBlockInfo.blockCoords->GetData(MEMORYDEVICE_CUDA),
		 minAge,
		 maxWeight,
		 voxelAllocationList,
		 lastFreeBlockId_device,
		 locks_device,
		 frameIdx,
		 ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType());					       
	delete visibleBlockInfo.blockCoords;
}

///@brief 对地图进行正则化
template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash>::Decay(
                         ITMScene<TVoxel,ITMVoxelBlockHash> *scene,
			 const ITMRenderState* renderState,
			 int maxWeight,
			 int minAge,
			 bool forceAllVoxels){
	int oldLastFreeBlockId = scene->localVBA.lastFreeBlockId;
	
	ITMSafeCall(cudaMemcpy(lastFreeBlockId_device, &(scene->localVBA.lastFreeBlockId), 1*sizeof(int), cudaMemcpyHostToDevice));
	
	///frameVisibleBlocks为一个queue，若其大小大于minAge,则选取最老的VisibleBlockInfo进行decay，decay后将其弹出队列
	if(static_cast<long>(frameVisibleBlocks.size()) > minAge){
	  //只是进行对‘minAge’之前帧的voxel blocks进行操作
	  VisibleBlockInfo visible = frameVisibleBlocks.front();
	  frameVisibleBlocks.pop();
	  //当要decay的当前帧的可见voxel blocks大于0时，才对该帧进行decay
	  if(visible.count > 0){
	    PartialDecay(scene, renderState, visible, minAge, maxWeight);
	  }
	}
	
	//这确保了ITM “知道” localVBA中已释放的voxel block的情况，我们需要使用它来统计释放了多少体素块
	ITMSafeCall(cudaMemcpy(&(scene->localVBA.lastFreeBlockId), lastFreeBlockId_device, 1*sizeof(int), cudaMemcpyDeviceToHost));
	int freeBlockCount = scene->localVBA.lastFreeBlockId - oldLastFreeBlockId;
        totalDecayedBlockCount += freeBlockCount;
	
	if(freeBlockCount > 0){
	  size_t savings = sizeof(TVoxel)*SDF_BLOCK_SIZE3*freeBlockCount;
	  float savingMb = (savings/1024.0f/1024.0f);
	  printf("Found %d candidate blocks to deallocate with weight [%d] or below and age [%d]."
	         "Saved %.2fMb. \n",
	         freeBlockCount,
	         maxWeight,
	         minAge,
	         savingMb);
	}
	else{
	  printf("Decay process found NO voxel blocks to deallocate.\n");
	} 
}

template<class TVoxel>
size_t ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash>::GetDecayedBlockCount() {
	return static_cast<size_t>(totalDecayedBlockCount);
}

// plain voxel array
template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	memsetKernel<TVoxel>(voxelBlocks_ptr, TVoxel(), numBlocks * blockSize);
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	fillArrayKernel<int>(vbaAllocationList_ptr, numBlocks);
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList)
{
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
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

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo = scene->index.getIndexData();

	dim3 cudaBlockSize(8, 8, 8);
	dim3 gridSize(scene->index.getVolumeSize().x / cudaBlockSize.x, scene->index.getVolumeSize().y / cudaBlockSize.y, scene->index.getVolumeSize().z / cudaBlockSize.z);

	if (scene->sceneParams->stopIntegratingAtMaxW) {
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device < TVoxel, true, false> << <gridSize, cudaBlockSize >> >(localVBA, arrayInfo,
				rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device < TVoxel, true, true> << <gridSize, cudaBlockSize >> >(localVBA, arrayInfo,
				rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
	}
	else
	{
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device < TVoxel, false, false> << <gridSize, cudaBlockSize >> >(localVBA, arrayInfo,
				rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device < TVoxel, false, true> << <gridSize, cudaBlockSize >> >(localVBA, arrayInfo,
				rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
	}
}

// hierarchical hash
template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHHash>::ITMSceneReconstructionEngine_CUDA(void)
{
	int noLevels = ITMVoxelBlockHHash::noLevels;
	ITMSafeCall(cudaMalloc((void**)&allocationTempData_device, sizeof(AllocationTempData)));
	ITMSafeCall(cudaMalloc((void**)&noAllocatedExcessEntries_device, noLevels * sizeof(int)));

	int noTotalEntries = ITMVoxelBlockHHash::noTotalEntries;
	ITMSafeCall(cudaMalloc((void**)&entriesAllocType_device, noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&blockCoords_device, noTotalEntries * sizeof(Vector4s)));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHHash>::~ITMSceneReconstructionEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(allocationTempData_device));
	ITMSafeCall(cudaFree(noAllocatedExcessEntries_device));

	ITMSafeCall(cudaFree(entriesAllocType_device));
	ITMSafeCall(cudaFree(blockCoords_device));
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	memsetKernel<TVoxel>(voxelBlocks_ptr, TVoxel(), numBlocks * blockSize);
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	fillArrayKernel<int>(vbaAllocationList_ptr, numBlocks);
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHHashEntry));
	tmpEntry.ptr = -3;
	ITMHHashEntry *hashEntry_ptr = scene->index.GetEntries();
	memsetKernel<ITMHHashEntry>(hashEntry_ptr, tmpEntry, scene->index.noTotalEntries);
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	for (int listId = 0; listId < SDF_HASH_NO_H_LEVELS; listId++)
	{
		int startPoint = listId * SDF_EXCESS_LIST_SIZE;
		fillArrayKernel<int>(excessList_ptr + startPoint, SDF_EXCESS_LIST_SIZE);
	}

	//scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
	for (int i = 0; i < SDF_HASH_NO_H_LEVELS; i++) scene->index.SetLastFreeExcessListId(i, SDF_EXCESS_LIST_SIZE - 1);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList)
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

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->useSwapping ? scene->globalCache->GetSwapStates(true) : 0;

	int noTotalEntries = scene->index.noTotalEntries;
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();

	int noLevels = ITMVoxelBlockHHash::noLevels;
	int noTotalEntriesPerLevel = ITMVoxelBlockHHash::noTotalEntriesPerLevel;

	dim3 cudaBlockSizeHV(16, 16);
	dim3 gridSizeHV((int)ceil((float)depthImgSize.x / (float)cudaBlockSizeHV.x), (int)ceil((float)depthImgSize.y / (float)cudaBlockSizeHV.y));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntriesPerLevel / (float)cudaBlockSizeAL.x));

	dim3 cudaBlockSizeAL3(256, 1);
	dim3 gridSizeAL3((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));

	dim3 cudaBlockSizeVS(256, 1);
	dim3 gridSizeVS((int)ceil((float)renderState_vh->noVisibleEntries / (float)cudaBlockSizeAL.x));

	float oneOverSmallestBlockSize = 1.0f / (smallestVoxelSize * SDF_BLOCK_SIZE);

	AllocationTempData tempData;
	tempData.noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;
	tempData.noAllocatedExcessEntries = 0; //NOT TO BE USED in HHash
        tempData.noVisibleEntries = 0;
        ITMSafeCall(cudaMemcpy(allocationTempData_device, &tempData, sizeof(AllocationTempData), cudaMemcpyHostToDevice));

	ITMSafeCall(cudaMemcpy(noAllocatedExcessEntries_device, lastFreeExcessListIds, noLevels * sizeof(int), cudaMemcpyHostToDevice));

	ITMSafeCall(cudaMemset(entriesAllocType_device, 0, sizeof(unsigned char)* noTotalEntries));

	if (gridSizeVS.x > 0) setToType3HH << <gridSizeVS, cudaBlockSizeVS >> > (entriesVisibleType, visibleEntryIDs, renderState_vh->noVisibleEntries, hashTable);

	buildHHashAllocAndVisibleType_device << <gridSizeHV, cudaBlockSizeHV >> >(entriesAllocType_device, entriesVisibleType,
		blockCoords_device, depth, invM_d, invProjParams_d, mu, depthImgSize, oneOverSmallestBlockSize, hashTable,
		scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max);

	if (!onlyUpdateVisibleList)
	{
		allocateVoxelBlocksListHHash_device << <gridSizeAL3, cudaBlockSizeAL3 >> >(voxelAllocationList, excessAllocationList, hashTable,
			noTotalEntries, (AllocationTempData*)allocationTempData_device, noAllocatedExcessEntries_device, entriesAllocType_device, entriesVisibleType, blockCoords_device);
	}

	bool useSwapping = scene->useSwapping;
	if (onlyUpdateVisibleList) useSwapping = false;

	for (int level = 0; level < noLevels; ++level) {
		float voxelSize = smallestVoxelSize * (1 << level);
		int levelOffset = level * noTotalEntriesPerLevel;

		if (useSwapping)
			buildVisibleList_device<true> << <gridSizeAL, cudaBlockSizeAL >> >(hashTable + levelOffset, swapStates + levelOffset, noTotalEntriesPerLevel, visibleEntryIDs, (AllocationTempData*)allocationTempData_device, entriesVisibleType + levelOffset, M_d, projParams_d, depthImgSize, voxelSize, levelOffset);
		else
			buildVisibleList_device<false> << <gridSizeAL, cudaBlockSizeAL >> >(hashTable + levelOffset, swapStates + levelOffset, noTotalEntriesPerLevel, visibleEntryIDs, (AllocationTempData*)allocationTempData_device, entriesVisibleType + levelOffset, M_d, projParams_d, depthImgSize, voxelSize, levelOffset);
	}

	if (useSwapping)
	{
		cudaBlockSizeAL = dim3(256, 1);
		gridSizeAL = dim3((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));
		reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries,
			(AllocationTempData*)allocationTempData_device, entriesVisibleType);
	}

	ITMSafeCall(cudaMemcpy(&tempData, allocationTempData_device, sizeof(AllocationTempData), cudaMemcpyDeviceToHost));
	renderState_vh->noVisibleEntries = tempData.noVisibleEntries;
	scene->localVBA.lastFreeBlockId = tempData.noAllocatedVoxelEntries;

	ITMSafeCall(cudaMemcpy(lastFreeExcessListIds, noAllocatedExcessEntries_device, noLevels * sizeof(int), cudaMemcpyDeviceToHost));

	scene->index.SetLastFreeExcessListIds(lastFreeExcessListIds);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float smallestVoxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(renderState_vh->noVisibleEntries);

	if (scene->sceneParams->stopIntegratingAtMaxW)
		integrateIntoSceneH_device<TVoxel,true> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, 
			scene->index.getIndexData(), smallestVoxelSize, mu, maxW);
	else
		integrateIntoSceneH_device<TVoxel,false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, 
			scene->index.getIndexData(), smallestVoxelSize, mu, maxW);
}

// device functions

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *voxelArray, const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	int x = blockIdx.x*blockDim.x+threadIdx.x;
	int y = blockIdx.y*blockDim.y+threadIdx.y;
	int z = blockIdx.z*blockDim.z+threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	
	if (stopMaxW) if (voxelArray[locId].w_depth == maxW) return;
//	if (approximateIntegration) if (voxelArray[locId].w_depth != 0) return;

	pt_model.x = (float)(x + arrayInfo->offset.x) * _voxelSize;
	pt_model.y = (float)(y + arrayInfo->offset.y) * _voxelSize;
	pt_model.z = (float)(z + arrayInfo->offset.z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *visibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	Vector3i globalPos;
	int entryId = visibleEntryIDs[blockIdx.x];

	const ITMHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

	TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (stopMaxW) if (localVoxelBlock[locId].w_depth == maxW) return;
	if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) return;

	pt_model.x = (float)(globalPos.x + x) * _voxelSize;
	pt_model.y = (float)(globalPos.y + y) * _voxelSize;
	pt_model.z = (float)(globalPos.z + z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

template<class TVoxel, bool stopMaxW>
__global__ void integrateIntoSceneH_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *liveEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, 
	Vector4f projParams_d, Vector4f projParams_rgb, const ITMVoxelBlockHHash::IndexData *indexData, float smallestVoxelSize, float mu, int maxW)
{
	Vector3i globalPos;
	int entryId = liveEntryIDs[blockIdx.x];
	const ITMHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	float localVoxelSize = smallestVoxelSize * (1 << ITMVoxelBlockHHash::GetLevelForEntry(entryId));
	globalPos.x = currentHashEntry.pos.x;
	globalPos.y = currentHashEntry.pos.y;
	globalPos.z = currentHashEntry.pos.z;
	globalPos *= SDF_BLOCK_SIZE;

	TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (stopMaxW) if (localVoxelBlock[locId].w_depth == maxW) return;

	pt_model.x = (float)(globalPos.x + x) * localVoxelSize;
	pt_model.y = (float)(globalPos.y + y) * localVoxelSize;
	pt_model.z = (float)(globalPos.z + z) * localVoxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation, TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

/// @brief 检查深度图x,y对应的voxel block是否已经被分配(通过检查hashEntry)，若已经被分配，是否需要将其可见性进行更新
__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, ITMHashEntry *hashTable, float viewFrustum_min,
	float viewFrustum_max)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
		projParams_d, mu, _imgSize, _voxelSize, hashTable, viewFrustum_min, viewFrustum_max);
}

__global__ void buildHHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float oneOverSmallestBlockSize, ITMHHashEntry *hashTable, float viewFrustum_min,
	float viewFrustum_max)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	buildHHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
		projParams_d, mu, _imgSize, oneOverSmallestBlockSize, hashTable, viewFrustum_min, viewFrustum_max);
}

__global__ void setToType3(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noVisibleEntries - 1) return;
	entriesVisibleType[visibleEntryIDs[entryId]] = 3;
}

__global__ void setToType3HH(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries, const ITMHashEntry *hashTable)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noVisibleEntries - 1) return;

	int ptr = hashTable[visibleEntryIDs[entryId]].ptr;
	// blocks might have disappeared due to splitting and merging
	if (ptr < -1) entriesVisibleType[visibleEntryIDs[entryId]] = 0;
	else entriesVisibleType[visibleEntryIDs[entryId]] = 3;
}

///@brief 根据voxelAllocationList存储的状态对targetIdx进行分配
///@param voxelAllocationList 存储着对应的空间点在ordered list或者excess list是否需要分配或者分配的状态
///@param blockCoords 对应的空间点三维坐标，需要对hashEntry中的成员变量pos进行赋值 
__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx, exlIdx;

	switch (entriesAllocType[targetIdx])
	{
	case 1: //needs allocation, fits in the ordered list
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

		if (vbaIdx >= 0) //there is room in the voxel block array
		{
			Vector4s pt_block_all = blockCoords[targetIdx];

			ITMHashEntry hashEntry;
			hashEntry.pos.x = pt_block_all.x; 
			hashEntry.pos.y = pt_block_all.y; 
			hashEntry.pos.z = pt_block_all.z;
			hashEntry.ptr = voxelAllocationList[vbaIdx];
			hashEntry.offset = 0;
			
			//在ordered list进行hash Entry的存储
			hashTable[targetIdx] = hashEntry;
		}
		break;

	case 2: //needs allocation in the excess list
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

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
			entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible
		}

		break;
	}
}

__global__ void allocateVoxelBlocksListHHash_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, int *noAllocatedExcessEntries, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx, exlIdx;
	ITMHashEntry hashEntry;

	switch (entriesAllocType[targetIdx])
	{
	case 1: //needs allocation, fits in the ordered list
	case 3: //needs allocation, reactivate old entry
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

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

		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		exlIdx = atomicSub(&noAllocatedExcessEntries[level], 1);

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

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesVisibleType)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx;
	int hashEntry_ptr = hashTable[targetIdx].ptr;

	if (entriesVisibleType[targetIdx] > 0 && hashEntry_ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
	}
}

template<bool useSwapping>
__global__ void buildVisibleList_device(ITMHashEntry *hashTable, ITMHashSwapState *swapStates, int noTotalEntries,
	int *visibleEntryIDs, AllocationTempData *allocData, uchar *entriesVisibleType, 
	Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize, int offsetToAdd)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	unsigned char hashVisibleType = entriesVisibleType[targetIdx];
	const ITMHashEntry & hashEntry = hashTable[targetIdx];

	if (hashVisibleType == 3)
	{
		bool isVisibleEnlarged, isVisible;

		if (useSwapping)
		{
			checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
			if (!isVisibleEnlarged) hashVisibleType = 0;
		} else {
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
			if (!isVisible) hashVisibleType = 0;
		}
		entriesVisibleType[targetIdx] = hashVisibleType;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	if (useSwapping)
	{
		if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, &allocData->noVisibleEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = targetIdx + offsetToAdd;
	}

#if 0
	// "active list": blocks that have new information from depth image
	// currently not used...
	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType == 1, noActiveEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) activeEntryIDs[offset] = targetIdx + offsetToAdd;
	}
#endif
}




template class ITMLib::Engine::ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

