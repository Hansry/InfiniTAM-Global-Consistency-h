// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSwappingEngine_CPU.h"
#include "../../DeviceAgnostic/ITMSwappingEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSwappingEngine_CPU(void)
{
}

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSwappingEngine_CPU(void)
{
}

/// @brief 从host memory拷贝指定的数据（其实就是swapStates[entryId].state==1对应的entryId）到local memory
template<class TVoxel>
int ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	/// 0 - most recent data is on host, data not currently in active memory
	/// 1 - data both on host and in active memory, information has not yet been combined
	/// 2 - most recent data is in active memory, should save this data back to host at some point
	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	int noTotalEntries = globalCache->noTotalEntries;

	int noNeededEntries = 0;
	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;
		if (swapStates[entryId].state == 1)
		{
		        //存储需要从host拷贝内容到device的entryId, host-->device
			neededEntryIDs_local[noNeededEntries] = entryId;
			noNeededEntries++;
		}
	}

	// would copy neededEntryIDs_local into neededEntryIDs_global here
	// 由于neededEntryIDs_local和neededEntryIDs_global指向了同一块内存地址，因此可以通过neededEntryIDs_global数组进行遍历得到
	// neededEntryIDs_local存储的需要从host->device的entries间接将neededEntryIDs_local拷贝给了neededEntryIDs_global
	if (noNeededEntries > 0)
	{
	        //syncedVoxelBlocks_global应该是存储了一个block中的Tvoxel数据类型的数据（8x8x8=512）
		memset(syncedVoxelBlocks_global, 0, noNeededEntries * SDF_BLOCK_SIZE3 * sizeof(TVoxel));
		memset(hasSyncedData_global, 0, noNeededEntries * sizeof(bool));
		for (int i = 0; i < noNeededEntries; i++)
		{
			int entryId = neededEntryIDs_global[i];

			if (globalCache->HasStoredData(entryId))
			{
				hasSyncedData_global[i] = true;
				//以globalCache->GetStoredVoxelBlock(entryId)为首地址，拷贝SDF_BLOCK_SIZE3*sizeof(Tvoxel)内存大小的数据到syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3
				//syncedVoxelBlocks_global为指针，通过移动指针对globalcache中的成员变量syncedVoxelBlocks_host进行赋值
				memcpy(syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3, globalCache->GetStoredVoxelBlock(entryId), SDF_BLOCK_SIZE3 * sizeof(TVoxel));
			}
		}
	}

	// would copy syncedVoxelBlocks_global and hasSyncedData_global and syncedVoxelBlocks_local and hasSyncedData_local here

	return noNeededEntries;
}

/// @brief 将从global memory(host)拷贝的数据融合到local memory(device)进行融合
template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashEntry *hashTable = scene->index.GetEntries();

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);
	
	//调用GetSyncedVoxelBlocks得到从host拷贝过来的数据，并将数据融合进device存储的数据(active memory)
	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

	int noNeededEntries = this->LoadFromGlobalMemory(scene);

	int maxW = scene->sceneParams->maxW;

	for (int i = 0; i < noNeededEntries; i++)
	{
	         //存储需要从host拷贝内容到device的entryId, host-->device
		int entryDestId = neededEntryIDs_local[i];

		if (hasSyncedData_local[i])
		{
		        //从host memory交换到device memory(active memory)，每次交换的数据为一个voxel block,包含SDF_BLOCK_SIZE3个Tvoxel数据类型的数据
			TVoxel *srcVB = syncedVoxelBlocks_local + i * SDF_BLOCK_SIZE3;
			//将来自host memory的数据拷贝到device memory指定的位置
			TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * SDF_BLOCK_SIZE3;

			//遍历voxel blocks中每一个Tvoxel数据单元
			for (int vIdx = 0; vIdx < SDF_BLOCK_SIZE3; vIdx++)
			{
			    CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[vIdx], dstVB[vIdx], maxW);
			}
		}

		swapStates[entryDestId].state = 2;
	}
}

/// @brief 将当前处于active memory（device memory）但是在当前视角不可见的数据存到gblobalMemory中（host memory）
template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	ITMHashEntry *hashTable = scene->index.GetEntries();

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();

	int noTotalEntries = globalCache->noTotalEntries;
	
	int noNeededEntries = 0;
	
	int noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;

	for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++)
	{
		if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;

		int localPtr = hashTable[entryDestId].ptr;
		
		ITMHashSwapState &swapState = swapStates[entryDestId];

		//如果该voxel block满足三个条件，即将其从active memory(device memory)--> host memory
		//1.该voxel block处于active memory
		//2.指向该voxel block的entries成员变量pt>=0,意味着在VBA有实际存储的数据
		//3.该voxel block在当前视角下不可见 (主要还是因为这个原因)
		if (swapState.state == 2 && localPtr >= 0)
		{
			TVoxel *localVBALocation = localVBA + localPtr * SDF_BLOCK_SIZE3;

			neededEntryIDs_local[noNeededEntries] = entryDestId;

			hasSyncedData_local[noNeededEntries] = true;
			//将需要swapped的数据拷贝到syncedVoxelBlocks_local中
			memcpy(syncedVoxelBlocks_local + noNeededEntries * SDF_BLOCK_SIZE3, localVBALocation, SDF_BLOCK_SIZE3 * sizeof(TVoxel));

			//将swapped后的数据状态改为0，意味着该数据在host memory中
			swapStates[entryDestId].state = 0;

			int vbaIdx = noAllocatedVoxelEntries;
			if (vbaIdx < SDF_BUCKET_NUM - 1)
			{
				noAllocatedVoxelEntries++;
				
				voxelAllocationList[vbaIdx + 1] = localPtr;
				//-1表示被swapped out
				hashTable[entryDestId].ptr = -1;

				for (int i = 0; i < SDF_BLOCK_SIZE3; i++) {
				  localVBALocation[i] = TVoxel();
				}
			}

			noNeededEntries++;
		}
	}

	scene->localVBA.lastFreeBlockId = noAllocatedVoxelEntries;

	// would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

	if (noNeededEntries > 0)
	{
		for (int entryId = 0; entryId < noNeededEntries; entryId++)
		{
			if (hasSyncedData_global[entryId]){
			    //syncedVoxelBlocks内存中的block数据转换到storedVoxelBlocks中
			    globalCache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * SDF_BLOCK_SIZE3);
			}	
		}
	}
}

/// @brief 将当前处于active memory（device memory）但是在当前视角不可见的数据存到gblobalMemory中（host memory）
template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	ITMHashEntry *hashTable = scene->index.GetEntries();
	uchar *entriesVisibleType = ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType();

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();

	int noTotalEntries = globalCache->noTotalEntries;
	
	int noNeededEntries = 0;
	
	/*  voxelAllocationList(如下占用了整个内存)
	 *  ________________________________________________________________
	 * |||||||||||||||||||||||||||||||||||||||||||______________________|
	 *   　(这里的竖线为已经分配的，而空白处为未分配的，LastFreeBlockId指向已分配的最后一个地址)
	 */
	int noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;

	for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++)
	{
		if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;

		int localPtr = hashTable[entryDestId].ptr;
		ITMHashSwapState &swapState = swapStates[entryDestId];

		//如果该voxel block满足三个条件，即将其从active memory(device memory)--> host memory
		//1.该voxel block处于active memory
		//2.指向该voxel block的entries成员变量pt>=0,意味着在VBA有实际存储的数据
		//3.该voxel block在当前视角下不可见 (主要还是因为这个原因)
		if (swapState.state == 2 && localPtr >= 0 && entriesVisibleType[entryDestId] == 0)
		{
			TVoxel *localVBALocation = localVBA + localPtr * SDF_BLOCK_SIZE3;

			neededEntryIDs_local[noNeededEntries] = entryDestId;

			hasSyncedData_local[noNeededEntries] = true;
			//将需要swapped的数据拷贝到syncedVoxelBlocks_local中
			memcpy(syncedVoxelBlocks_local + noNeededEntries * SDF_BLOCK_SIZE3, localVBALocation, SDF_BLOCK_SIZE3 * sizeof(TVoxel));

			//将swapped后的数据状态改为0，意味着该数据在host memory中
			swapStates[entryDestId].state = 0;

			int vbaIdx = noAllocatedVoxelEntries;
			if (vbaIdx < SDF_BUCKET_NUM - 1)
			{
				noAllocatedVoxelEntries++;
				
				//这句没看懂，为啥还要分配localPtr呢？？
				voxelAllocationList[vbaIdx + 1] = localPtr;
				//-1表示被swapped out
				hashTable[entryDestId].ptr = -1;

				for (int i = 0; i < SDF_BLOCK_SIZE3; i++) {
				  localVBALocation[i] = TVoxel();
				}
			}

			noNeededEntries++;
		}
	}

	scene->localVBA.lastFreeBlockId = noAllocatedVoxelEntries;

	// would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

	if (noNeededEntries > 0)
	{
		for (int entryId = 0; entryId < noNeededEntries; entryId++)
		{
			if (hasSyncedData_global[entryId]){
			    //syncedVoxelBlocks内存中的block数据转换到storedVoxelBlocks中
			    globalCache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * SDF_BLOCK_SIZE3);
			}	
		}
	}
}

template class ITMLib::Engine::ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
