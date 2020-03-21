// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSceneReconstructionEngine.h"
#include <queue>
#include <map>

namespace ITMLib
{
	namespace Engine
	{
	        struct VisibleBlockInfo {
		   size_t count;
		   size_t frameIdx;
		   ORUtils::MemoryBlock<int> *blockCoords;
		};
		
		struct DefusionVisibleBlockInfo{
		   size_t count;
		   ORUtils::MemoryBlock<int> *blockCoords;
		};
		
		
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine_CUDA : public ITMSceneReconstructionEngine < TVoxel, TIndex >
		{};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			void *allocationTempData_device;
			void *allocationTempData_host;
			unsigned char *entriesAllocType_device;
			Vector4s *blockCoords_device;
			
			//用来保存最近可见的block Id的列表，用于decay
			std::queue<VisibleBlockInfo> frameVisibleBlocks;
			std::map<double, DefusionVisibleBlockInfo> mDefusionBlockDataBase;
			
			int *lastFreeBlockId_device;
			//用来防止从hash table删除元素时造成的数据竞争（data races）
			int *locks_device;
			// Used by the full-volume decay mode
			Vector4s *allocatedBlockPositions_device;
			
			long totalDecayedBlockCount = 0L;
			size_t frameIdx = 0;
			
			///@brief 在可见列表上进行voxel decay,也就是在frameVisibleBlocks上
			///       Runs a voxel decay process on the blocks specified in 'VisibleBlockInfo'
			void PartialDecay(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
			                  const ITMRenderState *renderState,
		                          const VisibleBlockInfo &visibleBlockInfo,
		                          int minAge,
		                          int maxWeight);
			
			/*
			/// \brief 在整个地图上进行voxel decay
			void FullDecay(ITMScene<TVoxel,ITMVoxelBlockHash>* scene,
			               const ITMRenderState* renderState,
		                       int minAge,
		                       int maxWeight);
                        */
			
		public:
			void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);
			
			void DeIntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState);
			
			void Decay(ITMScene<TVoxel,ITMVoxelBlockHash> *scene,
			           const ITMRenderState *renderState,
	                           int maxWeight,
	                           int minAge,
	                           bool forceAllVoxels) override;
				   
		        size_t GetDecayedBlockCount() override;

			ITMSceneReconstructionEngine_CUDA(void);
			~ITMSceneReconstructionEngine_CUDA(void);
		};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHHash> : public ITMSceneReconstructionEngine<TVoxel, ITMVoxelBlockHHash>
		{
		private:
			void *allocationTempData_device;
			int *noAllocatedExcessEntries_device;

			unsigned char *entriesAllocType_device;
			Vector4s *blockCoords_device;

		public:
			void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMRenderState *renderState);
			
			ITMSceneReconstructionEngine_CUDA(void);
			~ITMSceneReconstructionEngine_CUDA(void);
		};

		
		// Reconstruction engine for plain voxel arrays (vanilla Kinectfusion-style).
		template<class TVoxel>
		class ITMSceneReconstructionEngine_CUDA<TVoxel, ITMPlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, ITMPlainVoxelArray >
		{
		public:
			void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);
			
			void Decay(ITMScene<TVoxel,ITMPlainVoxelArray> *scene,
			           const ITMRenderState *renderState,
	                           int maxWeight,
	                           int minAge,
	                           bool forceAllVoxels) override;
				   
			size_t GetDecayedBlockCount() override;
		};
	}
}
