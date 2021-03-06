// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSceneReconstructionEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine_CPU : public ITMSceneReconstructionEngine < TVoxel, TIndex >
		{};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
		{
		protected:
			ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
			ORUtils::MemoryBlock<Vector4s> *blockCoords;

		public:
			void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool isDefusion = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);
			
			void DeIntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState) override;
			
		        void Decay(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		                   const ITMRenderState *renderState,
			           int maxWeight, int minAge, bool forceAllVoxels) override;
				   
		        void DecayDefusionPart(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
		                   const ITMRenderState *renderState,
			           int maxWeight, int minAge, bool forceAllVoxels) override;				   
			
			void SlideWindow(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
			                 const ITMRenderState *renderState,
			                 int maxAge) override;
					 
			void SlideWindowDefusionPart(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
			                 const ITMRenderState *renderState,
			                 int maxAge, int maxSize) override;			
                        
			size_t GetDecayedBlockCount() override;

			ITMSceneReconstructionEngine_CPU(void);
			~ITMSceneReconstructionEngine_CPU(void);
		};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHHash> : public ITMSceneReconstructionEngine<TVoxel,ITMVoxelBlockHHash>
		{
		private:
			unsigned char *entriesAllocType;
			Vector4s *blockCoords;

		public:
			void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHHash> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, 
						    const ITMTrackingState *trackingState, const ITMRenderState *renderState, 
			                            bool onlyUpdateVisibleList = false, bool isDefusion = false);

			void IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, 
						const ITMTrackingState *trackingState, const ITMRenderState *renderState);

			ITMSceneReconstructionEngine_CPU(void);
			~ITMSceneReconstructionEngine_CPU(void);
		};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, ITMPlainVoxelArray >
		{
		public:
			void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool isDefusion = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);

			ITMSceneReconstructionEngine_CPU(void);
			~ITMSceneReconstructionEngine_CPU(void);
		};
	}
}
