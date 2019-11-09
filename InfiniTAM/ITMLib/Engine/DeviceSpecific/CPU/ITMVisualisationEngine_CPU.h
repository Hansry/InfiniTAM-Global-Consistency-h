// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CPU : public ITMVisualisationEngine < TVoxel, TIndex >
		{
		public:
			explicit ITMVisualisationEngine_CPU(void) { }
			~ITMVisualisationEngine_CPU(void) { }

			void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			int CountVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const {return 1;};
			void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize) const;
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		public:
			explicit ITMVisualisationEngine_CPU(void) { }
			~ITMVisualisationEngine_CPU(void) { }

			void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			int CountVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const {return 1;};
			void FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState_VH* CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize) const;
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash> : public ITMVisualisationEngine<TVoxel,ITMVoxelBlockHHash>
		{
		public:
			explicit ITMVisualisationEngine_CPU(void) { }
			~ITMVisualisationEngine_CPU(void) { }

			void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
				ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			int CountVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const {return 1;};

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const Vector2i & imgSize) const;
		};
	}
}
