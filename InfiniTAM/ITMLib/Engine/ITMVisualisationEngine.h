// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		class IITMVisualisationEngine
		{
		public:
			enum RenderImageType
			{
				RENDER_SHADED_GREYSCALE,
				RENDER_COLOUR_FROM_VOLUME,
				RENDER_COLOUR_FROM_NORMAL,
				RENDER_DEPTH_MAP,
				RENDER_COLOURCODED
			};
			
			enum RenderRaycastSelection
		        {
			    RENDER_FROM_NEW_RAYCAST,
			    RENDER_FROM_OLD_RAYCAST,
			    RENDER_FROM_OLD_FORWARDPROJ
		        };

			virtual ~IITMVisualisationEngine(void) {}

			static void DepthToUchar4(ITMUChar4Image *dst, ITMFloatImage *src);
			static void NormalToUchar4(ITMUChar4Image* dst, ITMFloat4Image *src);
			static void WeightToUchar4(ITMUChar4Image *dst, ITMFloatImage *src);

		};

		template<class TIndex> struct IndexToRenderState { typedef ITMRenderState type; };
		template<> struct IndexToRenderState<ITMVoxelBlockHash> { typedef ITMRenderState_VH type; };

		/** \brief
			Interface to engines helping with the visualisation of
			the results from the rest of the library.

			This is also used internally to get depth estimates for the
			raycasting done for the trackers. The basic idea there is
			to project down a scene of 8x8x8 voxel
			blocks and look at the bounding boxes. The projection
			provides an idea of the possible depth range for each pixel
			in an image, which can be used to speed up raycasting
			operations.
			*/
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine : public IITMVisualisationEngine
		{
// 		protected:
// 			const ITMScene<TVoxel, TIndex> *scene;
// 			ITMVisualisationEngine(const ITMScene<TVoxel, TIndex> *scene)
// 			{
// 				this->scene = scene;
// 			}
		public:
			/** Override */
			virtual typename IndexToRenderState<TIndex>::type *CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize) const = 0;
			
			/** Given a scene, pose and intrinsics, compute the
			visible subset of the scene and store it in an
			appropriate visualisation state object, created
			previously using allocateInternalState().
			*/
			virtual void FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				ITMRenderState *renderState) const = 0;

			/** Given scene, pose and intrinsics, create an estimate
			of the minimum and maximum depths at each pixel of
			an image.
			*/
			virtual void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				ITMRenderState *renderState) const = 0;

			/** This will render an image using raycasting. */
			virtual void RenderImage(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState, ITMUChar4Image *outputImage, 
			        ITMFloatImage *outputFloatImage, RenderImageType type = RENDER_SHADED_GREYSCALE) const = 0;

			/** Finds the scene surface using raycasting. */
			virtual void FindSurface(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState) const = 0;

			/** Create a point cloud as required by the
			ITMLib::Engine::ITMColorTracker classes.
			*/
			virtual void CreatePointCloud(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
				ITMRenderState *renderState, bool skipPoints) const = 0;

			/** Create an image of reference points and normals as required by the ITMLib::Engine::ITMDepthTracker classes.
			*/
			///@brief 计算像素对应的场景空间点和法线
			virtual void CreateICPMaps(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
				ITMRenderState *renderState) const = 0;
				
			/** Given a render state, Count the number of visible blocks
		            with minBlockId <= blockID <= maxBlockId .
		        */
		        virtual int CountVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId = 0, 
				 int maxBlockId = SDF_LOCAL_BLOCK_NUM) const = 0;

			/** Create an image of reference points and normals as
			required by the ITMLib::Engine::ITMDepthTracker classes.

			Incrementally previous raycast result.
			*/
			/// @brief 利用当前的renderState的场景表面三维空间点和法线的结果(renderState->raycastResult)，有可能是上一帧raycast得到的，也有可能是前n帧raycast得到的
                        ///        并投影到当前视角下，对于没有对应的三维空间点(fwdProMisssingPoints)的像素,重新进行raycast (在这里所有的三维空间点信息量纲貌似是m/voxelSize),
                        ///        得到所有像素对应的三维空间点后，进行法线和角度的计算
			virtual void ForwardRender(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState) const = 0;

			/** Creates a render state, containing rendering info
			for the scene.
			*/
			//virtual ITMRenderState* CreateRenderState(const ITMScene<TVoxel,TIndex> *scene, const Vector2i & imgSize) const = 0;
		};
	}
}
