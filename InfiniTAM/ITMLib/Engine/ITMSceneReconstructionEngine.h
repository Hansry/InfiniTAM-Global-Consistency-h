// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
	        //用来判断是否对测量进行加权
	        struct WeightParams {
		   bool depthWeighting = false;
		   int maxNewW = 10;
		   int maxDistance = 100;
		};
		
		/** \brief
		    Interface to engines implementing the main KinectFusion
		    depth integration process.

		    These classes basically manage
		    an ITMLib::Objects::ITMScene and fuse new image information
		    into them.
		*/
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine
		{
		private:
		        WeightParams fusionWeightParams;
		       
		public:
			/** Clear and reset a scene to set up a new empty
			    one.
			*/
			virtual void ResetScene(ITMScene<TVoxel, TIndex> *scene) = 0;

			/** Given a view with a new depth image, compute the
			    visible blocks, allocate them and update the hash
			    table so that the new image data can be integrated.
			*/
			virtual void AllocateSceneFromDepth(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool isDefusion = false) = 0;
				
			virtual void Decay(ITMScene<TVoxel,TIndex> *scene,
			                   const ITMRenderState *renderState,
		                           int maxWeight,
		                           int minAge,
		                           bool forceAllVoxels) = 0;
			
			virtual void DecayDefusionPart(ITMScene<TVoxel,TIndex> *scene,
			                   const ITMRenderState *renderState,
		                           int maxWeight,
		                           int minAge,
		                           bool forceAllVoxels) = 0;			
					   
			virtual void SlideWindow(ITMScene<TVoxel, TIndex> *scene,
			                         const ITMRenderState *renderState,
			                         int maxAge) = 0;
						 
			
			virtual void SlideWindowDefusionPart(ITMScene<TVoxel, TIndex> *scene,
			                         const ITMRenderState *renderState,
			                         int maxAge, int maxSize) = 0;
						 
			///@brief 返回被decayed掉（释放掉）的voxel block的个数
			virtual size_t GetDecayedBlockCount() = 0;
			
			virtual void SetFusionWeightParams(const WeightParams &weightParams) {
				this->fusionWeightParams = weightParams;
			}
			
			WeightParams GetFusionWeightParams() {
			      return fusionWeightParams;
			}

			/** Update the voxel blocks by integrating depth and
			    possibly colour information from the given view.
			*/
			virtual void IntegrateIntoScene(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState) = 0;
				
			///@brief 反融合操作
			virtual void DeIntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState) = 0;

			ITMSceneReconstructionEngine(void) { }
			virtual ~ITMSceneReconstructionEngine(void) { }
		};
	}
}
