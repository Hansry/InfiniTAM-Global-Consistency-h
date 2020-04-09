// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "ITMSceneReconstructionEngine.h"
#include "ITMVisualisationEngine.h"
#include "ITMSwappingEngine.h"
#include "ITMVoxelBlockOpEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		template<class TVoxel, class TIndex>
		class ITMDenseMapper
		{
		private:
			ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
			ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;
			ITMVoxelBlockOpEngine<TVoxel,TIndex> *voxelBlockOpEngine;

		public:
		  
		        /*
			 * 将ResetScene从非const改成const,即在后面加上了const (hansry)
			 * 
			 */
			void ResetScene(ITMScene<TVoxel,TIndex> *scene) const;

			/// Process a single frame
			void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState_live);
			
			/// @brief 反融合操作函数
			void DeProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState);

			/// Update the visible list (this can be called to update the visible list when fusion is turned off)
			void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState);

			/// @brief 移除权重小于maxWeight和年龄大于minAge的voxel,这些voxel对应的block将会被释放,
			///        如果foceAllVoxels=true,那么该操作将会在地图的所有voxels上进行，这可能会非常的慢；
			///        反之，系统只会在可见的voxel列表中进行decay,虽然这不是100%准确，但是对于大场景的地图来说，速度要快几个数量级。
			void Decay(ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState, int maxWeight, int minAge, bool forceAllVoxels = false);
			
			size_t GetDecayedBlockCount() const;
			
			void SetFusionWeightParams(const WeightParams &weightParams) {
				sceneRecoEngine->SetFusionWeightParams(weightParams);
			}
			
			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			
			ITMSwappingEngine<TVoxel, TIndex> *GetSwappingEngine(void) const{
			  return swappingEngine;
			}
			
			explicit ITMDenseMapper(const ITMLibSettings *settings);
			~ITMDenseMapper();
		};
	}
}

