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

			/// Update the visible list (this can be called to update the visible list when fusion is turned off)
			void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState);

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

