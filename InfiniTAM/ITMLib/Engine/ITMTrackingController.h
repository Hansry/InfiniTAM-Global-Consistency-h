// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "ITMTrackerFactory.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMTrackingController
		{
		private:
			const ITMLibSettings *settings;
			const ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex> *visualisationEngine;
			const ITMLowLevelEngine *lowLevelEngine;

			ITMTracker *tracker;

			MemoryDeviceType memoryType;

		public:
			void Track(ITMTrackingState *trackingState, const ITMView *view);
			/// @brief 通过光线投影绘制当前视角的可见地图的信息，便于跟踪和显示
			void Prepare(ITMTrackingState *trackingState, const ITMScene<ITMVoxel, ITMVoxelIndex> *scene, const ITMView *view, ITMRenderState *renderState);

			ITMTrackingController(ITMTracker *tracker, const ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex> *visualisationEngine, const ITMLowLevelEngine *lowLevelEngine,
				const ITMLibSettings *settings)
			{
				this->tracker = tracker;
				this->settings = settings;
				this->visualisationEngine = visualisationEngine;
				this->lowLevelEngine = lowLevelEngine;

				memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			}

			ITMTrackingState *BuildTrackingState(const Vector2i & trackedImageSize) const
			{
				return new ITMTrackingState(trackedImageSize, memoryType);
			}

			static Vector2i GetTrackedImageSize(const ITMLibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
			{
				return settings->trackerType == ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTrackingController(const ITMTrackingController&);
			ITMTrackingController& operator=(const ITMTrackingController&);
		};
	}
}
