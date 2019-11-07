// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "ITMPose.h"
#include "ITMPointCloud.h"
#include "ITMScene.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Stores some internal variables about the current tracking
		    state, most importantly the camera pose
		*/
		class ITMTrackingState
		{
		public:
			/** @brief
			    Excerpt of the scene used by the tracker to align
			    a new frame.

			    This is usually the main result generated by the
			    raycasting operation in a ITMLib::Engine::ITMSceneReconstructionEngine.
			*/
			ITMPointCloud *pointCloud;

			/// The pose used to generate the point cloud.
			ITMPose *pose_pointCloud;

			int age_pointCloud;

			/// Current pose of the depth camera.
			ITMPose *pose_d;
			
			enum TrackingResult
		        {
			   TRACKING_GOOD = 2,
			   TRACKING_POOR = 1,
			   TRACKING_FAILED = 0
		        } trackerResult;

			bool requiresFullRendering;

			bool TrackerFarFromPointCloud(void) const
			{
				// if no point cloud exists, yet
				if (age_pointCloud < 0) return true;
				// if the point cloud is older than n frames
				if (age_pointCloud > 5) return true;

				Vector3f cameraCenter_pc = -1.0f * (pose_pointCloud->GetR().t() * pose_pointCloud->GetT());
				Vector3f cameraCenter_live = -1.0f * (pose_d->GetR().t() * pose_d->GetT());

				Vector3f diff3 = cameraCenter_pc - cameraCenter_live;

				float diff = diff3.x * diff3.x + diff3.y * diff3.y + diff3.z * diff3.z;

				// if the camera center has moved by more than a threshold
				if (diff > 0.0005f) return true;

				return false;
			}

			ITMTrackingState(Vector2i imgSize, MemoryDeviceType memoryType)
			{
				this->pointCloud = new ITMPointCloud(imgSize, memoryType);

				this->pose_d = new ITMPose();
				this->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

				this->age_pointCloud = -1;
				this->pose_pointCloud = new ITMPose();
				this->pose_pointCloud->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

				requiresFullRendering = true;
			}

			~ITMTrackingState(void)
			{
				delete pointCloud;
				delete pose_d;
				delete pose_pointCloud;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTrackingState(const ITMTrackingState&);
			ITMTrackingState& operator=(const ITMTrackingState&);
		};
	}
}
