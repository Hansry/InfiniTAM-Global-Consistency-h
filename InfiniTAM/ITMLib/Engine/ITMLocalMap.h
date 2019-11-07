// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "../../ITMLib/Engine/ITMVisualisationEngine.h"
#include "../../ITMLib/Objects/ITMRenderState.h"
#include "../../ITMLib/Objects/ITMScene.h"
#include "../../ITMLib/Objects/ITMTrackingState.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"

namespace ITMLib {
  namespace Engine {
	struct ITMPoseConstraint
	{
	public:
		ITMPoseConstraint(void)
		{
			accu_num = 0;
		}
		
		/// @note 这里的AddObervation的计算表示有点奇怪，为啥可以直接对T进行相加？？？难道是重写了operator？？
		void AddObservation(const ITMLib::Objects::ITMPose & relative_pose, int weight = 1)
		{
			Matrix4f tmp = accu_poses.GetM() * (float)accu_num + relative_pose.GetM() * (float)weight;
			accu_num += weight;
			accu_poses.SetM(tmp / (float)accu_num);
			accu_poses.Coerce();
			//accu_poses = (accu_poses * (float)accu_num + relative_pose)/(float)(accu_num+1);
			accu_num++;
		}

		ITMLib::Objects::ITMPose GetAccumulatedObservations(void) const { return accu_poses; }
		int GetNumAccumulatedObservations(void) const { return accu_num; }

	private:
		ITMLib::Objects::ITMPose accu_poses;
		int accu_num;
	};

	///@note int为与当前地图具有关联的子地图在allData的id
	typedef std::map<int, ITMPoseConstraint> ConstraintList;

	class ITMLocalMap
	{
	public:
		ITMScene<ITMVoxel, ITMVoxelIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;
		ConstraintList relations;
		ITMLib::Objects::ITMPose estimatedGlobalPose;

		ITMLocalMap(const ITMLibSettings *settings, const IITMVisualisationEngine *visualisationEngine, const Vector2i & trackedImageSize)
		{
			MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&settings->sceneParams, settings->useSwapping, memoryType);
			renderState = visualisationEngine->CreateRenderState(trackedImageSize);
			trackingState = new ITMTrackingState(trackedImageSize, memoryType);
		}
		~ITMLocalMap(void)
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}
	};
  } //namespace Engine
} //namespace ITMLib

