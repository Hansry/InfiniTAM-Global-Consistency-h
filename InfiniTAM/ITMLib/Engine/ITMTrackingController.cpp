// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) {
	  tracker->TrackCamera(trackingState, view);
	}
	trackingState->requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
}

/// @brief 通过光线投影绘制当前视角的可见地图的信息，便于跟踪和显示
void ITMTrackingController::Prepare(ITMTrackingState *trackingState, const ITMScene<ITMVoxel, ITMVoxelIndex> *scene, const ITMView *view, ITMRenderState *renderState)
{
	//render for tracking

	if (settings->trackerType == ITMLibSettings::TRACKER_COLOR)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		///  由于rendering Block在当前坐标系位置已知，可以选取voxelBlock的8个corner深度的
                ///  最小最大值来作为投影到成像平面后对应的16x16小块的像素对应的深度范围，方便在对像素进行raycast的时候缩短其搜索范围
		visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), renderState);
		visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
		trackingState->age_pointCloud = 0;
	}
	else
	{
	        // 同样的，方便在对像素进行raycast的时候缩短其搜索范围
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

		if (trackingState->requiresFullRendering)
		{
		        /// 计算像素对应的场景空间点和法线
			visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
			/// 设置当前子地图的pose_pointCloud
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
			if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
			else trackingState->age_pointCloud = 0;
		}
		else
		{
			visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
			trackingState->age_pointCloud++;
		}
	}
}
