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
	/// useApproximateRaycast策略是当相机移动距离不是特别大的时候，是否利用前n帧raycast的深度与当前的深度做icp计算位姿
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
		/// 通过raycast生成当前帧对应的稠密地图表面的3d空间点和颜色信息
		visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
		trackingState->age_pointCloud = 0;
	}
	else
	{
	        // 同样的，方便在对像素进行raycast的时候缩短其搜索范围
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

		if (trackingState->requiresFullRendering)
		{
		        /// 通过raycast计算像素对应的场景空间点和法线,并保存在trackingState->pointCloud->locations中，以m为单位，float类型
			visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
			/// 设置当前子地图的pose_pointCloud
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
			if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
			else trackingState->age_pointCloud = 0;
		}
		else
		{
		        /// 利用当前的renderState的场景表面三维空间点和法线的结果(renderState->raycastResult)，有可能是上一帧raycast得到的，也有可能是前n帧raycast得到的
                        /// 并投影到当前视角下，对于没有对应的三维空间点(fwdProMisssingPoints)的像素,重新进行raycast (在这里所有的三维空间点信息量纲貌似是m/voxelSize),
                        /// 得到所有像素对应的三维空间点后，进行法线和角度的计算
 			visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
			trackingState->age_pointCloud++;
		}
	}
}
