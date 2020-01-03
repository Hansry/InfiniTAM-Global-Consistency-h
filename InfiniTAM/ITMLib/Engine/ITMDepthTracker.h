// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Base class for engine performing ICP based depth tracking.
		    A typical example would be the original KinectFusion
		    tracking algorithm.
		*/
		class ITMDepthTracker : public ITMTracker
		{
		private:
			const ITMLowLevelEngine *lowLevelEngine;
			ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

			ITMTrackingState *trackingState; const ITMView *view;

			int *noIterationsPerLevel;
			int noICPLevel;

			float terminationThreshold;

			/// @brief 对深度法线金字塔进行预处理
			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
			bool HasConverged(float *step) const;

			/// @brief 构建raycast和view的深度法线金字塔
			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);
		protected:
			float *distThresh;

			int levelId;
			TrackerIterationType iterationType;

			Matrix4f scenePose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

		public:
		  	/** \brief 
	                其中TrackingState为建好地图的点云或者通过光线投影得到的基于上一帧坐标系下的点云
	                view 包含了当前帧的深度信息、深度不确定信息、RGB信息、法线信息等  */
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);
		};
	}
}
