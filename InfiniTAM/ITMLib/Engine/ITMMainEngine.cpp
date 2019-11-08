// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

using namespace ITMLib::Engine;


// loop closure global adjustment runs on a separate thread
static const bool separateThreadGlobalAdjustment = true;


/* 　（有待理解）
 * - 每当添加一个新的局部场景时，添加到“to be established 3D relations”列表中
 * - 当检测到重定位时，将其添加到相同的列表中，保留3D relation关系上的任何现有信息
 * 
 * - 建立所有的3D relations:
 * - 尝试在俩个场景进行跟踪，
 * - 如果跟踪成功了，则添加到new candidates列表中
 * - 如果在超过n_reloctrialframes帧的情况下仍然小于n_overlap个"new candidates",则不将该3D relations添加到new canidadates列表中
 * - 如果至少存在n_overlap 个"new candidates":尝试计算3D relation,根据旧信息进行加权， 如果外点率小于p_relation_outliers且至少有n_overlap内点，则建立联系成功
 */

// - whenever a new local scene is added, add to list of "to be established 3D relations"
// - whenever a relocalisation is detected, add to the same list, preserving any existing information on that 3D relation
//
// - for all 3D relations to be established :
// - attempt tracking in both scenes
// - if success, add to list of new candidates
// - if less than n_overlap "new candidates" in more than n_reloctrialframes frames, discard
// - if at least n_overlap "new candidates" :
// - try to compute 3D relation, weighting old information accordingly
// - if outlier ratio below p_relation_outliers and at least n_overlap inliers, success

struct TodoListEntry {
	TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare)
		: dataId(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), preprepare(false) {}
	TodoListEntry(void) {}
	//dataId为在active map中子地图的index
	int dataId;
	bool track;
	bool fusion;
	bool prepare;
	bool preprepare;
};

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	meshingEngine = NULL;
	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>;
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

//	mesh = NULL;
//	if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

        SpecificLocalMap = new ITMLocalMap(settings, visualisationEngine, trackedImageSize);	
	renderState_freeview = NULL; //will be created by the visualisation engine

        imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, SpecificLocalMap->scene);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
	denseMapper->ResetScene(SpecificLocalMap->scene);
	
	mapManager = new ITMVoxelMapGraphManager(settings, visualisationEngine, denseMapper, trackedImageSize);
	mActiveDataManger = new ITMActiveMapManager(mapManager);
	
	///初始化的时候将创建的LocalMap设为PrimaryLocalMap
	mActiveDataManger->initiateNewLocalMap(true);
	mGlobalAdjustmentEngine = new ITMGlobalAdjustmentEngine();
	if(separateThreadGlobalAdjustment){
	  mGlobalAdjustmentEngine->startSeparateThread();
	}

	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;
}

ITMMainEngine::~ITMMainEngine()
{
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

ITMLocalMap* ITMMainEngine::GetPrimaryLocalMap(void) const{
    int idx = mActiveDataManger->findPrimaryLocalMapIdx();
    if(idx<0) {
      idx = 0;
    }
    return mapManager->getLocalMap(idx);
}

ITMTrackingState* ITMMainEngine::GetTrackingState(void) const {
    int idx = mActiveDataManger->findPrimaryDataIdx();
    if(idx<0) {
      idx = 0;
    }
    return mapManager->getLocalMap(idx)->trackingState;
}


ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, SpecificLocalMap->scene);
	return mesh;
}

/// @brief 保存主子地图的mesh
void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (meshingEngine == NULL) return;
	ITMMesh *mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);
	meshingEngine->MeshScene(mesh, GetPrimaryLocalMap()->scene);
	mesh->WriteSTL(objFileName);
	delete mesh;
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) {
	  viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);
	}
	else {
	  viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);
	}
	if (!mainProcessingActive) return;

	// tracking
	trackingController->Track(SpecificLocalMap->trackingState, view);

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, SpecificLocalMap->trackingState, SpecificLocalMap->scene, SpecificLocalMap->renderState);

	// raycast to renderState_live for tracking and free visualisation
	trackingController->Prepare(SpecificLocalMap->trackingState, SpecificLocalMap->scene, view, SpecificLocalMap->renderState);
}

Vector2i ITMMainEngine::GetImageSize(void) const
{
	return SpecificLocalMap->renderState->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = SpecificLocalMap->renderState->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOURCODED:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOURCODED) type = IITMVisualisationEngine::RENDER_COLOURCODED;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(SpecificLocalMap->scene, out->noDims);

		visualisationEngine->FindVisibleBlocks(SpecificLocalMap->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(SpecificLocalMap->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(SpecificLocalMap->scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }
