// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CPU.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

#include <vector>

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
ITMRenderState* ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize) const
{
	return new ITMRenderState(
		imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel>
ITMRenderState_VH* ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize) const
{
	return new ITMRenderState_VH(
		ITMVoxelBlockHash::noTotalEntries, imgSize, scene->sceneParams->viewFrustum_min,scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	ITMRenderState *renderState) const
{
  
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	float voxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	int noVisibleEntries = 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = 0;// = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if (hashEntry.ptr >= 0)
		{
			bool isVisible, isVisibleEnlarged;
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
			hashVisibleType = isVisible;
		}

		if (hashVisibleType > 0)
		{
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
}

///@brief 统计minBlockId和maxBLockId之间可见的voxel block的个数
template<class TVoxel>
int ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CountVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const
{
	const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH*)renderState;

	int noVisibleEntries = renderState_vh->noVisibleEntries;
	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	int ret = 0;
	for (int i = 0; i < noVisibleEntries; ++i) {
	        //visibleEntryIDs[i]为可见性的voxelblock在hashTable（通过scene->index.GetEntries()得到）中的序号,ptr为该voxel block在VBA的位置
		int blockID = scene->index.GetEntries()[visibleEntryIDs[i]].ptr;
		if ((blockID >= minBlockId)&&(blockID <= maxBlockId)) ++ret;
	}

	return ret;
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		//TODO : this could be improved a bit...
		Vector2f & pixel = minmaxData[locId];
		pixel.x = 0.2f;
		pixel.y = 3.0f;
	}
}

/// @brief 由于rendering Block在当前坐标系位置已知，可以选取voxelBlock的8个corner深度的
///        最小最大值来作为投影到成像平面后对应的16x16小块的像素对应的深度范围，方便在对像素进行raycast的时候缩短其搜索范围
template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	float voxelSize = scene->sceneParams->voxelSize;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
		const ITMHashEntry & blockData(scene->index.GetEntries()[visibleEntryIDs[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		//blockData.ptr>=0表示在VBA有实际存在数据
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX), 
			(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
		
		//由于将当前的voxel block从3D投影到2D成像平面上，得到bounding box，且将bouding box划分为多个16x16的小块，requiredNumBlocks为统计有多少个16x16的小块
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		//若超出最大的绘制的个数，则不再继续绘制，MAX_RENDERING_BLOCKS=65536*4
		if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = numRenderingBlocks;
		numRenderingBlocks += requiredNumBlocks;

		CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	// 遍历需要绘制的blocks（在2d平面上是16x16的大小），
	for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
		// fill minmaxData
		const RenderingBlock & b(renderingBlocks[blockNo]);

		//更新要绘制的block对应的16x16小块对应的pixel的最大最小值
		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f & pixel(minmaxData[x + y*imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

template<class TVoxel>
ITMRenderState* ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const Vector2i & imgSize) const
{
	return new ITMRenderState_VH(ITMVoxelBlockHHash::noTotalEntries, imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	const ITMHHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	float smallestVoxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	int noVisibleEntries = 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		int level = ITMVoxelBlockHHash::GetLevelForEntry(targetIdx);
		float voxelSize = smallestVoxelSize * (1 << level);
		unsigned char hashVisibleType = 0;// = entriesVisibleType[targetIdx];
		const ITMHHashEntry &hashEntry = hashTable[targetIdx];

		if (hashEntry.ptr >= 0)
		{
			bool isVisible, isVisibleEnlarged;
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
			hashVisibleType = isVisible;
		}

		if (hashVisibleType > 0)
		{
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; ++y) {
		for (int x = 0; x < imgSize.x; ++x) {
			Vector2f & pixel = minmaxData[x + y*imgSize.x];
			pixel.x = FAR_AWAY;
			pixel.y = VERY_CLOSE;
		}
	}

	float smallestVoxelSize = scene->sceneParams->voxelSize;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
		int blockId = visibleEntryIDs[blockNo];
		int level = ITMVoxelBlockHHash::GetLevelForEntry(blockId);
		float voxelSize = smallestVoxelSize * (1 << level);
		const ITMHashEntry & blockData(scene->index.GetEntries()[blockId]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX), 
			(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = numRenderingBlocks;
		numRenderingBlocks += requiredNumBlocks;

		CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
		// fill minmaxData
		const RenderingBlock & b(renderingBlocks[blockNo]);

		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f & pixel(minmaxData[x + y*imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

/// @brief 对于每个像素而言，调用castRay函数得到每个像素对应的场景表面三维坐标，即pointsRay,同时更新renderState中的entriesVisibleType
/// 由GenericRaycast得到renderState->raycastResult
template<class TVoxel, class TIndex>
static void GenericRaycast(const ITMScene<TVoxel,TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, Vector4f projParams, const ITMRenderState *renderState)
{
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float mu = scene->sceneParams->mu;
	//乘上oneOverVoxelSize意味着将以m为计量单位转换成以voxel为计量单位
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	//pointsRay存储的是当前帧的每个像素对应的场景表面的三维空间点,该三维空间点是基于世界坐标系下的
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
	{
		int y = locId/imgSize.x;
		int x = locId - y*imgSize.x;
		//主要是为了获得当前像素的最小最大深度值，方便对场景表面进行搜索
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(
			pointsRay[locId],
			x, y,
			voxelData,
			voxelIndex,
			invM,
			projParams,
			oneOverVoxelSize,
			mu,
			minmaximg[locId2]
		);
	}
}

/// @brief 绘制当前视角下的raycast image
/// @param scene 已经构建的场景
/// @param pose->GetInvM() 世界坐标系到当前坐标系下的变换矩阵，Twc
/// @param outputImage 光线投影绘制得到的image
template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type)
{
	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	// 由GenericRaycast得到renderState->raycastResult
	GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	//其中outRendering为存储了需要绘制点数组首指针
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	// 获得pointsRay,注意pointsRay是基于世界坐标系下的三维空间点
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME)&&
	    (!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	if ((type == IITMVisualisationEngine::RENDER_COLOURCODED)&&
	    (!TIndex::hasColorCoding)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOURCODED:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			PixelColourcoder<TVoxel, TIndex>::process(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
	}
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib->trafo_rgb_to_depth.calib;
        
	// 由GenericRaycast得到renderState->raycastResult
	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_rgb.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
		renderState->raycastImage->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
		renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
		scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(),
		skipPoints,
		scene->sceneParams->voxelSize,
		imgSize,
		-Vector3f(invM.getColumn(2))
	);
}

///@brief 计算像素对应的场景空间点和法线
template<class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	
	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
        
	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	
	/// pointsMap为存储以m为量纲的raycast深度图和normal，由pointsRay通过函数processPixelICP计算得到
	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	/// pointsRay为得到了当前视角下raycast回来的深度图，不过其量纲并不是以m为单位，而是以m/voxelSize为单位
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	        // true为smooth
		processPixelICP<true>(outRendering, pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

/// @brief 利用当前的renderState的场景表面三维空间点和法线的结果(renderState->raycastResult)，有可能是上一帧raycast得到的，也有可能是前n帧raycast得到的
///        并投影到当前视角下，对于没有对应的三维空间点(fwdProMisssingPoints)的像素,重新进行raycast (在这里所有的三维空间点信息量纲貌似是m/voxelSize),
///        得到所有像素对应的三维空间点后，进行法线和角度的计算
template<class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams.x = 1.0f / invProjParams.x;
	invProjParams.y = 1.0f / invProjParams.y;

	/// 旋转矩阵第三列取负
	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	/// 得到当前的renderState的场景表面三维空间点和法线的结果，有可能是上一帧raycast得到的，也有可能是前n帧raycast得到的
	const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
	float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	renderState->forwardProjection->Clear();

	for (int y = 0; y < imgSize.y; y++) 
	  for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		Vector4f pixel = pointsRay[locId];
                
		// 将像素对应的3D空间点投影到当前视角的Depth坐标系下的像平面上
		int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
		if (locId_new >= 0) {
		  forwardProjection[locId_new] = pixel;
		}
	}

	//统计投影回来的像平面没有三维空间点信息的像素数量
	int noMissingPoints = 0;
	for (int y = 0; y < imgSize.y; y++) 
	  for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		Vector4f fwdPoint = forwardProjection[locId];
		Vector2f minmaxval = minmaximg[locId2];
		float depth = currentDepth[locId];

		if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) && (minmaxval.x < minmaxval.y))
		//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
		{
			fwdProjMissingPoints[noMissingPoints] = locId;
			noMissingPoints++;
		}
	}

	renderState->noFwdProjMissingPoints = noMissingPoints;
    
	//对于没有三维空间点的像素，重新进行raycast从场景表面进行获取
	for (int pointId = 0; pointId < noMissingPoints; pointId++)
	{
		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(forwardProjection[locId], x, y, voxelData, voxelIndex, invM, invProjParams,
			1.0f / scene->sceneParams->voxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	//在填充完像平面对应的三维空间点信息后，对其计算法线和角度
	for (int y = 0; y < imgSize.y; y++) 
	  for (int x = 0; x < imgSize.x; x++)
		processPixelForwardRender<true>(outRendering, forwardProjection, imgSize, x, y, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, outputFloatImage, type);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose,  const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, outputFloatImage, type);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const
{
	GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState) const
{
	GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints) const
{ 
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::ForwardRender(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHHash>::ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	ForwardRender_common(scene, view, trackingState, renderState);
}

/// @brief 得到locations和colours,其中ptsRay由GenericRaycast已经得到，但是量纲为 (m/voxelSize)
/// @param locations 通过raycast得到像素点对应的稠密地图表面的三维空间点
/// @param colours 通过raycast得到像素点对应的稠密地图表面的三维空间点的颜色信息
template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource)
{
	int noTotalPoints = 0;

	for (int y = 0, locId = 0; y < imgSize.y; y++) 
	  for (int x = 0; x < imgSize.x; x++, locId++)
	{
		Vector3f outNormal; float angle; 
		Vector4f pointRay = ptsRay[locId];
		Vector3f point = pointRay.toVector3();
		bool foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (foundPoint) drawPixelGrey(outRendering[locId], angle);
		else outRendering[locId] = Vector4u((uchar)0);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint)
		{
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
			if (tmp.w > 0.0f) { 
			  tmp.x /= tmp.w; 
			  tmp.y /= tmp.w; 
			  tmp.z /= tmp.w; 
			  tmp.w = 1.0f; 
			}
			colours[noTotalPoints] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize; 
			pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize; 
			pt_ray_out.w = 1.0f;
			locations[noTotalPoints] = pt_ray_out;

			noTotalPoints++;
		}
	}

	return noTotalPoints;
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *state, 
									ITMUChar4Image *outputImage, ITMFloatImage *outputFloatImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(scene, pose, intrinsics, state, outputImage, outputFloatImage, type);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState) const
{
	GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash>::CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template class ITMLib::Engine::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
