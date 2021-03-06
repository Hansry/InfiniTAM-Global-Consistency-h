// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"

struct RenderingBlock {
	Vector2s upperLeft;
	Vector2s lowerRight;
	Vector2f zRange;
};

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.05f
#endif

static const CONSTPTR(int) renderingBlockSizeX = 16;
static const CONSTPTR(int) renderingBlockSizeY = 16;

static const CONSTPTR(int) MAX_RENDERING_BLOCKS = 65536*4;
//static const int MAX_RENDERING_BLOCKS = 16384;
static const CONSTPTR(int) minmaximg_subsample = 8;


/**
 *@brief 将单个BLOCK(具有8个corner)投影到当前视角的二维平面上(3D->2D)，存储其2D平面上upperLeft,lowerRight和zRange(深度范围)
 *@param blockPos voxel blockd的3D坐标，基于世界坐标系下
 *@param pose 当前坐标系到世界坐标系的变换Tcw    
 *@param upperLeft 3D block投影到当前视角成像平面后的左上角坐标
 *@param lowerRight 3D block投影到当前视角成像平面后的右下角坐标
*/
_CPU_AND_GPU_CODE_ inline bool ProjectSingleBlock(const THREADPTR(Vector3s) & blockPos, const THREADPTR(Matrix4f) & pose, const THREADPTR(Vector4f) & intrinsics, 
	const THREADPTR(Vector2i) & imgSize, float voxelSize, THREADPTR(Vector2i) & upperLeft, THREADPTR(Vector2i) & lowerRight, THREADPTR(Vector2f) & zRange)
{
	upperLeft = imgSize / minmaximg_subsample;
	lowerRight = Vector2i(-1, -1);
	zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
	
	//由于一个block有8个corner,将8个corner均投影到成像平面上得到其2D坐标
	for (int corner = 0; corner < 8; ++corner)
	{
		// project all 8 corners down to 2D image
	  	// 以blockPos为其中的一个corner,则其它corner通过该corner进行增减（注意blockPos是以voxel block为单位的，需要转换成以m为单位）
		Vector3s tmp = blockPos;
		tmp.x += (corner & 1) ? 1 : 0;
		tmp.y += (corner & 2) ? 1 : 0;
		tmp.z += (corner & 4) ? 1 : 0;
		
		//乘以SDF_BLOCK_SIZE*voxelSize后将voxel block为单位的corners坐标转成以m为单位的坐标
		Vector4f pt3d(TO_FLOAT3(tmp) * (float)SDF_BLOCK_SIZE * voxelSize, 1.0f);
		//将corner变换到当前帧坐标系下
		pt3d = pose * pt3d;
		if (pt3d.z < 1e-6) continue;

		Vector2f pt2d;
		pt2d.x = (intrinsics.x * pt3d.x / pt3d.z + intrinsics.z) / minmaximg_subsample;
		pt2d.y = (intrinsics.y * pt3d.y / pt3d.z + intrinsics.w) / minmaximg_subsample;

		// remember bounding box, zmin and zmax
		// bounding box: upperLeft保存8个corners最小的坐标，lowerRight保存8个corners最大的坐标，zmin保存8个corners最小的深度值，zmax保存8个corners最大的深度值
		if (upperLeft.x > floor(pt2d.x)) upperLeft.x = (int)floor(pt2d.x);
		if (lowerRight.x < ceil(pt2d.x)) lowerRight.x = (int)ceil(pt2d.x);
		if (upperLeft.y > floor(pt2d.y)) upperLeft.y = (int)floor(pt2d.y);
		if (lowerRight.y < ceil(pt2d.y)) lowerRight.y = (int)ceil(pt2d.y);
		if (zRange.x > pt3d.z) zRange.x = pt3d.z;
		if (zRange.y < pt3d.z) zRange.y = pt3d.z;
	}

	// do some sanity checks and respect image bounds
	if (upperLeft.x < 0) upperLeft.x = 0;
	if (upperLeft.y < 0) upperLeft.y = 0;
	if (lowerRight.x >= imgSize.x) lowerRight.x = imgSize.x - 1;
	if (lowerRight.y >= imgSize.y) lowerRight.y = imgSize.y - 1;
	if (upperLeft.x > lowerRight.x) return false;
	if (upperLeft.y > lowerRight.y) return false;
	//if (zRange.y <= VERY_CLOSE) return false; never seems to happen
	if (zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
	if (zRange.y < VERY_CLOSE) return false;

	return true;
}

/**
 * @brief 将voxel block从3D投影到2D平面得到的bounding box分割成renderingBlockSizeX*renderingBlockSizeY小块
 * @param renderingBlockList 存储需要绘制的block的列表
 * @param offset 当前需要绘制的block在rederingBlockList中的序号
 */
_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(DEVICEPTR(RenderingBlock) *renderingBlockList, int offset,
	const THREADPTR(Vector2i) & upperLeft, const THREADPTR(Vector2i) & lowerRight, const THREADPTR(Vector2f) & zRange)
{
	// split bounding box into 16x16 pixel rendering blocks
	for (int by = 0; by < ceil((float)(1 + lowerRight.y - upperLeft.y) / renderingBlockSizeY); ++by) {
		for (int bx = 0; bx < ceil((float)(1 + lowerRight.x - upperLeft.x) / renderingBlockSizeX); ++bx) {
			if (offset >= MAX_RENDERING_BLOCKS) return;
			//for each rendering block: add it to the list
			DEVICEPTR(RenderingBlock) & b(renderingBlockList[offset++]);
			b.upperLeft.x = upperLeft.x + bx*renderingBlockSizeX;
			b.upperLeft.y = upperLeft.y + by*renderingBlockSizeY;
			b.lowerRight.x = upperLeft.x + (bx + 1)*renderingBlockSizeX - 1;
			b.lowerRight.y = upperLeft.y + (by + 1)*renderingBlockSizeY - 1;
			if (b.lowerRight.x>lowerRight.x) b.lowerRight.x = lowerRight.x;
			if (b.lowerRight.y>lowerRight.y) b.lowerRight.y = lowerRight.y;
			
			//将bounding box划分为16x16小块之后，将bounding box的zRange赋值给每个小块的zRange
			b.zRange = zRange;
		}
	}
}


/**
 * @brief 通过光线投影得到3D world model在该当前视角下的图像或者深度图，对于图像中的每个像素，光线从相机投射到与表面的交点，但是由于大部分空间是不存储信息的，因此需要采取一种由粗到细的搜索方式
 * @param xy 对应的像素坐标
 * @param oneOverVoxelSize 将以m为计量单位转成以voxel为单位  
 * @param viewFrustum_minmax 为当前视角相机可见的最小深度和最大深度
 * @param invProjParams 为从成像平面到3D空间的反投影参数
 * @param invM 将当前坐标系的空间点变换到世界坐标系下，Twc
 * @param mu 为max truncation band of the signed distance transform
 */
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool castRay(DEVICEPTR(Vector4f) &pt_out, int x, int y, const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(typename TIndex::IndexData) *voxelIndex, Matrix4f invM, Vector4f projParams, float oneOverVoxelSize, 
	float mu, const CONSTPTR(Vector2f) & viewFrustum_minmax)
{
	Vector4f pt_camera_f; Vector3f pt_block_s, pt_block_e, rayDirection, pt_result;
	bool pt_found, hash_found;
	float sdfValue = 1.0f;
	float totalLength, stepLength, totalLengthMax, stepScale;

	stepScale = mu * oneOverVoxelSize;

	pt_camera_f.z = viewFrustum_minmax.x;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
	pt_camera_f.w = 1.0f;
	totalLength = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
	pt_block_s = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;

	pt_camera_f.z = viewFrustum_minmax.y;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
	pt_camera_f.w = 1.0f;
	totalLengthMax = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
	pt_block_e = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;

	//光线投影的方向
	rayDirection = pt_block_e - pt_block_s;
	float direction_norm = 1.0f / sqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y + rayDirection.z * rayDirection.z);
	//对搜索方向进行归一化
	rayDirection *= direction_norm;

	//搜索的起点坐标，以voxel为计量单位
	pt_result = pt_block_s;

	typename TIndex::IndexCache cache;

	while (totalLength < totalLengthMax) {
		sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, hash_found, cache);

		if (!hash_found) {
		        //如果当前找到的体素为空，则步长为SDF_BLOCK_SIZE,以SEARCH_BLOCK_COARSE方式搜寻
			stepLength = SDF_BLOCK_SIZE;
		} else {
			if ((sdfValue <= 0.1f) && (sdfValue >= -0.5f)) {
			        //基本上到场景的表面了，进行插值（取邻域的7个voxel进行加权），以SEARCH_BLOCK_SURFACE
				sdfValue = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, hash_found, cache);
			}
			//如果插值完得到的sdf值依旧小于0,则需要往回搜索，毕竟<=0已经进入场景内部了
			if (!(sdfValue > 0.0f)) break;
			stepLength = MAX(sdfValue * stepScale, 1.0f);
		}
                //如果当前找到的体素不为空，则意味着光线投影已经到达了分配的区域，缩小步长,此时的步长与max trancation band of the sdf有关，即以SEARCH_BLOCK_FINE方式搜寻
		pt_result += stepLength * rayDirection; totalLength += stepLength;
	}

	//进入场景内部，往回搜索,此时的状态为WRONG_SIDE
	if (sdfValue <= 0.0f)
	{
	        //由于此时sdValue为负，所以往反方向搜索
		stepLength = sdfValue * stepScale;
		pt_result += stepLength * rayDirection;

		sdfValue = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, pt_found, cache);
		stepLength = sdfValue * stepScale;
		pt_result += stepLength * rayDirection;
	} else pt_found = false;

	pt_out.x = pt_result.x; pt_out.y = pt_result.y; pt_out.z = pt_result.z;
	if (pt_found) pt_out.w = 1.0f; else pt_out.w = 0.0f;

	return pt_found;
}

///@brief 将像素对应的3D空间点投影到当前视角的Depth坐标系下的像平面上
///@param pixel 世界坐标系下空间点的坐标（以m为单位）
///@param M Tcw
///@param projParams 投影参数
_CPU_AND_GPU_CODE_ inline int forwardProjectPixel(Vector4f pixel, const CONSTPTR(Matrix4f) &M, const CONSTPTR(Vector4f) &projParams,
	const THREADPTR(Vector2i) &imgSize)
{
	pixel.w = 1;
	pixel = M * pixel;

	Vector2f pt_image;
	pt_image.x = projParams.x * pixel.x / pixel.z + projParams.z;
	pt_image.y = projParams.y * pixel.y / pixel.z + projParams.w;

	if ((pt_image.x < 0) || (pt_image.x > imgSize.x - 1) || (pt_image.y < 0) || (pt_image.y > imgSize.y - 1)) return -1;

	return (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
}

/// @brief 从SDF中计算出法线，而非通过像素点对应的模型表面空间点的坐标计算出法线
/// @param foundPoint 是否当前需要计算法线的像素有对应的场景表面空间点
/// @param point 当前需要计算法线的像素对应的场景表面空间点的坐标，其计量单位为voxel,基于世界坐标系下
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(Vector3f) & point,
                                                     const CONSTPTR(TVoxel) *voxelBlockData, const CONSTPTR(typename TIndex::IndexData) *indexData,
                                                     const THREADPTR(Vector3f) & lightSource, THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle)
{
	if (!foundPoint) return;

	outNormal = computeSingleNormalFromSDF(voxelBlockData, indexData, point);

	float normScale = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
	outNormal *= normScale;

	angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
	if (!(angle > 0.0)) foundPoint = false;
}

/**
 @brief 计算对应像素的法线和角度
 @param foundPoint 该像素是否有对应的场景表面空间点
 @param pointsRay 该像素对应的场景表面空间点的坐标(世界坐标系下)
 @param outNormal 该像素对应的场景表面空间点的法线
 @param angle 该像素对应的场景表面空间点法线的角度
*/
template <bool useSmoothing>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(int) &x, const THREADPTR(int) &y,
	const CONSTPTR(Vector4f) *pointsRay, const THREADPTR(Vector3f) & lightSource, const THREADPTR(float) &voxelSize,
	const THREADPTR(Vector2i) &imgSize, THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle)
{
	if (!foundPoint) return;

	Vector4f xp1_y, xm1_y, x_yp1, x_ym1;

	//由于计算当前像素对应空间点的法线需要相邻的空间点，以此必须保证相邻空间点存在
	if (useSmoothing)
	{
		if (y <= 2 || y >= imgSize.y - 3 || x <= 2 || x >= imgSize.x - 3) { foundPoint = false; return; }

		xp1_y = pointsRay[(x + 2) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 2) * imgSize.x];
		xm1_y = pointsRay[(x - 2) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 2) * imgSize.x];
	}
	else
	{
		if (y <= 1 || y >= imgSize.y - 2 || x <= 1 || x >= imgSize.x - 2) { foundPoint = false; return; }

		xp1_y = pointsRay[(x + 1) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
		xm1_y = pointsRay[(x - 1) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
	}

	Vector4f diff_x(0.0f, 0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f, 0.0f);

	bool doPlus1 = false;
	//<=0意味着没有找到相应的空间点
	if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) doPlus1 = true;
	else
	{
		diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;
		//得到xy方向差分向量模的最大值
		float length_diff = MAX(diff_x.x * diff_x.x + diff_x.y * diff_x.y + diff_x.z * diff_x.z,
			diff_y.x * diff_y.x + diff_y.y * diff_y.y + diff_y.z * diff_y.z);

		//如果差分向量的模太大，则意味着当前像素对应的空间点与相邻空间点距离太远，计算出来的法线精度较差，需要重新计算
		if (length_diff * voxelSize * voxelSize > (0.15f * 0.15f)) doPlus1 = true;
	}

	//当选择的是useSmoothing模式的时候，会选择隔俩个像素对应的空间点计算法线，若空间点的差分向量模太长，则再计算一次，此时选择的是隔一个像素对应的空间点
	if (doPlus1)
	{
		if (useSmoothing)
		{
			xp1_y = pointsRay[(x + 1) + y * imgSize.x]; x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
			xm1_y = pointsRay[(x - 1) + y * imgSize.x]; x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
			diff_x = xp1_y - xm1_y; diff_y = x_yp1 - x_ym1;
		}

		if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0)
		{
			foundPoint = false;
			return;
		}
	}

	//叉积x,y方向的差分向量得到z方向的向量，即法线
	outNormal.x = -(diff_x.y * diff_y.z - diff_x.z*diff_y.y);
	outNormal.y = -(diff_x.z * diff_y.x - diff_x.x*diff_y.z);
	outNormal.z = -(diff_x.x * diff_y.y - diff_x.y*diff_y.x);

	//归一化法线
	float normScale = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
	outNormal *= normScale;

	angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
	if (!(angle > 0.0)) foundPoint = false;
}

_CPU_AND_GPU_CODE_ inline void drawPixelGrey(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle)
{
	float outRes = (0.8f * angle + 0.2f) * 255.0f;
	dest = Vector4u((uchar)outRes);
}

_CPU_AND_GPU_CODE_ inline void drawPixelNormal(DEVICEPTR(Vector4u) & dest, const THREADPTR(Vector3f) & normal_obj)
{
	dest.r = (uchar)((0.3f + (-normal_obj.r + 1.0f)*0.35f)*255.0f);
	dest.g = (uchar)((0.3f + (-normal_obj.g + 1.0f)*0.35f)*255.0f);
	dest.b = (uchar)((0.3f + (-normal_obj.b + 1.0f)*0.35f)*255.0f);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawPixelColour(DEVICEPTR(Vector4u) & dest, const CONSTPTR(Vector3f) & point, 
	const CONSTPTR(TVoxel) *voxelBlockData, const CONSTPTR(typename TIndex::IndexData) *indexData)
{
	Vector4f clr = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelBlockData, indexData, point);

	dest.x = (uchar)(clr.x * 255.0f);
	dest.y = (uchar)(clr.y * 255.0f);
	dest.z = (uchar)(clr.z * 255.0f);
	dest.w = 255;
}

/// \brief Renders a voxel based on its Z coordinate in the (raycasting) camera's frame.
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawPixelDepth(
		DEVICEPTR(float) & dest,
		const CONSTPTR(Vector3f) & point,
		const CONSTPTR(Matrix4f) &camPose,
		const float voxelSizeMeters
) {
	Vector4f point_h;
	point_h.x = point.x * voxelSizeMeters;
	point_h.y = point.y * voxelSizeMeters;
	point_h.z = point.z * voxelSizeMeters;
	point_h.w = 1.0f;

	Vector4f point_cam = camPose * point_h;
	point_cam /= point_cam.w;

	dest = point_cam.z;
};

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelICP(DEVICEPTR(Vector4u) &outRendering, DEVICEPTR(Vector4f) &pointsMap, DEVICEPTR(Vector4f) &normalsMap,
	const THREADPTR(Vector3f) & point, bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
	float voxelSize, const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint)
	{
		drawPixelGrey(outRendering, angle);

		Vector4f outPoint4;
		outPoint4.x = point.x * voxelSize; outPoint4.y = point.y * voxelSize;
		outPoint4.z = point.z * voxelSize; outPoint4.w = 1.0f;
		pointsMap = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;
		normalsMap = outNormal4;
	}
	else
	{
		Vector4f out4;
		out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

		pointsMap = out4; normalsMap = out4; outRendering = Vector4u((uchar)0);
	}
}

template<bool useSmoothing>
_CPU_AND_GPU_CODE_ inline void processPixelICP(DEVICEPTR(Vector4u) *outRendering, DEVICEPTR(Vector4f) *pointsMap, DEVICEPTR(Vector4f) *normalsMap,
	const CONSTPTR(Vector4f) *pointsRay, const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize,
	const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

        ///  判断当前视角下对应像素是否存在空间点
	bool foundPoint = point.w > 0.0f;

	/// 计算法线及夹角
	computeNormalAndAngle<useSmoothing>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint)
	{
		drawPixelGrey(outRendering[locId], angle);

		/// 将找到的点量纲转换为m
		Vector4f outPoint4;
		outPoint4.x = point.x * voxelSize; 
		outPoint4.y = point.y * voxelSize;
		outPoint4.z = point.z * voxelSize; 
		outPoint4.w = 1.0f;
		pointsMap[locId] = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x;
		outNormal4.y = outNormal.y;
		outNormal4.z = outNormal.z;
		outNormal4.w = 0.0f;
		normalsMap[locId] = outNormal4;
	}
	else
	{
		Vector4f out4;
		out4.x = 0.0f; 
		out4.y = 0.0f; 
		out4.z = 0.0f; 
		out4.w = -1.0f;

		pointsMap[locId] = out4; 
		normalsMap[locId] = out4; 
		outRendering[locId] = Vector4u((uchar)0);
	}
}

template<bool useSmoothing>
_CPU_AND_GPU_CODE_ inline void processPixelForwardRender(DEVICEPTR(Vector4u) *outRendering, const CONSTPTR(Vector4f) *pointsRay, 
	const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize, const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering[locId], angle);
	else outRendering[locId] = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelGrey(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point, 
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex, 
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering, angle);
	else outRendering = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
struct PixelColourcoder
{
	static _CPU_AND_GPU_CODE_ inline void process(DEVICEPTR(Vector4u) &outRendering, const DEVICEPTR(Vector3f) & point,
		bool foundPoint, const DEVICEPTR(TVoxel) *voxelData, const DEVICEPTR(typename TIndex::IndexData) *voxelIndex, Vector3f lightSource)
	{
		processPixelGrey<TVoxel, TIndex>(outRendering, point, foundPoint, voxelData, voxelIndex, lightSource);
	}
};

template<class TVoxel>
struct PixelColourcoder<TVoxel, ITMVoxelBlockHHash> {
	static _CPU_AND_GPU_CODE_ inline void process(DEVICEPTR(Vector4u) &outRendering, const DEVICEPTR(Vector3f) & point, 
	bool foundPoint, const DEVICEPTR(TVoxel) *voxelData, const DEVICEPTR(ITMVoxelBlockHHash::IndexData) *voxelIndex, Vector3f lightSource)
	{
		Vector3f outNormal;
		float angle;

		computeNormalAndAngle<TVoxel, ITMVoxelBlockHHash>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		ITMVoxelBlockHHash::IndexCache cache; bool isFound;
		readFromSDF_float_uninterpolated(voxelData, voxelIndex, point, isFound, cache);

		Vector4u colour;
		if (foundPoint) {
			float outRes = (0.8f * angle + 0.2f) * 255.0f;
			if (cache.blockSize<=1) colour = Vector4u(outRes, 0.0f, 0.0f, 255.0f);
			else if (cache.blockSize<=2) colour = Vector4u(outRes, outRes, 0.0f, 255.0f);
			else if (cache.blockSize<=4) colour = Vector4u(0.0f, outRes, 0.0f, 255.0f);
			else colour = Vector4u(0.0f, 0.0f, outRes, 255.0f);
		}
		else colour = Vector4u((uchar)0);
		outRendering = colour;
	}
};

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelColour(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point,
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex, 
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelColour<TVoxel, TIndex>(outRendering, point, voxelData, voxelIndex);
	else outRendering = Vector4u((uchar)0);
}


/// \brief Colors the pixel using a grayscale value based on its depth from the camera.
/// Used for rendering depth maps from arbitrary viewpoints, which is very handy for evaluating the
/// SLAM system.
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelColourDepth(
	DEVICEPTR(float) &outRendering,
	const CONSTPTR(Vector3f) &point,
	bool foundPoint,
	Matrix4f &camPose,
	float voxelSizeMeters
) {
	if (foundPoint) {
		drawPixelDepth<TVoxel, TIndex>(outRendering, point, camPose, voxelSizeMeters);
	}
	else {
		outRendering = 0.0f;
	}
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelNormal(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point,
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelNormal(outRendering, outNormal);
	else outRendering = Vector4u((uchar)0);
}
