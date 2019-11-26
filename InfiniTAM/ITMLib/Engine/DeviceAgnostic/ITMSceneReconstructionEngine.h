// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "ITMPixelUtils.h"
#include "ITMRepresentationAccess.h"

///@brief 对TSDF模型中的sdf值进行融合，并不是深度值
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
	const CONSTPTR(Vector4f) & projParams_d, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image (depth image? hansry)
	// 将世界坐标系下的空间点转到当前帧depth坐标系下
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	// 从深度图获得对应的空间点坐标(空间点投影到像平面上)
	depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	// depth_measure为当前视角下传感器测量的深度值，pt_camera.z为当前视角光心到对应网格点的距离，从理论来说俩者应该一致
	eta = depth_measure - pt_camera.z;
	
	// 若depth_measure小于pt_camera.z太多，则不更新网格的tsdf值
	// mu即为max truncation
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::SDF_valueToFloat(voxel.sdf); oldW = voxel.w_depth;

	newF = MIN(1.0f, eta / mu);
	//新的sdf值的权重为1.0
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	// write back
	voxel.sdf = TVoxel::SDF_floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_rgb,
	const CONSTPTR(Vector4f) & projParams_rgb, float mu, uchar maxW, float eta, const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC; Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float)voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = MIN(newW, maxW);

	buffV3u = TO_UCHAR3(newC * 255.0f);

	voxel.clr = buffV3u;
	voxel.w_color = (uchar)newW;
}

///@brief 对TSDF模型中的sdf值或者颜色信息进行融合，并不是深度值
template<bool hasColor, class TVoxel> struct ComputeUpdatedVoxelInfo;

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
		const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
		const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const THREADPTR(Vector2i) & imgSize_rgb)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};

/// @brief 检查深度图x,y对应的voxel block是否已经被分配(通过检查hashEntry)，若已经被分配，是否需要将其可见性进行更新
/// @param entriesAllocType_device 存储着 对应着entriAllocType_device的某个元素（对应索引为hashIdx） 的空间点是否需要被分配或者swap in, 
///                                1表示在ordered part进行分配, 2表示需要在excess中进行分配
/// @param blockCoords 存储了voxel block在当前坐标系下的三维空间坐标，而不是每个空间点的三维坐标
_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, int x, int y,
	DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable, float viewFrustum_min, float viewFrustum_max)
{
	float depth_measure; unsigned int hashIdx; int noSteps;
	Vector3f pt_camera_f, point_e, point, direction; Vector3s blockPos;

	//相机坐标系下将深度图转成空间点，通过width of the band进行约束
	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	//根据当前深度图的空间点转换到世界坐标系下，计算待分配空间(volume)的范围
	Vector4f tmp;
	tmp.x = pt_camera_f.x * (1.0f - mu / norm);
	tmp.y = pt_camera_f.y * (1.0f - mu / norm);
	tmp.z = pt_camera_f.z * (1.0f - mu / norm);
	tmp.w = 1.0f;
	
	//oneOverVoxelSize = 1.0/(voxel_size*SDF_BLOCK_SIZE)
	//将m为单位换成了以voxel block为计量单位
	point = TO_VECTOR3(invM_d * tmp) * oneOverVoxelSize;
	tmp.x = pt_camera_f.x * (1.0f + mu / norm);
	tmp.y = pt_camera_f.y * (1.0f + mu / norm);
	tmp.z = pt_camera_f.z * (1.0f + mu / norm);
	point_e = TO_VECTOR3(invM_d * tmp) * oneOverVoxelSize;

	direction = point_e - point;
	norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	//ceil返回不小于value的最小整数
	noSteps = (int)ceil(2.0f*norm);

	//归一化，只取方向
	direction /= (float)(noSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
	        //注意blockPos为以voxel blocks大小为单位的坐标，也就是说是每个voxel blocks的坐标而不是每个空间点的坐标
		blockPos = TO_SHORT_FLOOR3(point);

		//compute index in hash table
		//template<typename T> _CPU_AND_GPU_CODE_ inline int hashIndex(const THREADPTR(T) & blockPos) {
	        //    return (((uint)blockPos.x * 73856093u) ^ ((uint)blockPos.y * 19349669u) ^ ((uint)blockPos.z * 83492791u)) & (uint)SDF_HASH_MASK;
                //}    
		hashIdx = hashIndex(blockPos);

		//check if hash table contains entry
		bool isFound = false;

		ITMHashEntry hashEntry = hashTable[hashIdx];

		//由voxel block坐标通过hash function得到hash entry，判断hash entry存储的voxel block坐标(注意这里是voxel block的空间坐标，
		//而不是空间点的坐标,是否与输入voxel block坐标一样， 若hash entry存储的voxel block坐标与输入的voxel block坐标一样，
		//则已经被分配或者被swapped out （hashEntry.ptr>=-1）
		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
		        //将该hash entry添加到可见列表中，若hashEntry.ptr==-1，则说明voxel block已经被swapped out,若hashEntry>-1,则说明该voxel block原本就是可见的
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

			isFound = true;
		}
                //计算得到的hash entry存储的voxel block与输入的voxel block坐标不同
		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
				// TODO: This causes a bug in case blocks ever get deleted by setting ptr to -1
				// as there might be an excess list further back
				// also, in the actual allocation the offset might get reset to 0 in such cases
			{
				while (hashEntry.offset >= 1)
				{
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];

					if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
					        // 2: in host memory (be swapped out) but visible
					        // 1: in device memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}
                        //如果计算的hashEntry在order part和unorder part都没有找到对应的voxel block，则需要新分配
                        //分配有俩种情况：
                        //1.当得到的hash entry在ordered part存储的voxel hash与输入的voxel block坐标不同，且在ordered part未被分配，即hashEntry.ptr<-1, 则需要在ordered part分配
                        //2.当得到的hash entry在ordered part存储的voxel hash与输入的voxel block坐标不同，但在ordered part已被分配，即hashEntry.ptr>=-1,则需要在excess list分配
			if (!isFound) //still not found
			{
			        //其中2表示在excess中进行分配，1表示在ordered part进行分配
				entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation 
				if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible

				blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1);
			}
		}

		point += direction;
	}
}

_CPU_AND_GPU_CODE_ inline void buildHHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *globalEntriesAllocType, DEVICEPTR(uchar) *globalEntriesVisibleType, int x, int y,
	DEVICEPTR(Vector4s) *globalBlockCoords, const DEVICEPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverSmallestBlockSize, DEVICEPTR(ITMHHashEntry) *globalHashTable, float viewFrustum_min, float viewFrustum_max)
{
	float depth_measure; unsigned int hashIdx; int noSteps;
	Vector3f pt_camera_f, point, point_e, direction; Vector3s blockPos;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

	//find block coords for start ray
	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrtf(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	point   = (invM_d * (pt_camera_f * (1.0f - mu/norm))) * oneOverSmallestBlockSize;
	point_e = (invM_d * (pt_camera_f * (1.0f + mu/norm))) * oneOverSmallestBlockSize;

	direction = point_e - point;
	norm = sqrtf(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	noSteps = (int)ceilf(2.0f * norm);
	direction /= (noSteps-1);

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
		for (int level = SDF_HASH_NO_H_LEVELS-1; level >= 0; level--) {
			int hierBlockSize = (1 << level);
			ITMHHashEntry *hashTable = globalHashTable + level * ITMVoxelBlockHHash::noTotalEntriesPerLevel;
			uchar *entriesAllocType = globalEntriesAllocType + level * ITMVoxelBlockHHash::noTotalEntriesPerLevel;
			uchar *entriesVisibleType = globalEntriesVisibleType + level * ITMVoxelBlockHHash::noTotalEntriesPerLevel;
			Vector4s *blockCoords = globalBlockCoords + level * ITMVoxelBlockHHash::noTotalEntriesPerLevel;

			blockPos = TO_SHORT_FLOOR3(point/(float)hierBlockSize);

			//compute index in hash table
			hashIdx = hashIndex(blockPos);

			//check if hash table contains entry
			bool shouldContinueDown = false;
			bool foundValue = false;
			char allocType = 1;
			int reactivateIdx = -1;
			do
			{
				const ITMHHashEntry &hashEntry = hashTable[hashIdx];

				if (hashEntry.pos == blockPos) {
					if (hashEntry.ptr == -2) {
						// block found, but it's split
						shouldContinueDown = true;
					} else if (hashEntry.ptr == -1) {
						// block found, but swapped out
						entriesVisibleType[hashIdx] = 2;
					} else if (hashEntry.ptr >= 0) {
						// block found
						entriesVisibleType[hashIdx] = 1;
					}
					foundValue = true;
					break;
				}
				if (hashEntry.ptr < -2) reactivateIdx = hashIdx;

				allocType = 2;
				int offsetExcess = hashEntry.offset - 1;
				if (offsetExcess < 0) break;

				hashIdx = SDF_BUCKET_NUM + offsetExcess;
			} while (true);

			if (shouldContinueDown) continue;
			if (foundValue) break;

			if (reactivateIdx != -1) {
				hashIdx = reactivateIdx;
				allocType = 3;
			}
			entriesAllocType[hashIdx] = allocType;
			Vector4s tempVector(blockPos.x, blockPos.y, blockPos.z, 1);
			blockCoords[hashIdx] = tempVector;
			//per-image hash collisions are ignored (will be picked up next frame)
			break;
		}

		point += direction;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkPointVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector4f) &pt_image, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_buff;

	pt_buff = M_d * pt_image;

	if (pt_buff.z < 1e-10f) return;

	pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
	pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

	if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) { 
	  isVisible = true; 
	  isVisibleEnlarged = true; 
	}
	else if (useSwapping)
	{
		Vector4i lims;
		lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
		lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

		if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w) isVisibleEnlarged = true;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector3s) &hashPos, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(float) &voxelSize, const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_image;
	float factor = (float)SDF_BLOCK_SIZE * voxelSize;

	isVisible = false; isVisibleEnlarged = false;

	// 0 0 0
	pt_image.x = (float)hashPos.x * factor; pt_image.y = (float)hashPos.y * factor;
	pt_image.z = (float)hashPos.z * factor; pt_image.w = 1.0f;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 0 1
	pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 1
	pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 1
	pt_image.x += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 0 
	pt_image.z -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 0 
	pt_image.y -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 0
	pt_image.x -= factor; pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 1
	pt_image.x += factor; pt_image.y -= factor; pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;
}

