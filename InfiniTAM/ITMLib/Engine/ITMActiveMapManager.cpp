// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMActiveMapManager.h"

using namespace ITMLib::Engine;

// try loop closures for this number of frames
static const int N_linktrials = 20;
// at least these many frames have to be tracked successfully
static const int N_linkoverlap = 10;
// try relocalisations for this number of frames
static const int N_reloctrials = 20;
// at least these many tracking attempts have to succeed for relocalisation
static const int N_relocsuccess = 10;
// When checking "overlap with original local map", find how many of the first N
// blocks are still visible
static const int N_originalblocks = 1000;
static const float F_originalBlocksThreshold = 0.2f; //0.4f

ITMActiveMapManager::ITMActiveMapManager(ITMMapGraphManager *_localMapManager)
{
	localMapManager = _localMapManager;
}

/// @brief 创建新的local map
int ITMActiveMapManager::initiateNewLocalMap(bool isPrimaryLocalMap)
{
        /// 通过localMapManager建立新的local map，并返回其存储在addData中的index
	int newIdx = localMapManager->createNewLocalMap();

	ActiveDataDescriptor newLink;
	newLink.localMapIndex = newIdx;
	/// 存储新地图的类型
	newLink.type = isPrimaryLocalMap ? PRIMARY_LOCAL_MAP : NEW_LOCAL_MAP;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);

	return newIdx;
}

/// @brief 在重定位或者发生回环的时候，处于inactive的localmap可能会变为active，这时候需要将该inactive localmap的Id存储到activeData中
/// @param localMapId 发生回环或者重定位时候的inactive localmap的id (在存储了所有localmap的vector的index)
/// @param isRelocalisation 如果跟踪失败且不插入关键帧(意味着当前帧与关键帧库中有相似的帧)，则为重定位，若跟踪成功且不插入关键帧，则为回环
/// @return 返回该inactive localmap->active localmap之后存储在activeData中的index，一般存储在activeData列表的末位
int ITMActiveMapManager::initiateNewLink(int localMapId, const ITMPose & pose, bool isRelocalisation)
{
	static const bool ensureUniqueLinks = true;

	// make sure only one relocalisation per local map is attempted at a time
	if (ensureUniqueLinks) {
		for (size_t i = 0; i < activeData.size(); ++i) {
			if (activeData[i].localMapIndex == localMapId) {
			  return -1;
			}
		}
	}

	if (!localMapManager->resetTracking(localMapId, pose)) return -1;

	ActiveDataDescriptor newLink;
	newLink.localMapIndex = localMapId;
	newLink.type = isRelocalisation ? RELOCALISATION : LOOP_CLOSURE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
	
	return (int)activeData.size() - 1;
}

//返回id为dataID的可见的voxel block个数占整个已分配的内存地图大小的比例
float ITMActiveMapManager::visibleOriginalBlocks(int dataID) const
{
	int localMapId = activeData[dataID].localMapIndex;

	int allocated = localMapManager->getLocalMapSize(localMapId);
	int counted = localMapManager->countVisibleBlocks(localMapId, 0, N_originalblocks, true);
	
	int tmp = N_originalblocks;
	if (allocated < tmp) tmp = allocated;
	return (float)counted / (float)tmp;
}

/// @brief 根据当前主子地图的voxel block在当前视角下的可见率来决定是否重新开启新的子地图
bool ITMActiveMapManager::shouldStartNewArea(void) const
{
	int primaryLocalMapIdx = -1;
	int primaryDataIdx = -1;

	// don't start two new local maps at a time
	//std::cout << "activeData: "<<activeData.size()<<std::endl;
	for (int i = 0; i < (int)activeData.size(); ++i)
	{
		if (activeData[i].type == NEW_LOCAL_MAP) {
		  return false;
		}
		if (activeData[i].type == PRIMARY_LOCAL_MAP) 
		{
			primaryDataIdx = i;
			primaryLocalMapIdx = activeData[i].localMapIndex;
		}
	}

	// TODO: check: if relocalisation fails for some time, start new local map
	if (primaryLocalMapIdx < 0) return false;
	else return visibleOriginalBlocks(primaryDataIdx) < F_originalBlocksThreshold;

	return false;
}

/// @brief 判断newDataId和primaryDataId的voxel block可见率来确定primary local map，当newDataId的voxel block可见率高于primaryDataId的voxel block可见率后，
///        还需要判断与bestDataId子地图的(其中bestDataId为目前为止遍历的子地图中，voxel block可见率最高的子地图)voxel block可见率
bool ITMActiveMapManager::shouldMovePrimaryLocalMap(int newDataId, int bestDataId, int primaryDataId) const
{
	int localMapIdx_primary = -1;
	int localMapIdx_best = -1;
	int localMapIdx_new = -1;

	int blocksInUse_primary = -1;
	float visibleRatio_primary = 1.0f;
	int blocksInUse_best = -1;
	float visibleRatio_best = 1.0f;
	bool isNewLocalMap_best = false;
	int blocksInUse_new = -1;
	float visibleRatio_new = 1.0f;
	bool isNewLocalMap_new = false;

	if (primaryDataId >= 0) localMapIdx_primary = activeData[primaryDataId].localMapIndex;
	if (bestDataId >= 0) localMapIdx_best = activeData[bestDataId].localMapIndex;
	if (newDataId >= 0) localMapIdx_new = activeData[newDataId].localMapIndex;

	// count blocks in all relevant localMaps
	if (localMapIdx_primary >= 0) 
	{
		blocksInUse_primary = localMapManager->getLocalMapSize(localMapIdx_primary);
		visibleRatio_primary = visibleOriginalBlocks(primaryDataId);
	}

	if (localMapIdx_new >= 0) 
	{
		isNewLocalMap_new = (activeData[newDataId].type == NEW_LOCAL_MAP);
		blocksInUse_new = localMapManager->getLocalMapSize(localMapIdx_new);
		if (blocksInUse_new < 0) return false;
		visibleRatio_new = visibleOriginalBlocks(newDataId);
	}

	if (localMapIdx_best >= 0) 
	{
		isNewLocalMap_best = (activeData[bestDataId].type == NEW_LOCAL_MAP);
		blocksInUse_best = localMapManager->getLocalMapSize(localMapIdx_best);
		visibleRatio_best = visibleOriginalBlocks(bestDataId);
	}

	if (blocksInUse_primary < 0) {
		// TODO: if relocalisation fails, a new local map gets started,
		//       and is eventually accepted, this case will get relevant
		return true;
	}

	// step 1: is "new" better than "primary" ?
        // 步骤1: 判断“new”子地图是否好于"primary"子地图,通过visibleRatio来进行判断
	// don't continue a local map that is already full
/*	if (blocksInUse_new >= N_maxblocknum) return false;

	if (blocksInUse_new >= blocksInUse_primary) return false;*/
	if (visibleRatio_new <= visibleRatio_primary) return false;

	// step 2: is there any contender for a new local map to move to?
	if (blocksInUse_best < 0) return true;

	// if this is a new local map, but we previously found that we can loop
	// close, don't accept the new local map!
	if (isNewLocalMap_new && !isNewLocalMap_best) return false;
	// if this is a loop closure and we have not found any alternative
	// loop closure before, accept the new one!
	if (!isNewLocalMap_new && isNewLocalMap_best) return true;

	// if the two are equal, take the smaller one
	//return (blocksInUse_new < blocksInUse_best);
	return (visibleRatio_new > visibleRatio_best);
}

/// @brief 返回Primary map在activeData的位置
int ITMActiveMapManager::findPrimaryDataIdx(void) const
{
	for (int i = 0; i < (int)activeData.size(); ++i) 
	      if (activeData[i].type == PRIMARY_LOCAL_MAP) {
		return i;
	      }
	return -1;
}

/// @brief 返回Primary map在存储子地图容器的位置
int ITMActiveMapManager::findPrimaryLocalMapIdx(void) const
{
	int id = findPrimaryDataIdx();
	if (id < 0) return -1;
	return activeData[id].localMapIndex;
}

int ITMActiveMapManager::findBestVisualisationDataIdx(void) const
{
	int bestIdx = -1;
	for (int i = 0; i < static_cast<int>(activeData.size()); ++i) 
	{
		if (activeData[i].type == PRIMARY_LOCAL_MAP) return i;
		else if (activeData[i].type == NEW_LOCAL_MAP) bestIdx = i;
		else if (activeData[i].type == RELOCALISATION) 
		{
			if (bestIdx < 0) { bestIdx = i; continue; }
			if (activeData[bestIdx].type == NEW_LOCAL_MAP) continue;
			if (activeData[bestIdx].constraints.size() < activeData[i].constraints.size()) bestIdx = i;
		}
	}
	return bestIdx;
}

int ITMActiveMapManager::findBestVisualisationLocalMapIdx(void) const
{
	int id = findBestVisualisationDataIdx();
	if (id < 0) return -1;
	return activeData[id].localMapIndex;
}

/// @brief 存储的俩个子地图之间的变换矩阵，Told_to_new指的是点云的变换那种
void ITMActiveMapManager::recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess)
{
	ActiveDataDescriptor & data = activeData[dataID];

	int primaryLocalMapID = findPrimaryLocalMapIdx();
	/// localMapIndex为当前local map在所有子地图下的Index
	int localMapId = data.localMapIndex;
	data.trackingAttempts++;
        
	if (trackingResult == ITMTrackingState::TRACKING_GOOD)
	{
		if (data.type == RELOCALISATION) {
		  data.constraints.push_back(localMapManager->getTrackingPose(dataID)->GetM());
		}
		else if (((data.type == NEW_LOCAL_MAP) || (data.type == LOOP_CLOSURE)) && primaryTrackingSuccess)
		{
			//在t时刻相机在index为localMapId子地图的位姿，T_｛t,c-new},取逆之后变为T_{t,new-c}
			Matrix4f Tnew_inv = localMapManager->getTrackingPose(localMapId)->GetInvM();
			
			//在t时刻相机在primaryLocalMap的位姿，T_{t,c-old}
			Matrix4f Told = localMapManager->getTrackingPose(primaryLocalMapID)->GetM();
			//在坐标系情况下，为
			//Tnew_to_old = T_{t,new_c}*T_{t,c_old}
			//但是如果是看做是点云的变换的话，即为Told_to_new了
			Matrix4f Told_to_new = Tnew_inv * Told;

			//指的是点云的变换
			data.constraints.push_back(Told_to_new);
		}
	}
	else if (trackingResult == ITMTrackingState::TRACKING_FAILED)
	{
		if (data.type == PRIMARY_LOCAL_MAP)
		{
			for (size_t j = 0; j < activeData.size(); ++j)
			{
				if (activeData[j].type == NEW_LOCAL_MAP) activeData[j].type = LOST_NEW;
				else activeData[j].type = LOST;
			}
		}
	}
}

/// @brief  wt = sqrt(2br'-b^{2})/r', r' = max( ||v(T_{t,i,j})-v(T'_{i,j})|| , b) = max(||residual||, b)
/// @return wt
static float huber_weight(float residual, float b)
{
	double r_abs = fabs(residual);
	if (r_abs < b) {
	  return 1.0f;
	}
	double tmp = 2.0 * b * r_abs - b*b;
	return (float)(sqrt(tmp) / r_abs);
}

/** estimate a relative pose, taking into account a previous estimate (weight 0
*	indicates that no previous estimate is available).
*	out_numInliers and out_inlierPose is the number of inliers from amongst the
*	new observations and the pose computed from them.
*　　估计相对位姿，考虑之前的估计（权重为0表示没有之前的估计）。
*   @brief 对于子图i,j而言，在某个时刻可以由当前帧计算出俩个子图的变换关系，因此多个时刻下拥有n个变换关系constrains，根据权重对每个observation和PreviousEstimate进行加权得到新的Pose
*   @param observations 对于i,j子地图而言，由于可以经过多次观测得到i,j的相对变换T—｛t,i,j｝，每次观测都可以建立一个constraints
*   @param previousEstimate previous estimate T_{i,j}
*   @param out_numInliers 通过最后最终每个观测的权重来计算观测内点的数量
*   @param out_inlierPose 通过判断为内点的观测和previousEstimate得到的位姿
*/
static ITMPose estimateRelativePose(const std::vector<Matrix4f> & observations, const ITMPose & previousEstimate, float previousEstimate_weight, int *out_numInliers, ITMPose *out_inlierPose)
{
	static const float huber_b = 0.1f;
	static const float weightsConverged = 0.01f;
	static const int maxIter = 10;
	static const float inlierThresholdForFinalResult = 0.8f;
	//存储了每个observation变换关系Told_to_new的权重，最后一维存储了previous estimate的权重
	std::vector<float> weights(observations.size() + 1, 1.0f);
	std::vector<ITMPose> poses;

	//将多次观察到的位姿存储在poses容器中
	for (size_t i = 0; i < observations.size(); ++i) {
	   poses.push_back(ITMPose(observations[i]));
	}
	//李代数，6维向量
	float params[6];
	for (int iter = 0; iter < maxIter; ++iter) 
	{
		// estimate with fixed weights
	        // 步骤1:对previousEstimate和observations的相对变换进行加权（根据当前weights中的权重进行加权，初始化的时候权重均为1）,要转换成李代数才能进行加法运算　
		float sumweight = previousEstimate_weight;
		for (int j = 0; j < 6; ++j) {
		  params[j] = weights.back() * previousEstimate_weight * previousEstimate.GetParams()[j];
		}
		for (size_t i = 0; i < poses.size(); ++i) 
		{
			for (int j = 0; j < 6; ++j) {
			  params[j] += weights[i] * poses[i].GetParams()[j];
			}
			sumweight += weights[i];
		}
		for (int j = 0; j < 6; ++j) {
		  params[j] /= sumweight;
		}
		
		// 步骤2:对每个observation和previousEstimate对应的相对位姿计算新的权重
		float weightchanges = 0.0f;
		for (size_t i = 0; i < weights.size(); ++i) 
		{
			const ITMPose *p;
			float w = 1.0f;
			//observations得到的相对变换
			if (i < poses.size()) {
			  p = &(poses[i]);
			}
			//previousEstimate的相对变换
			else
			{
				p = &(previousEstimate);
				w = previousEstimate_weight;
			}

			//resiual = sqrt(||V(T_{t,i,j})-V(T'_{i,j})||)
			float residual = 0.0f;
			for (int j = 0; j < 6; ++j) 
			{
				float r = p->GetParams()[j] - params[j];
				residual += r*r;
			}
			residual = sqrt(residual);
			
			//newWeight = wt = sqrt(2*b*r'-b^{2})/r', r'= max(||residual||, b)
			//使用了huber函数
			float newweight = huber_weight(residual, huber_b);
			//对每个观测和previousEstimation的权重改变量进行求和
			weightchanges += w * fabs(newweight - weights[i]);
			weights[i] = newweight;
		}

		//每次计算新权重平均改变的量
		float avgweightchange = weightchanges / (weights.size() - 1 + previousEstimate_weight);
		//若当前的权重平均该变量太小，则跳出迭代
		if (avgweightchange < weightsConverged) break;
	}

	int inliers = 0;
	Matrix4f inlierTrafo;
	inlierTrafo.setZeros();
	//步骤3:根据最后计算的权重对observations中的所有约束进行内点和外点的计算，统计inliers的数量
	for (size_t i = 0; i < poses.size(); ++i) if (weights[i] > inlierThresholdForFinalResult) 
	{
	        //很奇怪，为什么可以直接对变换矩阵的元素进行相加？？？？对加法并不封闭啊是不是应该改一下呢？？？？
		inlierTrafo += observations[i];
		++inliers;
	}
	if (out_inlierPose) {
	  out_inlierPose->SetM(inlierTrafo / (float)MAX(inliers, 1));
	}
	if (out_numInliers) {
	  *out_numInliers = inliers;
	}
	return ITMPose(params);
}

/// @brief 判断重定位是否成功
int ITMActiveMapManager::CheckSuccess_relocalisation(int dataID) const
{
	// sucessfully relocalised
	if (activeData[dataID].constraints.size() >= N_relocsuccess) return 1;

	// relocalisation failed: declare as LOST
	if ((N_reloctrials - activeData[dataID].trackingAttempts) < (N_relocsuccess - (int)activeData[dataID].constraints.size())) return -1;

	// keep trying
	return 0;
}

/// @brief 检查当前dataID的子地图和primaryDataID的主子地图在当前视角跟踪得到的constrains（或者从上一次checkSuccess_newlinke到目前保存的所有constrains）与之前跟踪累计的相对变换关系加权后.
///        通过内点和外点来判断是否接受将加权后的变换关系成为新的relation
/// @return 是否建立联系
int ITMActiveMapManager::CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ITMPose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	// take previous data from local map relations into account!
	//ORUtils::SE3Pose previousEstimate;
	//int previousEstimate_weight = 0;
	int primaryLocalMapIndex = -1;
	if (primaryDataID >= 0) {
	  primaryLocalMapIndex = activeData[primaryDataID].localMapIndex;
	}
	
	//primaryLocalMap和当前localmap关系
	const ITMPoseConstraint & previousInformation = localMapManager->getRelation_const(primaryLocalMapIndex, link.localMapIndex);
	/* hmm... do we want the "Estimate" (i.e. the pose corrected by pose
	   graph optimization) or the "Observations" (i.e. the accumulated
	   poses seen in previous frames?
	   This should only really make a difference, if there is a large
	   disagreement between the two, in which case one might argue that
	   most likely something went wrong with a loop-closure, and we are
	   not really sure the "Estimate" is true or just based on an erroneous
	   loop closure. We therefore want to be consistent with previous
	   observations not estimations...
	*/
	
	/*
	 * 我们需要“估计”(即通过位姿图优化来校正的位姿)还是“观察”(即在前几帧中的累计位姿)?如果两者之间存在很大分歧的情况下，我们可能会认为很可能是某个回环检测出了问题，
	 * 而此时我们并不确定“估计”是正确的，或者不仅仅是基于一个错误的回环检测。因此，我们希望与以前的观测结果保持一致，而不是进行估算。
        */
	
	// 前几帧的观测累计位姿
	ITMPose previousEstimate = previousInformation.GetAccumulatedObservations();
	// 由多少帧观测累计得到的位姿
	int previousEstimate_weight = previousInformation.GetNumAccumulatedObservations();

	int inliers_local;
	ITMPose inlierPose_local;
	if (inliers == NULL) {
	  inliers = &inliers_local;
	}
	if (inlierPose == NULL) {
	  inlierPose = &inlierPose_local;
	}
	
	//link.constraints为当前子地图坐标系到primary子地图在当前视角下下同时对俩个地图进行跟踪得到的变换关系
	//previousEstimate是通过之前跟踪而保留下来的变换关系
	//previousEstimate_weight为之前跟踪保留下来的变换关系的权重
	estimateRelativePose(link.constraints, previousEstimate, (float)previousEstimate_weight, inliers, inlierPose);

	// accept link
	if (*inliers >= N_linkoverlap) {
	  return 1;
	}
	// reject link
	if ((N_linktrials - link.trackingAttempts) < (N_linkoverlap - *inliers)) {
	  return -1;
	}
	//  keep trying
	return 0;
}

/// @brief 为主子地图和当前子地图俩个子地图添加observation
void ITMActiveMapManager::AcceptNewLink(int fromData, int toData, const ITMPose & pose, int weight)
{
	int fromLocalMapIdx = activeData[fromData].localMapIndex;
	int toLocalMapIdx = activeData[toData].localMapIndex;

	{
	        //找到id为fromLocalMapIdx子地图与toLocalMapIdx子地图的相对变换，并添加observation,Tf,t (应该是f的点云到t的点云变换关系)
		ITMPoseConstraint &c = localMapManager->getRelation(fromLocalMapIdx, toLocalMapIdx);
		c.AddObservation(pose, weight);
	}
	{
	        //同样对toLocalMapIdx的子地图也添加fromLocalMapIdx子地图的观测
		ITMPose invPose(pose.GetInvM());
		ITMPoseConstraint &c = localMapManager->getRelation(toLocalMapIdx, fromLocalMapIdx);
		c.AddObservation(invPose, weight);
	}
}

/// @brief  return whether or not the local map graph has changed
/// @brief 如果子地图与主子地图相对变换关系发生了改变，则意味着local map graph也改变了，此时返回true.否则返回false
bool ITMActiveMapManager::maintainActiveData(void)
{
	bool localMapGraphChanged = false;

	int primaryDataIdx = findPrimaryDataIdx();
	int moveToDataIdx = -1;
	for (int i = 0; i < (int)activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if (link.type == RELOCALISATION)
		{
		        ///判断重定位是否成功，如果不成功，则改变当前子地图的跟踪状态
			int success = CheckSuccess_relocalisation(i);
			if (success == 1)
			{
				if (moveToDataIdx < 0) {
				   moveToDataIdx = i;
				}
				else {
				  link.type = LOST;
				}
			}
			else if (success == -1) {
			  link.type = LOST;
			}
		}

		if ((link.type == LOOP_CLOSURE) || (link.type == NEW_LOCAL_MAP))
		{
			ITMPose inlierPose; 
			int inliers;

			int success = CheckSuccess_newlink(i, primaryDataIdx, &inliers, &inlierPose);
			if (success == 1)
			{
			        //将CheckSuccess_newlink函数计算得到的inlierPose作为primaryDataIdx和Id为i的子地图的坐标系变换关系，T-{i,primaryDataIdx}
				AcceptNewLink(primaryDataIdx, i, inlierPose, inliers);
				//清空当前link的所有约束，因为AcceptNewLink已经将这些约束进行考虑了
				link.constraints.clear();
				link.trackingAttempts = 0;
				//是否需要替换主子地图，判断的标准是地图voxel blocks的可见率，可见率越高，则成为主子地图的几率越高
				if (shouldMovePrimaryLocalMap(i, moveToDataIdx, primaryDataIdx)) {
				  moveToDataIdx = i;
				}
				localMapGraphChanged = true;
			}
			else if (success == -1)
			{
				if (link.type == NEW_LOCAL_MAP) {
				  link.type = LOST_NEW;
				}
				else {
				  link.type = LOST;
				}
			}
		}
	}

	std::vector<int> restartLinksToLocalMaps;
	primaryDataIdx = -1;
	for (int i = 0; i < (int)activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		//moveToDataIdx初始化的时候为-1,只有当重定位成功或者某个子地图成为primary local map的时候，才会将moveToDataIdx改为那个id
		//对于RELOCALISATION,若重定位成功，且被checkSuccess，那么该子地图则会变成主子地图
		//对于LOOP_CLOSURE和NEW_LOCAL_MAP，若被checkSuccess,则检查其voxel blocks的可见率是否大于当前的primary local map，若高于，则变为primary local map，否则会对这俩种子地图进行重调整
		if ((signed)i == moveToDataIdx) {
		        link.type = PRIMARY_LOCAL_MAP;
		}
		//这个应该是上一个PRIMARY_LOCAL_MAP
		if ((link.type == PRIMARY_LOCAL_MAP) && (moveToDataIdx >= 0) && ((signed)i != moveToDataIdx)) 
		{
			link.type = LOST;
			restartLinksToLocalMaps.push_back(link.localMapIndex);
		}
		if ((link.type == NEW_LOCAL_MAP) && (moveToDataIdx >= 0)) {
		        link.type = LOST_NEW;
		}
		if ((link.type == LOOP_CLOSURE) && (moveToDataIdx >= 0)) {
			link.type = LOST;
			restartLinksToLocalMaps.push_back(link.localMapIndex);
		}
		if ((link.type == RELOCALISATION) && (moveToDataIdx >= 0)) {
		        link.type = LOST;
		}
		if (link.type == PRIMARY_LOCAL_MAP)
		{
			if (primaryDataIdx >= 0) {
			   fprintf(stderr, "OOOPS, two or more primary localMaps...\n");
			}
			primaryDataIdx = i;
		}
	}

	for (size_t i = 0; i < activeData.size(); )
	{
		ActiveDataDescriptor & link = activeData[i];
		//对于最新的跟踪丢失的子地图，移除它
		if (link.type == LOST_NEW)
		{
			// NOTE: there will only be at most one new local map at
			// any given time and it's guaranteed to be the last
			// in the list. Removing this new local map will therefore
			// not require rearranging indices!
		  
                        // NOTE: 在任何时候最多只能有一个新的本地地图，它保证是列表中的最后一个。因此，删除这个本地地图将不需要重新调整索引
			localMapManager->removeLocalMap(link.localMapIndex);
			link.type = LOST;
		}
		if (link.type == LOST) {
		  activeData.erase(activeData.begin() + i);
		}
		else {
		  i++;
		}
	}

	for (std::vector<int>::const_iterator it = restartLinksToLocalMaps.begin(); it != restartLinksToLocalMaps.end(); ++it) {
		initiateNewLink(*it, *(localMapManager->getTrackingPose(*it)), false);
	}

	// NOTE: 当移除了new local map之后，需要初始化新的地图
	//std::cout << shouldStartNewArea()<<std::endl;
	if (shouldStartNewArea())
	{
		int newIdx = initiateNewLocalMap();

		if (primaryDataIdx >= 0)
		{
			int primaryLocalMapIdx = activeData[primaryDataIdx].localMapIndex;
			//localMapManager->getTrackingPose(primaryLocalMapIdx)->GetM()应该是当前帧相机坐标系到主子地图的相对变换，Tc->p
			//localMapManager->getEstimatedGlobalPose(primaryLocalMapIdx).GetM()应该是当前主子地图到世界坐标系的相对变换，Tp->w
			//因此ORUtils::SE3Pose（）为当前帧到世界坐标系下的变换，即Tc->w = Tc->p*Tp->w
			localMapManager->setEstimatedGlobalPose(newIdx, ITMPose(localMapManager->getTrackingPose(primaryLocalMapIdx)->GetM() * localMapManager->getEstimatedGlobalPose(primaryLocalMapIdx).GetM()));
		}
	}

	return localMapGraphChanged;
}
