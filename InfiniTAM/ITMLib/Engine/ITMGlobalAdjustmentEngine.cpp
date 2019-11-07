// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMGlobalAdjustmentEngine.h"

#include "../../MiniSlamGraphLib/GraphNodeSE3.h"
#include "../../MiniSlamGraphLib/GraphEdgeSE3.h"
#include "../../MiniSlamGraphLib/SlamGraphErrorFunction.h"
#include "../../MiniSlamGraphLib/LevenbergMarquardtMethod.h"

#ifndef NO_CPP11
#include <mutex>
#include <thread>
#include <condition_variable>
#endif

using namespace ITMLib::Engine;

///@note struct的默认成员函数及数据成员是公有的
struct ITMGlobalAdjustmentEngine::PrivateData 
{
#ifndef NO_CPP11
	PrivateData(void) { 
	  stopThread = false; wakeupSent = false; 
	}
	std::mutex workingData_mutex;
	std::mutex processedData_mutex;
	std::thread processingThread;
	bool stopThread;

	std::mutex wakeupMutex;
	std::condition_variable wakeupCond;
	bool wakeupSent;
#endif
};

ITMGlobalAdjustmentEngine::ITMGlobalAdjustmentEngine(void)
{
	privateData = new PrivateData();
	workingData = NULL;
	processedData = NULL;
}

ITMGlobalAdjustmentEngine::~ITMGlobalAdjustmentEngine(void)
{
	stopSeparateThread();
	if (workingData != NULL) delete workingData;
	if (processedData != NULL) delete processedData;
	delete privateData;
}

bool ITMGlobalAdjustmentEngine::hasNewEstimates(void) const
{
	return (processedData != NULL);
}

/// @brief 将在pose graph更新后每个子地图的位姿refine每个子地图基于世界坐标系的位姿
bool ITMGlobalAdjustmentEngine::retrieveNewEstimates(ITMMapGraphManager & dest)
{
#ifndef NO_CPP11
	if (processedData == NULL) return false;

	privateData->processedData_mutex.lock();
	PoseGraphToMultiScene(*processedData, dest);
	delete processedData;
	processedData = NULL;
	privateData->processedData_mutex.unlock();
#endif
	return true;
}

bool ITMGlobalAdjustmentEngine::isBusyEstimating(void) const
{
#ifndef NO_CPP11
	// if someone else is currently using the mutex (most likely the
	// consumer thread), we consider the global adjustment engine to
	// be busy
	if (!privateData->workingData_mutex.try_lock()) return true;

	privateData->workingData_mutex.unlock();
#endif
	return false;
}

/// @brief 根据当前地图观测量并通过multiScenToPoseGraph建立pose graph优化
bool ITMGlobalAdjustmentEngine::updateMeasurements(const ITMMapGraphManager & src)
{
#ifndef NO_CPP11
	// busy, can't accept new measurements at the moment
	if (!privateData->workingData_mutex.try_lock()) {
	  return false;
	}
	if (workingData == NULL) {
	  workingData = new MiniSlamGraph::PoseGraph;
	}
	MultiSceneToPoseGraph(src, *workingData);
	privateData->workingData_mutex.unlock();
#endif
	return true;
}

/// default: blockingWait = true 
bool ITMGlobalAdjustmentEngine::runGlobalAdjustment(bool blockingWait)
{
#ifndef NO_CPP11
	// first make sure there is new data and we have exclusive access to it
	if (workingData == NULL) return false;

	if (blockingWait) {
	  privateData->workingData_mutex.lock();
	}
        else if (!privateData->workingData_mutex.try_lock()) {
	  return false;
	}
	// now run the actual global adjustment
	workingData->prepareEvaluations();
	MiniSlamGraph::SlamGraphErrorFunction errf(*workingData);
	MiniSlamGraph::SlamGraphErrorFunction::Parameters para(*workingData);
	MiniSlamGraph::LevenbergMarquardtMethod::minimize(errf, para);
	workingData->setNodeIndex(para.getNodes());

	// copy data to output buffer
	privateData->processedData_mutex.lock();
	if (processedData != NULL) delete processedData;
	processedData = workingData;
	workingData = NULL;
	privateData->processedData_mutex.unlock();

	privateData->workingData_mutex.unlock();
#endif
	return true;
}

/// @brief 创建新的线程并分离到后台
bool ITMGlobalAdjustmentEngine::startSeparateThread(void)
{
#ifndef NO_CPP11
        //若已经创建了globalAdjustMentEngine线程，且已经joinable，且返回false
	if (privateData->processingThread.joinable()) return false;

	//创建新的线程，调用当前对象的estimationThreadMain函数成员
	privateData->processingThread = std::thread(&ITMGlobalAdjustmentEngine::estimationThreadMain, this);
#endif
	return true;
}

bool ITMGlobalAdjustmentEngine::stopSeparateThread(void)
{
#ifndef NO_CPP11
	if (!privateData->processingThread.joinable()) return false;

	privateData->stopThread = true;
	wakeupSeparateThread();
	privateData->processingThread.join();
#endif
	return true;
}

/// @brief 线程的主要入口
void ITMGlobalAdjustmentEngine::estimationThreadMain(void)
{
#ifndef NO_CPP11
	while (!privateData->stopThread)
	{
		runGlobalAdjustment(true);
		std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
		if (!privateData->wakeupSent) {
		  privateData->wakeupCond.wait(lck);
		}
		privateData->wakeupSent = false;
	}
#endif
}

/// @brief 由于在estimationThreadMain的时候条件变量对象wakeupCond中的wait()函数被调用，同时使用了std::unique_lock来锁住当前线程，因此当前线程会被一直堵塞，直至调用了wakeupSeparateThread()函数，
///        在wakeupSeparateThread()函数中，条件变量对象wakeupCond调用了notification函数唤醒了当前线程
void ITMGlobalAdjustmentEngine::wakeupSeparateThread(void)
{
#ifndef NO_CPP11
	std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
	privateData->wakeupSent = true;
	privateData->wakeupCond.notify_all();
#endif
}

/// @brief 构建pose graph optimizer
void ITMGlobalAdjustmentEngine::MultiSceneToPoseGraph(const ITMMapGraphManager & src, MiniSlamGraph::PoseGraph & dest)
{
        /// 步骤1:添加每个子地图相对于世界坐标系的位姿
        /// 为pose graph optimizer添加节点，即每个子地图在世界坐标系下的位姿,将第一个子地图的位姿设为固定
	for (int localMapId = 0; localMapId < (int)src.numLocalMaps(); ++localMapId)
	{
		MiniSlamGraph::GraphNodeSE3 *pose = new MiniSlamGraph::GraphNodeSE3();

		pose->setId(localMapId);
		pose->setPose(src.getEstimatedGlobalPose(localMapId));
		//将第一个子地图设为fix node
		if (localMapId == 0) {
		  pose->setFixed(true);
		}
		dest.addNode(pose);
	}

	/// 步骤2:添加不同子地图之间的约束
	/// 由于需要进行pose graph的优化，因此需要添加每个节点之间的添加一对多或者多对一约束
	for (int localMapId = 0; localMapId < (int)src.numLocalMaps(); ++localMapId) 
	{
		const ConstraintList & constraints = src.getConstraints(localMapId);
		for (ConstraintList::const_iterator it = constraints.begin(); it != constraints.end(); ++it) 
		{
			MiniSlamGraph::GraphEdgeSE3 *odometry = new MiniSlamGraph::GraphEdgeSE3();
			
			///localMapId为当前节点的id
			odometry->setFromNodeId(localMapId);
			///it->first为与当前节点产生约束的id,T-{localMapId,it->first}
			odometry->setToNodeId(it->first);
			///添加倆子地图的约束，也就是观测值
			odometry->setMeasurementSE3(it->second.GetAccumulatedObservations());
			
			//TODO odometry->setInformation
			//添加边
			dest.addEdge(odometry);
		}
	}
}

///@brief 用pose graph optimizer优化后的子地图的位姿调整ITMMapGraphManager中的位姿(相对于世界坐标系而言)
void ITMGlobalAdjustmentEngine::PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph & src, ITMMapGraphManager & dest)
{
	for (int localMapId = 0; localMapId < (int)dest.numLocalMaps(); ++localMapId) 
	{
		MiniSlamGraph::SlamGraph::NodeIndex::const_iterator it = src.getNodeIndex().find(localMapId);
		if (it == src.getNodeIndex().end()) continue;
		const MiniSlamGraph::GraphNodeSE3 *pose = (const MiniSlamGraph::GraphNodeSE3*)it->second;
		ITMPose outpose = pose->getPose();
		dest.setEstimatedGlobalPose(localMapId, outpose);
	}
}

