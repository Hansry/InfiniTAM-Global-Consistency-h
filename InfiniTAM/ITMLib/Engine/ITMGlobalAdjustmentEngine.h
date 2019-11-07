// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../MiniSlamGraphLib/PoseGraph.h"
#include "ITMMapGraphManager.h"

namespace ITMLib {
  namespace Engine {
        /**
	 * @brief 
	 * 该engine使用位姿图优化（pose graph optimisation）计算全局位姿调整。其基本思想是，
	 * 每当某个main engine认为有必要时，它使用updateMeasurements()函数向全局位姿调整发送新信息，无论出于什么原因，如果engine拒绝这些新的观测量，updateMeasurements将会立即返回false,
	 * 而main eigen需要再次提交相同搞得数据。另一方面，retrieveNewEstimates()允许main engine从位姿图
	 * 优化中检索结果（如果有的话），如果没有则为false
	 * 
	 * 
	 * 位姿图优化本身可以通过函数runGlobalAdjustment()方法显示地调用。但是，整个类被设计成在后台的分离线程中运行。
	 * 相应的方法是运行startSeparateThread()和stopSeparateThread(),每当传递新的观测时，还建议调用wakeupSeparateThread().
	 * 当位姿图优化正在进行时，线程将拒绝新的数据，否则它可能会休眠
	 */

	/**     This engine computes global pose adjustments using pose graph optimisation.
		The basic idea is that whenever some "main engine" considers it necessary,
		it should send new information to such a global adjustment step using the
		function updateMeasurements(). Should for whatever reason the engine reject
		these new measurements, this function should return false immediately, and
		the main engine has to keep resubmitting the same data again. On the other
		hand, retrieveNewEstimates() allows the main engine to retrieve the results
		from pose graph optimisation, if there are any, or false, if there aren't.

		The pose graph optimisation itself can be called explicitly using the method
		runGlobalAdjustment(). However, the whole class is also designed to be run
		in a separate thread in the background. The corresponding methods are
		startSeparateThread() and stopSeparateThread(), and whenever new
		measurements are being passed, a call to wakeupSeparateThread() is also
		recommended. The thread will reject new data while a pose graph optimisation
		is currently in progress, and it may go to sleep otherwise.
	*/
	class ITMGlobalAdjustmentEngine {
	private:
		struct PrivateData;

	public:
		ITMGlobalAdjustmentEngine(void);
		~ITMGlobalAdjustmentEngine(void);

		bool hasNewEstimates(void) const;

		// Check whether pose graph optimisation has converged and produced a
		// new result. if it hasn't return false, otherwise copy them over
		bool retrieveNewEstimates(ITMMapGraphManager & dest);

		bool isBusyEstimating(void) const;

		// Check whether thread is busy, if it is, return false, otherwise
		// create a copy of all new measurements and make it busy
		
		/// @brief 根据当前地图观测量并通过multiScenToPoseGraph建立pose graph优化,如果之前的数据还在更新优化中，则暂时不对当前观测进行优化
		bool updateMeasurements(const ITMMapGraphManager & src);

		bool runGlobalAdjustment(bool blockingWait = false);

		bool startSeparateThread(void);
		bool stopSeparateThread(void);
		
		/// @brief 由于在estimationThreadMain的时候条件变量对象wakeupCond中的wait()函数被调用，同时使用了std::unique_lock来锁住当前线程，因此当前线程会被一直堵塞，直至调用了wakeupSeparateThread()函数，
		///        在wakeupSeparateThread()函数中，条件变量对象wakeupCond调用了notification函数唤醒了当前线程
		void wakeupSeparateThread(void);

	private:
	        /// @brief 线程的主要入口
		void estimationThreadMain(void);

		/// @brief 构建pose graph optimizer
		static void MultiSceneToPoseGraph(const ITMMapGraphManager & src, MiniSlamGraph::PoseGraph & dest);
		
		///@brief 用pose graph optimizer优化后的子地图的位姿调整ITMMapGraphManager中的位姿(相对于世界坐标系而言)
		static void PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph & src, ITMMapGraphManager & dest);

		MiniSlamGraph::PoseGraph *workingData;
		MiniSlamGraph::PoseGraph *processedData;

		PrivateData *privateData;
	};
  } //namespace Engine
} //namespace ITMLib
