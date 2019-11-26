// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSwappingEngine_CPU : public ITMSwappingEngine < TVoxel, TIndex >
		{
		public:
			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
			void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		};

		template<class TVoxel>
		class ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		public:
			// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
			// Potentially this could stream into the host memory from somwhere else (disk, database, etc.).

			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
			void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
			
			/// NOTE 将判断是否可见的约束去掉，要做的是将整个子地图都swaping到host meomry中
			void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

			ITMSwappingEngine_CPU(void);
			~ITMSwappingEngine_CPU(void);
		};
	}
}
