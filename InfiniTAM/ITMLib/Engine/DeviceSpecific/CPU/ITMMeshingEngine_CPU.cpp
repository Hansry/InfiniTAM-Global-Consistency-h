// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CPU.h"
#include "../../DeviceAgnostic/ITMMeshingEngine.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMMeshingEngine_CPU(void) 
{
}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMMeshingEngine_CPU(void) 
{
}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index.GetEntries();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	mesh->triangles->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) 
		  for (int y = 0; y < SDF_BLOCK_SIZE; y++) 
		    for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f vertList[12];
			int cubeIndex = buildVertList(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable);
			
			if (cubeIndex < 0) continue;

			for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
			{
			        Vector3f p0 = vertList[triangleTable[cubeIndex][i]];
				Vector3f p1 = vertList[triangleTable[cubeIndex][i + 1]];
				Vector3f p2 = vertList[triangleTable[cubeIndex][i + 2]];
				triangles[noTriangles].p0 = p0 * factor;
				triangles[noTriangles].p1 = p1 * factor;
				triangles[noTriangles].p2 = p2 * factor;
				
				Vector3f c0 =  VoxelColorReader<TVoxel::hasColorInformation, TVoxel, ITMVoxelBlockHash>::interpolate3(
								localVBA,
								hashTable,
								p0);
				
				Vector3f c1 =   VoxelColorReader<TVoxel::hasColorInformation, TVoxel, ITMVoxelBlockHash>::interpolate3(
								localVBA,
								hashTable,
								p1);
				
				Vector3f c2 =   VoxelColorReader<TVoxel::hasColorInformation, TVoxel, ITMVoxelBlockHash>::interpolate3(
								localVBA,
								hashTable,
								p2);

				triangles[noTriangles].c0 = c0;
				triangles[noTriangles].c1 = c1;
				triangles[noTriangles].c2 = c2;

				if (noTriangles < noMaxTriangles - 1) {
				  noTriangles++;
				}
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHHash>::ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHHash>::~ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHHashEntry *hashTable = scene->index.GetEntries();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.noTotalEntries;
	float smallestVoxelSize = scene->sceneParams->voxelSize;

	mesh->triangles->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;
		int level = ITMVoxelBlockHHash::GetLevelForEntry(entryId);

		globalPos = currentHashEntry.pos.toInt();

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f vertList[12];
			int cubeIndex = buildVertList(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable, level);

			if (cubeIndex < 0) continue;

			for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
			{
				triangles[noTriangles].p0 = vertList[triangleTable[cubeIndex][i]] * smallestVoxelSize;
				triangles[noTriangles].p1 = vertList[triangleTable[cubeIndex][i + 1]] * smallestVoxelSize;
				triangles[noTriangles].p2 = vertList[triangleTable[cubeIndex][i + 2]] * smallestVoxelSize;

				if (noTriangles < noMaxTriangles - 1) noTriangles++;
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMPlainVoxelArray>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{}

template class ITMLib::Engine::ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
