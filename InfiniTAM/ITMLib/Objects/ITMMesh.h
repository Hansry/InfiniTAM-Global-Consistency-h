// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/Image.h"

#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <iostream>

namespace ITMLib
{
	namespace Objects
	{
		class ITMMesh
		{
		public:
			struct Triangle { 
			  Vector3f p0, p1, p2; 
			  Vector3f c0, c1, c2;
			};
		  
// 		        struct Triangle { 
// 			  Vector3f p0, p1, p2; 
// 			};
		
			MemoryDeviceType memoryType;

			uint noTotalTriangles;
			static const uint noMaxTriangles = SDF_LOCAL_BLOCK_NUM * NO_TRIANGLES; //default: 64

			ORUtils::MemoryBlock<Triangle> *triangles;

			explicit ITMMesh(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;
				this->noTotalTriangles = 0;

				triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
			}

			void WriteOBJ(const char *fileName)
			{
				ORUtils::MemoryBlock<Triangle> *cpu_triangles;
				bool shouldDelete = false;
				if (memoryType == MEMORYDEVICE_CUDA)
				{
					printf("Copying generated mesh from GPU to CPU memory...\n");
					cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
					cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
					shouldDelete = true;
					printf("Triangles copied to RAM.\n");
				}
				else cpu_triangles = triangles;

				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

				if (noTotalTriangles > noMaxTriangles) {
					std::stringstream ss;
					ss << "Unable to save mesh to file [" << fileName << "]. Too many triangles: "
					   << noTotalTriangles << " while the maximum is " << noMaxTriangles << ".";
					throw std::runtime_error(ss.str());
				}

				FILE *f = fopen(fileName, "w+");

				if (f != NULL)
				{
					// TODO(andrei): Do this in chunks. It doesn't seem like the order of the
					// triangles matters, so we can just launch multiple phases of the meshing
					// kernel, which runs fast but is memory-hungry, in order to really support
					// arbitrary-sized maps.
					printf("Starting to write mesh...\n");
					int count = 0;
					for (uint i = 0; i < noTotalTriangles; i++) {
						if ((i + 1) % 100000 == 0) {
							printf("Triangle %d/%d\n", i + 1, noTotalTriangles);
						}
						const Vector3f &c0 = triangleArray[i].c0;
						const Vector3f &c1 = triangleArray[i].c1;
						const Vector3f &c2 = triangleArray[i].c2;
						if((c0.r < 0.01 && c0.b < 0.01 && c0.g < 0.01) || (c1.r < 0.01 && c1.b < 0.01 && c1.g < 0.01) || (c2.r < 0.01 && c2.b < 0.01 && c2.g < 0.01)){
						  count ++;
						  continue;
						}
						fprintf(f,
								"v %f %f %f %f %f %f\n",
								triangleArray[i].p0.x,
								triangleArray[i].p0.y,
								triangleArray[i].p0.z,
								c0.r,
								c0.g,
								c0.b);
						fprintf(f,
								"v %f %f %f %f %f %f\n",
								triangleArray[i].p1.x,
								triangleArray[i].p1.y,
								triangleArray[i].p1.z,
								c1.r,
								c1.g,
								c1.b);
						fprintf(f,
								"v %f %f %f %f %f %f\n",
								triangleArray[i].p2.x,
								triangleArray[i].p2.y,
								triangleArray[i].p2.z,
								c2.r,
								c2.g,
								c2.b);
					}

					for (uint i = 0; i<(noTotalTriangles-count); i++) {
						fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
					}
					fclose(f);

					printf("Mesh file writing to [%s] complete.\n", fileName);
				}
				else {
					throw std::runtime_error("Could not open file for writing the mesh.\n");
				}

				if (shouldDelete) delete cpu_triangles;
			}
			
// 			void WriteOBJ(const char *fileName)
// 			{
// 				ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
// 				if (memoryType == MEMORYDEVICE_CUDA)
// 				{
// 					cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
// 					cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
// 					shoulDelete = true;
// 				}
// 				else cpu_triangles = triangles;
// 
// 				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);
// 
// 				FILE *f = fopen(fileName, "w+");
// 				if (f != NULL)
// 				{
// 					for (uint i = 0; i < noTotalTriangles; i++)
// 					{
// 						fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
// 						fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
// 						fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
// 					}
// 
// 					for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
// 					fclose(f);
// 				}
// 
// 				if (shoulDelete) delete cpu_triangles;
// 			}
                  
			void WriteSTL(const char *fileName)
			{
				ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
				if (memoryType == MEMORYDEVICE_CUDA)
				{
					cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
					cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
					shoulDelete = true;
				}
				else cpu_triangles = triangles;

				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

				FILE *f = fopen(fileName, "wb+");

				if (f != NULL) {
					for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

					fwrite(&noTotalTriangles, sizeof(int), 1, f);

					float zero = 0.0f; short attribute = 0;
					for (uint i = 0; i < noTotalTriangles; i++)
					{
						fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
						fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
						fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

						fwrite(&attribute, sizeof(short), 1, f);

						//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
						//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
						//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
					}

					//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
					fclose(f);
				}

				if (shoulDelete) delete cpu_triangles;
			}

			~ITMMesh()
			{
				delete triangles;
			}

			// Suppress the default copy constructor and assignment operator
			ITMMesh(const ITMMesh&);
			ITMMesh& operator=(const ITMMesh&);
		};
	}
}
