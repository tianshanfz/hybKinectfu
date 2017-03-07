
#include "CudaWrappers.h"
#include "CudaDeviceDataMan.h"
#include "marchingcube_table.h"
__device__ Vertex vertexInterp(float isolevel, const float3& p1, const float3& p2, float d1, float d2, const uchar3& c1, const uchar3& c2)
{
	Vertex r1; r1.pos = p1; r1.color = make_float3(c1.x, c1.y, c1.z) *(1.0/ 255.f);
	Vertex r2; r2.pos = p2; r2.color = make_float3(c2.x, c2.y, c2.z)*(1.0/ 255.f);

	if (abs(isolevel - d1) < 0.00001f)		return r1;
	if (abs(isolevel - d2) < 0.00001f)		return r2;
	if (abs(d1 - d2) < 0.00001f)			return r1;

	float mu = (isolevel - d1) / (d2 - d1);

	Vertex res;
	res.pos.x = p1.x + mu * (p2.x - p1.x); // Positions
	res.pos.y = p1.y + mu * (p2.y - p1.y);
	res.pos.z = p1.z + mu * (p2.z - p1.z);

	res.color.x = (float)(c1.x + mu * (float)(c2.x - c1.x)) / 255.f; // Color
	res.color.y = (float)(c1.y + mu * (float)(c2.y - c1.y)) / 255.f;
	res.color.z = (float)(c1.z + mu * (float)(c2.z - c1.z)) / 255.f;

	return res;
}
__device__ void appendTriangle(const Triangle& t,MarchingcubeData& data)
{
	if (*data.triangleNumPtr() >= data.maxTriangles() ){
		return; // todo
	}
	uint addr = atomicAdd(data.triangleNumPtr(), 1);

	Triangle& triangle = data.at(addr);
	triangle.v0 = t.v0;
	triangle.v1 = t.v1;
	triangle.v2 = t.v2;
	return;
}

__device__ void extractIsoSurfaceAtPosition(const int3& voxelPos,bool has_color, const tsdfvolume& volume,float threshold_marchingcube,MarchingcubeData& marchingcube_data)
{

	float3 worldPos = volume.voxelPosToWorld(voxelPos);
	const float isolevel = 0.0f;
	float3 cell_size;
	cell_size.x=volume.size().x/ volume.resolution().x;
	cell_size.y = volume.size().y / volume.resolution().y;
	cell_size.z = volume.size().z / volume.resolution().z;
	const float3 P = cell_size * 0.5f;
	const float3 M = cell_size*(-0.5f);
	float3 p000 = worldPos + make_float3(M.x, M.y, M.z); float dist000; uchar3 color000;
	if(false==volume.interpolateSDF(p000, dist000))return;
	if(has_color)volume.interpolateColor(p000, color000);
	float3 p100 = worldPos + make_float3(P.x, M.y, M.z); float dist100; uchar3 color100;
	if(false==volume.interpolateSDF(p100, dist100))return;
	if(has_color)volume.interpolateColor(p100, color100);
	float3 p010 = worldPos + make_float3(M.x, P.y, M.z); float dist010; uchar3 color010;
	if(false==volume.interpolateSDF(p010, dist010))return;
	if(has_color)volume.interpolateColor(p010, color010);
	float3 p001 = worldPos + make_float3(M.x, M.y, P.z); float dist001; uchar3 color001;
	if(false==volume.interpolateSDF(p001, dist001))return;
	if(has_color)volume.interpolateColor(p001, color001);
	float3 p110 = worldPos + make_float3(P.x, P.y, M.z); float dist110; uchar3 color110;
	if(false==volume.interpolateSDF(p110, dist110))return;
	if(has_color)volume.interpolateColor(p110, color110);
	float3 p011 = worldPos + make_float3(M.x, P.y, P.z); float dist011; uchar3 color011;
	if(false==volume.interpolateSDF(p011, dist011))return;
	if(has_color)volume.interpolateColor(p011, color011);
	float3 p101 = worldPos + make_float3(P.x, M.y, P.z); float dist101; uchar3 color101;
	if(false==volume.interpolateSDF(p101, dist101))return;
	if(has_color)volume.interpolateColor(p101, color101);
	float3 p111 = worldPos + make_float3(P.x, P.y, P.z); float dist111; uchar3 color111;
	if(false==volume.interpolateSDF(p111, dist111))return;
	if(has_color)volume.interpolateColor(p111, color111);

	uint cubeindex = 0;
	if (dist010 < isolevel) cubeindex += 1;
	if (dist110 < isolevel) cubeindex += 2;
	if (dist100 < isolevel) cubeindex += 4;
	if (dist000 < isolevel) cubeindex += 8;
	if (dist011 < isolevel) cubeindex += 16;
	if (dist111 < isolevel) cubeindex += 32;
	if (dist101 < isolevel) cubeindex += 64;
	if (dist001 < isolevel) cubeindex += 128;

/*		const float thres = m_params.fThreshMarchingCubes;
	float distArray[] = { dist000, dist100, dist010, dist001, dist110, dist011, dist101, dist111 };
//		printf("dist000%.2f dist111:%.2f ", dist000, dist111);
	for (uint k = 0; k < 8; k++) {
		for (uint l = 0; l < 8; l++) {
			if (distArray[k] * distArray[l] < 0.0f) {
				if (abs(distArray[k]) + abs(distArray[l]) > thres) return;
			}
			else {
				if (abs(distArray[k] - distArray[l]) > thres) return;
			}
		}
	}*/

	if (abs(dist000) >threshold_marchingcube) return;
	if (abs(dist100) >threshold_marchingcube) return;
	if (abs(dist010) >threshold_marchingcube) return;
	if (abs(dist001) >threshold_marchingcube) return;
	if (abs(dist110) >threshold_marchingcube) return;
	if (abs(dist011) >threshold_marchingcube) return;
	if (abs(dist101) > threshold_marchingcube) return;
	if (abs(dist111) >threshold_marchingcube) return;

	if (edgeTable[cubeindex] == 0 || edgeTable[cubeindex] == 255) return; // added by me edgeTable[cubeindex] == 255

	Voxel v = volume.getVoxel(voxelPos);


	Vertex vertlist[12];
	if (edgeTable[cubeindex] & 1)	vertlist[0] = vertexInterp(isolevel, p010, p110, dist010, dist110,color010,color110);
	if (edgeTable[cubeindex] & 2)	vertlist[1] = vertexInterp(isolevel, p110, p100, dist110, dist100, color110,color100);
	if (edgeTable[cubeindex] & 4)	vertlist[2] = vertexInterp(isolevel, p100, p000, dist100, dist000, color100,color000);
	if (edgeTable[cubeindex] & 8)	vertlist[3] = vertexInterp(isolevel, p000, p010, dist000, dist010, color000,color010);
	if (edgeTable[cubeindex] & 16)	vertlist[4] = vertexInterp(isolevel, p011, p111, dist011, dist111, color011,color111);
	if (edgeTable[cubeindex] & 32)	vertlist[5] = vertexInterp(isolevel, p111, p101, dist111, dist101, color111,color101);
	if (edgeTable[cubeindex] & 64)	vertlist[6] = vertexInterp(isolevel, p101, p001, dist101, dist001, color101,color001);
	if (edgeTable[cubeindex] & 128)	vertlist[7] = vertexInterp(isolevel, p001, p011, dist001, dist011,color001,color011);
	if (edgeTable[cubeindex] & 256)	vertlist[8] = vertexInterp(isolevel, p010, p011, dist010, dist011, color010,color011);
	if (edgeTable[cubeindex] & 512)	vertlist[9] = vertexInterp(isolevel, p110, p111, dist110, dist111, color110,color111);
	if (edgeTable[cubeindex] & 1024) vertlist[10] = vertexInterp(isolevel, p100, p101, dist100, dist101,color100,color101);
	if (edgeTable[cubeindex] & 2048) vertlist[11] = vertexInterp(isolevel, p000, p001, dist000, dist001, color000,color001);

	for (int i = 0; triTable[cubeindex][i] != -1; i += 3)
	{
		Triangle t;
		t.v0 = vertlist[triTable[cubeindex][i + 0]];
		t.v1 = vertlist[triTable[cubeindex][i + 1]];
		t.v2 = vertlist[triTable[cubeindex][i + 2]];
		appendTriangle(t,marchingcube_data);
	}
}
__global__ void extractIsoSurfaceKernel(bool has_color,tsdfvolume volume,float threshold_marchingcube,MarchingcubeData marchingcube_data)
{

	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= volume.resolution().x || y >= volume.resolution().y)
	{
		return;
	}
	//	float3 worldCamPos = make_float3(camera.getTransformMat()*make_float4(0.0f, 0.0f, 0.0f, 1.0f));//ÊÓµãÎ»ÖÃ
	for (int z = 0; z < volume.resolution().z; z++)
	{
		//float3 worldPos = volume.voxelPosToWorld(make_int3(x,y,z));
		extractIsoSurfaceAtPosition(make_int3(x,y,z), has_color,volume,threshold_marchingcube,marchingcube_data);
	}
}
void cudaMarchingcube(bool has_color,float threshold_marchingcube)
{
	tsdfvolume volume=CudaDeviceDataMan::instance()->volume;

	MarchingcubeData marchingcube_data =CudaDeviceDataMan::instance()->marchingcube_data;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(volume.resolution().x, BLOCK_SIZE_2D_X), divUp(volume.resolution().y, BLOCK_SIZE_2D_Y));
	extractIsoSurfaceKernel << <gridSize, blockSize >> >(has_color,volume,threshold_marchingcube,marchingcube_data);
	cudaDeviceSynchronize();

}
