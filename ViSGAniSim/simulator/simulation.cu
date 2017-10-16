
#include "cuda_simulation.h"
#include "spring.h"
#include "load_obj.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <iostream>
using namespace std;

extern GLenum GL_MODE;

__global__ void get_face_normal(glm::vec4* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face);   //update cloth face normal
__global__ void verlet(glm::vec4* pos_vbo, glm::vec4 * g_pos_in, glm::vec4 * g_pos_old_in, glm::vec4 * g_pos_out, glm::vec4 * g_pos_old_out,glm::vec4* const_pos,
					  unsigned int* neigh1, unsigned int* neigh2,
					  glm::vec3* p_normal, unsigned int* vertex_adjface, glm::vec3* face_normal,
					  const unsigned int NUM_VERTICES,
					  BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,glm::vec3* collision_force, int* collided_vertex);  //verlet intergration

CUDA_Simulation::CUDA_Simulation()
{
	
}

CUDA_Simulation::~CUDA_Simulation()
{
	
}

CUDA_Simulation::CUDA_Simulation(Obj& cloth, Springs& springs):readID(0), writeID(1),sim_cloth(&cloth),NUM_ADJFACE(sim_parameter.NUM_ADJFACE),cuda_spring(&springs)
{
	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&cuda_vbo_resource, sim_cloth->vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	get_vertex_adjface();     //必须位于init_cuda前面，否则邻域数据为空
	init_cuda();              //将相关数据传送GPU


		
}

void CUDA_Simulation::simulate()
{
	size_t num_bytes;
	cudaError_t cudaStatus = cudaGraphicsMapResources(1, &cuda_vbo_resource, 0);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&cuda_p_vertex, &num_bytes, cuda_vbo_resource);
	cuda_p_normal = (glm::vec3*)((float*)cuda_p_vertex + 4 * sim_cloth->uni_vertices.size() + 2 * sim_cloth->uni_tex.size());   // 获取normal位置指针

	//cuda kernel compute .........
	verlet_cuda();
	cudaStatus = cudaGraphicsUnmapResources(1, &cuda_vbo_resource, 0);
	swap_buffer();
}

void CUDA_Simulation::init_cuda()
{
	size_t heap_size = 256 * 1024 * 1024;  //set heap size, the default is 8M
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	//将sim_cloth的点的坐标发送到GPU
	cudaError_t cudaStatus;      
	const unsigned int vertices_bytes = sizeof(glm::vec4) * sim_cloth->uni_vertices.size();
	cudaStatus = cudaMalloc((void**)&const_cuda_pos, vertices_bytes); // cloth vertices (const)
	cudaStatus = cudaMalloc((void**)&X[0], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X[1], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X_last[0], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&X_last[1], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&collision_force, sizeof(glm::vec3) * sim_cloth->uni_vertices.size());  //collision response force
	cudaMemset(collision_force, 0, sizeof(glm::vec3) * sim_cloth->uni_vertices.size());    //initilize to 0

	X_in = X[readID];
	X_out = X[writeID];
	X_last_in = X_last[readID];
	X_last_out = X_last[writeID];

	cudaStatus = cudaMemcpy(const_cuda_pos, &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X[0], &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X_last[0], &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	//计算normal所需的数据：每个点邻接的面的索引 + 每个面的3个点的索引 + 以及所有点的索引（虽然OPENGL有该数据）
	const unsigned int vertices_index_bytes = sizeof(unsigned int) * sim_cloth->vertex_index.size();       //点的索引
	cudaStatus = cudaMalloc((void**)&cuda_vertex_index, vertices_index_bytes);	
	cudaStatus = cudaMemcpy(cuda_vertex_index, &sim_cloth->vertex_index[0], vertices_index_bytes, cudaMemcpyHostToDevice);

	const unsigned int face_normal_bytes = sizeof(glm::vec3) * sim_cloth->faces.size();    //面的法向量
	cudaStatus = cudaMalloc((void**)&cuda_face_normal, face_normal_bytes);

	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //每个点邻接的面的索引
	cudaStatus = cudaMalloc((void**)&cuda_vertex_adjface, vertex_adjface_bytes);
	cudaStatus = cudaMemcpy(cuda_vertex_adjface, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice);
	
	//弹簧信息，即两级邻域点信息传送GPU
	cuda_neigh1 = cuda_spring->cuda_neigh1;
	cuda_neigh2 = cuda_spring->cuda_neigh2;

	//debug
	cudaMalloc((void**)&collided_vertex, sizeof(int)*sim_cloth->uni_vertices.size());
	cudaMemset(collided_vertex, 0, sizeof(int)*sim_cloth->uni_vertices.size());
	cpu_collided_veretx.resize(sim_cloth->uni_vertices.size());
	updated_vertex.resize(sim_cloth->uni_vertices.size());
}

void CUDA_Simulation::get_vertex_adjface()
{
	vector<vector<unsigned int>> adjaceny(sim_cloth->uni_vertices.size());
	for(int i=0;i<sim_cloth->faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth->faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	//test
	/*for(int i=0;i<10;i++)
	{
		for(int j=0;j<adjaceny[i].size();j++)
			cout << adjaceny[i][j] << "  ";
		cout << endl;
		
	}
*/
	vertex_adjface.resize(sim_cloth->uni_vertices.size()*NUM_ADJFACE);
	for(int i=0;i<adjaceny.size();i++)
	{
		int j;
		for(j=0;j<adjaceny[i].size() && j<NUM_ADJFACE;j++)
		{
			vertex_adjface[i*NUM_ADJFACE+j] = adjaceny[i][j];
		}
		if(NUM_ADJFACE>adjaceny[i].size())
			vertex_adjface[i*NUM_ADJFACE+j] = UINT_MAX;                  //Sentinel
	}
}

void CUDA_Simulation::verlet_cuda()
{
	cudaError_t cudaStatus;
	unsigned int numThreads0, numBlocks0;
	computeGridSize(sim_cloth->faces.size(), 512, numBlocks0, numThreads0);
	unsigned int cloth_index_size = sim_cloth->vertex_index.size(); 
	get_face_normal <<<numBlocks0, numThreads0 >>>(X_in, cuda_vertex_index, cloth_index_size, cuda_face_normal);  
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n", cudaStatus, cudaGetErrorString(cudaStatus));

	
	unsigned int numThreads, numBlocks;
	unsigned int numParticles = sim_cloth->uni_vertices.size();
	

	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(cuda_p_vertex, X_in, X_last_in, X_out, X_last_out,const_cuda_pos,
										cuda_neigh1,cuda_neigh2,
										cuda_p_normal,cuda_vertex_adjface,cuda_face_normal,
										numParticles,
										d_leaf_nodes,d_internal_nodes,d_primitives, collision_force,
										collided_vertex);

	// stop the CPU until the kernel has been executed
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "verlet cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n",
			cudaStatus, cudaGetErrorString(cudaStatus));
		exit(-1);
	}

	//debug
	//cudaMemcpy(&cpu_collided_veretx[0],collided_vertex,sizeof(int)*numParticles, cudaMemcpyDeviceToHost);
	//cudaMemcpy(&updated_vertex[0], cuda_p_vertex,sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);
	//cout << "*****collided veretx index************" << endl;
	//for (int i = 0; i < cpu_collided_veretx.size(); i++)
	//{
	//	if (cpu_collided_veretx[i] == 1)
	//		cout << i << "  ";
	//}
	//cout << endl;

}

void CUDA_Simulation::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void CUDA_Simulation::swap_buffer()
{
	int tmp = readID;
	readID = writeID;
	writeID = tmp;

	X_in = X[readID];
	X_out = X[writeID];
	X_last_in = X_last[readID];
	X_last_out = X_last[writeID];

}

void CUDA_Simulation::add_bvh(BVHAccel& bvh)
{
	d_leaf_nodes = bvh.d_leaf_nodes;
	d_internal_nodes = bvh.d_internal_nodes;
	d_primitives = bvh.d_primitives;
}



void CUDA_Simulation::draw_collided_vertex()
{

	//draw outline first
		for (int i = 0; i < sim_cloth->faces.size(); i++)
		{
			glm::vec4 ver[3];
			glm::vec3 normal[3];
			for (int j = 0; j < 3; j++)
			{
				ver[j] = updated_vertex[sim_cloth->faces[i].vertex_index[j]];
			}
			glPointSize(1.0);
			glBegin(GL_MODE);
			glColor3f(1.0, 1.0,1.0);
			for (int j = 0; j < 3; j++)
			{
				glVertex3f(ver[j].x, ver[j].y, ver[j].z);
			}
				
			glEnd();
		}


	for (int i = 0; i < cpu_collided_veretx.size(); i++)
	{
		glm::vec4 v = updated_vertex[i];
		if (cpu_collided_veretx[i] == 1)
		{
			//draw it
			glPointSize(10.0);
			glBegin(GL_POINTS);
				glColor3f(1.0, 0, 0);
				glVertex3f(v.x, v.y, v.z);
			glEnd();
		}
	}
}