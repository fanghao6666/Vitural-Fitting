#pragma once
#include<stdlib.h>
#include<iostream>
#include"load_obj.h"
#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "parameter.h"
#include"watch.h"
#include "./bvh/bvh.h"

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);

extern void get_primitives(Obj& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives)
{
	//prepare primitives
	obj_vertices.resize(body.uni_vertices.size());
	for (int i = 0; i < body.uni_vertices.size(); i++)
	{
		obj_vertices[i] = glm::vec3(body.uni_vertices[i].x,
			body.uni_vertices[i].y,
			body.uni_vertices[i].z);
	}
	glm::vec3* d_obj_vertices;
	copyFromCPUtoGPU((void**)&d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3)*obj_vertices.size());
	glm::vec3* h_obj_vertices = &obj_vertices[0];

	//create primitives
	h_primitives.resize(body.vertex_index.size() / 3);
	for (int i = 0; i < h_primitives.size(); i++)
	{
		Primitive tem_pri(h_obj_vertices, d_obj_vertices, body.vertex_index[i * 3 + 0],
			body.vertex_index[i * 3 + 1],
			body.vertex_index[i * 3 + 2]);
		h_primitives[i] = tem_pri;
	}

}