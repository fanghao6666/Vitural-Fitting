// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "load_obj.h"
#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "parameter.h"
#include "./bvh/bvh.h"
#include <iostream>
using namespace std;

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);


void get_primitives(Obj& body, vector<glm::vec3>& obj_vertices,vector<Primitive>& h_primitives)
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

int main(int argc, char** argv)
{
	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	//测试衣服
	//Obj cloth("../cloth/cloth.obj");    //pose0
	//cloth.scale_translate(0.31, 0, 1.95, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress2/dress2-iso.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.24, 0, 1.2, 0.02); 
	//cloth.unified();

	/*Obj cloth("../cloth_no_boundary/dress3/dress3.obj",SINGLE_LAYER_NOB);  
	cloth.rotation(90, X);   
	cloth.scale_translate(0.24, 0, 0.9, 0.02); 
	cloth.unified();*/

	//Obj cloth("../cloth_no_boundary/dress-asymmetric/dress-asymmetric.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.25, 0, 1.10, 0.02);
	//cloth.unified();

	/*Obj cloth("../cloth_no_boundary/dress-victor/dress-victor.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.scale_translate(0.25, 0, 1.60, 0.02);
	cloth.unified();*/

	//Obj cloth("../cloth_no_boundary/robe/robe.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.30, 0, 1.1, 0.02);
	//cloth.unified();

	/*Obj cloth("../cloth_no_boundary/tshirt/tshirt.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.rotation(-5, Z);
	cloth.scale_translate(0.29, 0, 2, 0.02);
	cloth.unified();*/

	/*Obj cloth("../cloth_no_boundary/shirt/shirt.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.rotation(-4, Z);
	cloth.scale_translate(0.28, 0, 2.0, 0.02);
	cloth.unified();*/

	/*Obj cloth("../cloth_no_boundary/skirt/skirt.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.scale_translate(0.29, 0, 0.5, 0);
	cloth.unified();*/

	/*Obj cloth("../cloth_no_boundary/tshirt2/tshirt2.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.rotation(-4, Z);
	cloth.scale_translate(0.26, 0, 1.9, 0.02);
	cloth.unified();*/


	//Obj cloth("../cloth_no_boundary/shorts/shorts.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.29, 0, 0.5, 0);
	//cloth.unified();

	Obj cloth("../cloth_no_boundary/vest/vest.obj",SINGLE_LAYER_NOB);  
	cloth.rotation(90, X);   
	cloth.scale_translate(0.30, 0, 1.7, 0.02); 
	cloth.unified();

	Springs cuda_spring(&cloth);  

	//Obj body("../pose/pose0.obj");
	//body.scale_translate(0.30, 0, 1.0, 0);
	//body.unified();

	Obj body("../pose/pose1.obj");
	body.scale_translate(0.30, 0, 1.0, 0);
	body.unified();
	
	//show body and cloth on the scene
	main_scene->add(cloth);
	main_scene->add(body);

	Obj bvh_body = body;
	bvh_body.vertex_extend(0.01);
	bvh_body.unified();


	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;
	get_primitives(bvh_body, obj_vertices, h_primitives);
	BVHAccel cuda_bvh(h_primitives);
	//cuda_bvh.pre_drawoutline();
	//main_scene->add(cuda_bvh);


	CUDA_Simulation simulation(cloth,cuda_spring);   //add(obj)会初始化gpu端数据，simulation需要用到这些数据
	simulation.add_bvh(cuda_bvh);
	main_scene->add(simulation);
	main_scene->render();
	
	return 0;
}

