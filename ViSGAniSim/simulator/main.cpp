// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "load_obj.h"
#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "parameter.h"
#include "./bvh/bvh.h"
#include"watch.h"
#include <iostream>
using namespace std;

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);
void get_primitives(Obj& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives);
int NumPose = 49;


int main(int argc, char** argv)
{
	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	////测试衣服
	//Obj cloth("../cloth/cloth.obj");    //pose0
	//cloth.scale_translate(0.31, 0, 1.95, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress2/dress2-iso.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.24, 0, 1.2, 0.02); 
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress3/dress3.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.24, 0, 0.9, 0.02); 
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress-asymmetric/dress-asymmetric.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.25, 0, 1.10, 0.02);
	//cloth.unified();

	Obj cloth("../cloth_no_boundary/dress-victor/dress-victor.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.scale_translate(0.25, 0, 1.60, 0.02);
	cloth.unified();

	//Obj cloth("../cloth_no_boundary/robe/robe.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.29, 0, 1.1, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/tshirt/tshirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.rotation(-5, Z);
	//cloth.scale_translate(0.29, 0, 2, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/shirt/shirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.rotation(-4, Z);
	//cloth.scale_translate(0.28, 0, 2.0, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/skirt/skirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.29, 0, 0.5, 0);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/tshirt2/tshirt2.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.rotation(-4, Z);
	//cloth.scale_translate(0.3, 0, 2.0, 0.02);
	//cloth.unified();


	//Obj cloth("../cloth_no_boundary/shorts/shorts.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.29, 0, 0.5, 0);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/vest/vest.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.30, 0, 1.7, 0.02); 
	//cloth.unified();

	Springs cuda_spring(&cloth);  


	vector<Obj> body;

	Obj now_body("../pose2/0.obj");
	now_body.scale_translate(0.30, 0, 1.0, 0.0);
	//now_body.scale_translate(0.30, 0, 0.96, -0.1);
	now_body.unified();
	main_scene->body.push_back(now_body);   //add initial pose

	for (int i =1;i<=NumPose;i++)
	{
		string file_name;
		switch (i)
		{
		case 0:
		{
			file_name = "../pose2/0.obj";
			break;
		}
		case 1:
		{
			file_name = "../pose2/1.obj";
			break;
		}
		case 2:
		{
			file_name = "../pose2/2.obj";
			break;
		}
		case 3:
		{
			file_name = "../pose2/3.obj";
			break;
		}
		case 4:
		{
			file_name = "../pose2/4.obj";
			break;
		}
		case 5:
		{
			file_name = "../pose2/5.obj";
			break;
		}
		case 6:
		{
			file_name = "../pose2/6.obj";
			break;
		}
		case 7:
		{
			file_name = "../pose2/7.obj";
			break;
		}
		case 8:
		{
			file_name = "../pose2/8.obj";
			break;
		}
		case 9:
		{
			file_name = "../pose2/9.obj";
			break;
		}
		case 10:
		{
			file_name = "../pose2/10.obj";
			break;
		}
		case 11:
		{
			file_name = "../pose2/11.obj";
			break;
		}
		case 12:
		{
			file_name = "../pose2/12.obj";
			break;
		}
		case 13:
		{
			file_name = "../pose2/13.obj";
			break;
		}
		case 14:
		{
			file_name = "../pose2/14.obj";
			break;
		}
		case 15:
		{
			file_name = "../pose2/15.obj";
			break;
		}
		case 16:
		{
			file_name = "../pose2/16.obj";
			break;
		}
		case 17:
		{
			file_name = "../pose2/17.obj";
			break;
		}
		case 18:
		{
			file_name = "../pose2/18.obj";
			break;
		}
		case 19:
		{
			file_name = "../pose2/19.obj";
			break;
		}
		case 20:
		{
			file_name = "../pose2/20.obj";
			break;
		}
		case 21:
		{
			file_name = "../pose2/21.obj";
			break;
		}
		case 22:
		{
			file_name = "../pose2/22.obj";
			break;
		}
		case 23:
		{
			file_name = "../pose2/23.obj";
			break;
		}
		case 24:
		{
			file_name = "../pose2/24.obj";
			break;
		}
		case 25:
		{
			file_name = "../pose2/25.obj";
			break;
		}
		case 26:
		{
			file_name = "../pose2/26.obj";
			break;
		}
		case 27:
		{
			file_name = "../pose2/27.obj";
			break;
		}
		case 28:
		{
			file_name = "../pose2/28.obj";
			break;
		}
		case 29:
		{
			file_name = "../pose2/29.obj";
			break;
		}
		case 30:
		{
			file_name = "../pose2/30.obj";
			break;
		}
		case 31:
		{
			file_name = "../pose2/31.obj";
			break;
		}
		case 32:
		{
			file_name = "../pose2/32.obj";
			break;
		}
		case 33:
		{
			file_name = "../pose2/33.obj";
			break;
		}
		case 34:
		{
			file_name = "../pose2/34.obj";
			break;
		}
		case 35:
		{
			file_name = "../pose2/35.obj";
			break;
		}
		case 36:
		{
			file_name = "../pose2/36.obj";
			break;
		}
		case 37:
		{
			file_name = "../pose2/37.obj";
			break;
		}
		case 38:
		{
			file_name = "../pose2/38.obj";
			break;
		}
		case 39:
		{
			file_name = "../pose2/39.obj";
			break;
		}
		case 40:
		{
			file_name = "../pose2/40.obj";
			break;
		}
		case 41:
		{
			file_name = "../pose2/41.obj";
			break;
		}
		case 42:
		{
			file_name = "../pose2/42.obj";
			break;
		}
		case 43:
		{
			file_name = "../pose2/43.obj";
			break;
		}
		case 44:
		{
			file_name = "../pose2/44.obj";
			break;
		}
		case 45:
		{
			file_name = "../pose2/45.obj";
			break;
		}
		case 46:
		{
			file_name = "../pose2/46.obj";
			break;
		}
		case 47:
		{
			file_name = "../pose2/47.obj";
			break;
		}
		case 48:
		{
			file_name = "../pose2/48.obj";
			break;
		}
		case 49:
		{
			file_name = "../pose2/49.obj";
			break;
		}

		default:
			break;
		}
		now_body.ChangeObj(file_name);
		//now_body.scale_translate(0.30, 0, 0.96, -0.1);
		now_body.scale_translate(0.30, 0, 1.0, 0);
		main_scene->body.push_back(now_body);
	}
	
	main_scene->add(cloth);
	main_scene->add(main_scene->body[0]);


	Obj bvh_body = main_scene->body[0];
	bvh_body.vertex_extend(0.02);
	bvh_body.unified();


	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;
	get_primitives(bvh_body, obj_vertices, h_primitives);

	now_body.~Obj();
	bvh_body.~Obj();


	BVHAccel cuda_bvh(h_primitives);


	CUDA_Simulation simulation(cloth,cuda_spring);   //add(obj)会初始化gpu端数据，simulation需要用到这些数据
	simulation.add_bvh(cuda_bvh);
	main_scene->add(simulation);
	main_scene->render();
	
	return 0;
}


