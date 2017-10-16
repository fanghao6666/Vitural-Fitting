#pragma once
#include "vao_buffer.h"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>

using namespace std;


//just for simulation, no render info
struct Face
{
	unsigned int vertex_index[3];
	unsigned int tex_index[3];
	unsigned int normal_index[3];
};

enum cloth_type { SINGLE_LAYER_NOB, SINGLE_LAYER_BOUNDARY };
enum direction {X,Y,Z};
class Obj
{
public:
	Obj();
	Obj(const string file,cloth_type type = SINGLE_LAYER_BOUNDARY);
	~Obj();

	void scale_translate(float S, float x_up, float y_up, float z_up);

	// unify the data, so that one vertex -> one normal -> one texture, 
	// or error acurred while rendering
	void unified();  
	cloth_type get_obj_type();
	void rotation(float angle, direction dir);
	//沿着NORMAL方向扩展点，不同于SCALE
	void vertex_extend(float dist);   
	


public:
	vector<glm::vec4> uni_vertices;             //unifieed data
	vector<glm::vec2> uni_tex;
	vector<glm::vec3> uni_normals;
	vector<unsigned int> vertex_index;           // unified the index for render
	GLuint g_textureID;
	VAO_Buffer vbo;
                


	string obj_file;                  //load from *.obj
	string mtl_file;
	string texture_file;

	vector<glm::vec4> vertices; 
	vector<glm::vec3> normals;
	vector<glm::vec2> tex;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;

private:
	
	cloth_type obj_type;
	glm::vec3 get_center();

};


