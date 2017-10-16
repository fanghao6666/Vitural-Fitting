#pragma once
#include "load_obj.h"
#include <set>
#include <vector>
#include <map>
#include "parameter.h"


class Matrix
{
public:
	Matrix() {};
	~Matrix();
	void Insert_Matrix(unsigned int i, unsigned int j, unsigned int k, vector<pair<unsigned int,unsigned int>> &value_inline);

private:
	map<pair<unsigned int,unsigned int>,unsigned int>  mat;

};


class Springs
{
public:
	Springs();
	~Springs();
	Springs(Obj* cloth);  //创建两级邻域，boundary and cloth弹簧劲度系数一致
	void draw();


public:
	unsigned int* cuda_neigh1;  //二维数组转为一维数组
	unsigned int* cuda_neigh2;
	unsigned int NUM_NEIGH1;    //一级邻域的最大数目，若小于NUM_NEIGH1，以MAX_INT结尾
	unsigned int NUM_NEIGH2;    //二级邻域的最大数目，若小于NUM_NEIGH2，以MAX_INT结尾

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //只包含pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //应该已经包含pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //存储每个点的所有一级邻域信息(存储点的索引),即 structure spring
	vector<vector<unsigned int>> neigh2;   //存储每个点的所有二级邻域信息(存储点的索引),即 bend spring
	Obj* spring_obj;

private:
	//void ad spring(float stiffness,vector<glm::vec4>& vertices,unsigned int p1,unsigned int p2);
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	bool cuda_neigh();
	void get_cloth_boundary_spring();
	void get_boundary_boundary_spring();
};