#include "primitive.h"

BBox Primitive::get_bbox() const
{
	BBox bbox(vertices[v0]);
	bbox.expand(vertices[v1]);
	bbox.expand(vertices[v2]);

	//沿着法线方向适当拓展或收缩三角面片,后期改为点的各自法线方向
	float depth = 0.001;
	glm::vec3 n = get_normal();
	bbox.expand(vertices[v0] - depth*n);
	bbox.expand(vertices[v1] - depth*n);
	bbox.expand(vertices[v2] - depth*n);

	return bbox;
}

bool Primitive::intersect(const glm::vec3& point) const
{
	//use normal or barycentric coordinates
	glm::vec3 side1, side2, normalface;
	side1 = vertices[v1] - vertices[v0];
	side2 = vertices[v2] - vertices[v0];
	normalface = glm::cross(side1, side2);
	

	glm::vec3 tem = point - vertices[v0];
	
	if (glm::dot(tem, normalface) > 0)
		return true;

	return false;
}

glm::vec3 Primitive::get_normal() const
{
	glm::vec3 side1, side2, normalface;
	side1 = vertices[v1] - vertices[v0];
	side2 = vertices[v2] - vertices[v0];
	normalface = glm::cross(side1, side2);
	return glm::normalize(normalface);
}