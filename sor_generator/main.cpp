#define _USE_MATH_DEFINES // for C++  
#include <cmath> 

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

void ConnectRing(TriMesh &mesh, std::vector<TriMesh::VertexHandle> &bottom, std::vector<TriMesh::VertexHandle> &top)
{
	if (bottom.size() != top.size()) {
		printf("Bottom and top must have the same lengths\n");
		printf("They will not be connected\n");
		return;
	}

	for (int i = 1; i < bottom.size(); i++) {
		std::vector<TriMesh::VertexHandle>  face1 = { bottom[i], bottom[i - 1], top[i] };
		mesh.add_face(face1);
		std::vector<TriMesh::VertexHandle>  face2 = { bottom[i - 1], top[i - 1], top[i] };
		mesh.add_face(face2);
	}
	std::vector<TriMesh::VertexHandle>  face3 = { bottom[0], bottom[bottom.size() - 1], top[0] };
	mesh.add_face(face3);
	std::vector<TriMesh::VertexHandle>  face4 = { bottom[bottom.size() - 1], top[bottom.size() - 1], top[0] };
	mesh.add_face(face4);
}

std::vector<TriMesh::VertexHandle> CreateRing(TriMesh &mesh, double height, double radius, int num_divisions)
{
	std::vector<TriMesh::VertexHandle> ring;
	for (int i = 0; i < num_divisions; i++) {
		double theta = i * 2 * M_PI / num_divisions;
		ring.push_back(mesh.add_vertex(TriMesh::Point(radius * cos(theta), height, radius * sin(theta))));
	}
	return ring;
}

std::vector<TriMesh::VertexHandle> CreatePlane(TriMesh &mesh, double height, double radius, int num_divisions/*, int num_radial_divisions*/)
{
	std::vector<TriMesh::VertexHandle> ring;
	TriMesh::VertexHandle center_point = mesh.add_vertex(TriMesh::Point(0, height, 0));
	for (int i = 0; i < num_divisions; i++) {
		double theta = i * 2 * M_PI / num_divisions;
		ring.push_back(mesh.add_vertex(TriMesh::Point(radius * cos(theta), 
													  height, 
			                                          radius * sin(theta))));

		if (i > 0) {
			std::vector<TriMesh::VertexHandle>  face_vhandles = { center_point, ring[i - 1], ring[i] };
			mesh.add_face(face_vhandles);
		}
	}
	std::vector<TriMesh::VertexHandle>  face_vhandles = { center_point, ring[num_divisions - 1], ring[0] };
	mesh.add_face(face_vhandles);

	return ring;
}



void CloseRing(TriMesh &mesh, std::vector<TriMesh::VertexHandle> &ring, double height)
{
	TriMesh::VertexHandle center_point = mesh.add_vertex(TriMesh::Point(0, height, 0));
	for (int i = 1; i < ring.size(); i++) {
		std::vector<TriMesh::VertexHandle>  face1 = { ring[i], ring[i - 1], center_point };
		mesh.add_face(face1);
	}
	std::vector<TriMesh::VertexHandle>  face2 = { ring[0], ring[ring.size() - 1], center_point };
	mesh.add_face(face2);
}

bool CreateSOR(std::string path, std::vector<double> radi, double height, double bottom_elevation, int num_divisions, double thickness)
{
	TriMesh mesh;

	// Creates the outside bottom
	std::vector<TriMesh::VertexHandle> prev_ring = CreatePlane(mesh, bottom_elevation - thickness, radi[0] + thickness, num_divisions);
	// Creates the outside surface
	double outside_height_step = (height) / (radi.size() - 1);
	for (int i = 1; i < radi.size(); i++) {
		std::vector<TriMesh::VertexHandle> curr_ring = CreateRing(mesh, bottom_elevation + outside_height_step*i, 
			radi[i] + thickness, num_divisions);
		ConnectRing(mesh, prev_ring, curr_ring);
		prev_ring = curr_ring;
	}

	// Creates the inner surface
	double inside_height_step = (height) / (radi.size() - 1);
	for (int i = radi.size() - 1; i >= 0; i--) {
		std::vector<TriMesh::VertexHandle> curr_ring = CreateRing(mesh, bottom_elevation + inside_height_step*i, radi[i], num_divisions);
		ConnectRing(mesh, prev_ring, curr_ring);
		prev_ring = curr_ring;
	}
	CloseRing(mesh, prev_ring, bottom_elevation);

	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, path))
		{
			std::cerr << "Cannot write mesh to file '" << path << "'" << std::endl;
			return false;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
		return false;
	}
	return true;
}

void WriteConfig(std::string path, double min_radius, double max_radius, double height)
{
	std::ofstream config(path);
	config << min_radius << std::endl;
	config << max_radius << std::endl;
	config << height << std::endl;
	config.close();
}

void CalcCircleCenter(double x1, double y1, double x2, double y2, double x3, double y3, 
	double &radius, double &cx, double&cy) 
{
	double ma = (y2 - y1) / (x2 - x1);
	double mb = (y3 - y2) / (x3 - x2);
	
	cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma *(x2 + x3)) / (2 * (mb - ma));
	cy = -1.0 / ma * (cx - (x1 + x2) / 2) + (y1 + y2) / 2;

	radius = sqrt((cx - x1)*(cx - x1) + (cy - y1)*(cy - y1));
}

std::vector<double> GenerateRadi(double step_size, 
	double main_height, double base_radius, double max_radius, double neck_radius)
{
	std::vector<double> radi;

	radi.push_back(base_radius - step_size);
	//guarantees that there is not a right angle at the bottom of the container
	// This prevents leaks
	radi.push_back(base_radius);

	double radius, centerX, centerY;
	CalcCircleCenter(base_radius, 0, max_radius, main_height / 2, neck_radius, main_height, radius, centerX, centerY);
	//printf("Circle: center (%f %f) radius: %f\n", centerX, centerY, radius);
	for (int i = 1; i < main_height / step_size + 1; i++) {
		double y = i * step_size;
		double x;
		if ( centerX > 0 ) {
			x = centerX - sqrt(radius*radius - (centerY - y) * (centerY - y));
		}
		else {
			x = centerX + sqrt(radius*radius - (centerY - y) * (centerY - y));
		}
			
		//printf("%f, %f\n", x, centerX + sqrt(radius*radius - (centerY - y) * (centerY - y)));
		radi.push_back(x);
	}

	return radi;
}

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main(int argc, const char* argv[])
{
	const int num_divisions = 16;
	const double thickness = 0.005;
	std::string mesh_extension = ".obj";
	std::string config_extension = ".cfg";


	const double height = 1;
	double bottom_elevation = 3.5 - height;

	const double step_size = 0.1;
	const double min = 0.3;
	const double max = 0.6;


	std::string base_path = "../output/autogenerated_";

	for (int i = 0; i < 10; i++) {
		std::string path = base_path + std::to_string(i);
		double base_radius = fRand(min, max);
		double center_radius = fRand(min, max);;
		double neck_radius = fRand(min, max);
		double min_radius = std::min(base_radius, std::min(center_radius, neck_radius));
		double max_radius = std::max(base_radius, std::max(center_radius, neck_radius));
		std::vector<double> radi = GenerateRadi(step_size, height, base_radius, center_radius, neck_radius);

		printf("Radi: ");
		for (int i = 0; i < radi.size(); i++) {
			printf("%f ", radi[i]);
		}
		printf("\n");

		CreateSOR(path + mesh_extension, radi, height, 0, num_divisions, thickness);
		CreateSOR(path +"_to_render_" + mesh_extension, radi, height, bottom_elevation, num_divisions, thickness);
		WriteConfig(path + config_extension, neck_radius, max_radius, height);
	}
	

	system("pause");
	return 0;
}