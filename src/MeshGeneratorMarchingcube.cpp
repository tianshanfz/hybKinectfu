/*
 * MeshGeneratorMarchingcube.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: hanyinbo
 */

#include "MeshGeneratorMarchingcube.h"
#include "utils/mesh/MeshIO.h"
#include "AppParams.h"
#include "cuda/CudaWrappers.h"
#include "cuda/CudaDeviceDataMan.h"
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/texture_mapping.h>
#include"keyframeMan.h"
MeshGeneratorMarchingcube::MeshGeneratorMarchingcube() {
	// TODO Auto-generated constructor stub

}

MeshGeneratorMarchingcube::~MeshGeneratorMarchingcube() {
	// TODO Auto-generated destructor stub
}

void MeshGeneratorMarchingcube::generateMesh()
{

	cudaMarchingcube(AppParams::instance()->_switch_params.useRGBData ,
			300*AppParams::instance()->_volume_params.fVolumeMeterSize/AppParams::instance()->_volume_params.nResolution);
}
bool MeshGeneratorMarchingcube::copyTrianglesToCPU()
{
	MarchingcubeData marchingcube_data =CudaDeviceDataMan::instance()->_marchingcube_data;
	MarchingcubeData marchingcube_data_cpu;
	marchingcube_data_cpu.init(CPU);
	marchingcube_data_cpu.copyFrom(marchingcube_data);
	int triangle_num=marchingcube_data_cpu.triangleNums();
	cout << "Marching Cubes triangles = " <<triangle_num  << std::endl;
	if (triangle_num == 0)
	{
		return false;
	}
	_meshes.m_Vertices.resize( 3 * triangle_num);
	if (AppParams::instance()->_switch_params.useRGBData == true)
	{
		_meshes.m_Colors.resize(3* triangle_num);
	}

	//float3*  vc = (float3*) triangle;
	for (unsigned int i = 0; i <  triangle_num; i++) {
		Triangle* ptriangle=marchingcube_data_cpu.triangleData()+i;
		_meshes.m_Vertices[3*i] = ptriangle->v0.pos;
		_meshes.m_Vertices[3 * i + 1] = ptriangle->v1.pos;
		_meshes.m_Vertices[3 * i + 2] = ptriangle->v2.pos;
		if (AppParams::instance()->_switch_params.useRGBData == true)
		{
			_meshes.m_Colors[3 * i] = make_float4(ptriangle->v0.color.x,ptriangle->v0.color.y,ptriangle->v0.color.z, 1.0);
			_meshes.m_Colors[3 * i + 1] = make_float4(ptriangle->v1.color.x,ptriangle->v1.color.y,ptriangle->v1.color.z, 1.0);
			_meshes.m_Colors[3 * i + 2] = make_float4(ptriangle->v2.color.x,ptriangle->v2.color.y,ptriangle->v2.color.z, 1.0);
		}
	}

	marchingcube_data_cpu.destroy();
	return true;
}
bool MeshGeneratorMarchingcube::saveMesh(const string& filename)
{
	if(false==copyTrianglesToCPU())
	{
		cout<<"copy triangles to cpu failed"<<endl;
		return false;

	}
	//create index buffer (required for merging the triangle soup)
	_meshes.m_FaceIndicesVertices.resize(_meshes.m_Vertices.size());
	for (unsigned int i = 0; i < (unsigned int)_meshes.m_Vertices.size() / 3; i++) {
		_meshes.m_FaceIndicesVertices[i][0] = 3 * i + 0;
		_meshes.m_FaceIndicesVertices[i][1] = 3 * i + 1;
		_meshes.m_FaceIndicesVertices[i][2] = 3 * i + 2;
	}
	std::cout << "vertices:\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;
	std::cout << "merging close vertices... ";
	_meshes.mergeCloseVertices(0.0001f, true);
	std::cout << "done!" << std::endl;
	std::cout << "vertices e:\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;
	std::cout << "removing duplicate faces... ";
	_meshes.removeDuplicateFaces();
	std::cout << "done!" << std::endl;
	_meshes.computeVertexNormals();
	std::cout << "vertices :\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;

//	m_meshData.applyTransform(transform);

	std::cout << "saving mesh (" << filename << ") ...";
	ml::MeshIOf::saveToFile(filename, _meshes);
	std::cout << "done!" << std::endl;
	return textureMesh(filename);
	//clearMeshBuffer();
	//return 0;
}
bool saveMTLandOBJFile (const std::string &file_name,
             const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    return false;
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      return false;
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
      tex_mesh.cloud.fields[d].name == "normal_x" ||
      tex_mesh.cloud.fields[d].name == "normal_y" ||
      tex_mesh.cloud.fields[d].name == "normal_z"))
      {
        if (!v_written)
        {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
    	return false;
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    printf("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  printf("Writting faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0)
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << std::endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
      fs << "# Faces" << std::endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1
        << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    printf ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  printf ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  printf ("Writing material files\n");
  //dont do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return true;

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return true;
}
bool MeshGeneratorMarchingcube::textureMesh(const string &filename)
{
	 pcl::PolygonMesh triangles;
	 pcl::io::loadPolygonFileOBJ(filename,triangles);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::fromPCLPointCloud2(triangles.cloud, *cloud);
	 pcl::TextureMesh mesh;
	 mesh.cloud = triangles.cloud;
	 std::vector< pcl::Vertices> polygon_1;
	 polygon_1.assign (triangles.polygons.begin(),triangles.polygons.end());
	 mesh.tex_polygons.push_back(polygon_1);
	 printf("Input mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size (),  cloud->points.size());
	 pcl::texture_mapping::CameraVector my_cams;
	 auto keyframe_list=KeyframeMan::instance()->getFrameList();
	 int i=0;
	 for(const auto& frame:keyframe_list)
	 {
		 Mat44 pose=frame.camera_pose;
		 pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		 cam.focal_length=AppParams::instance()->_rgb_camera_params.fx;
		 cam.width=AppParams::instance()->_rgb_camera_params.cols;
		 cam.height=AppParams::instance()->_rgb_camera_params.rows;
		 cam.pose(0,3)=pose.m14,cam.pose(1,3)=pose.m24,cam.pose(2,3)=pose.m34;
		 cam.pose(0,0)=pose.m11,cam.pose(0,1)=pose.m12,cam.pose(0,2)=pose.m13;
		 cam.pose(1,0)=pose.m21,cam.pose(1,1)=pose.m22,cam.pose(1,2)=pose.m23;
		 cam.pose(2,0)=pose.m31,cam.pose(2,1)=pose.m32,cam.pose(2,2)=pose.m33;
		 cam.pose(3,0) = 0.0,cam.pose(3,1) = 0.0,cam.pose (3,2) = 0.0,cam.pose (3,3) = 1.0; //Scale

		 stringstream rgb_filename;
		 rgb_filename<<"rgb"<<i<<".png";
		 rgb_filename>>cam.texture_file;
		 frame.frame_rgb.saveToFile("output/"+cam.texture_file);
		 my_cams.push_back (cam);
		 i++;
	 }
	 mesh.tex_materials.resize (my_cams.size () + 1);
	 for(int i = 0 ; i <= my_cams.size() ; ++i)
	 {
		 pcl::TexMaterial mesh_material;
		 mesh_material.tex_Ka.r = 0.2f;
		 mesh_material.tex_Ka.g = 0.2f;
		 mesh_material.tex_Ka.b = 0.2f;

		 mesh_material.tex_Kd.r = 0.8f;
		 mesh_material.tex_Kd.g = 0.8f;
		 mesh_material.tex_Kd.b = 0.8f;

		 mesh_material.tex_Ks.r = 1.0f;
		 mesh_material.tex_Ks.g = 1.0f;
		 mesh_material.tex_Ks.b = 1.0f;

		 mesh_material.tex_d = 1.0f;
		 mesh_material.tex_Ns = 75.0f;
		 mesh_material.tex_illum = 2;

		 std::stringstream tex_name;
		 tex_name << "material_" << i;
 		 tex_name >> mesh_material.tex_name;
		 if(i < my_cams.size ())
		   mesh_material.tex_file = my_cams[i].texture_file;
		 else
			 mesh_material.tex_file = "occluded.jpg";
		 mesh.tex_materials[i] = mesh_material;
	 }
	 pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	 tm.textureMeshwithMultipleCameras(mesh, my_cams);
	 for(int i = 0 ; i <=my_cams.size() ; ++i)
	  {
	    printf("Sub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
	  }
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud);
	  n.setInputCloud (cloud);
	  n.setSearchMethod (tree);
	  n.setKSearch (20);
	  n.compute (*normals);

	  // Concatenate XYZ and normal fields
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	  pcl::toPCLPointCloud2 (*cloud_with_normals, mesh.cloud);

	  return saveMTLandOBJFile("output/textured_mesh.obj", mesh, 5);
}
