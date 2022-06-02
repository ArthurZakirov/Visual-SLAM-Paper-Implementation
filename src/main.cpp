#include <iostream>
#include <fstream>
#include "math.h"
#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "Volume.h"

#define USE_POINT_TO_PLANE 1 // 1-point to plane, 0-point to point (=procrustes)
#define USE_LINEAR_ICP 0	 // 1-linearize and solve ATAx=ATb, 0-iterative solver (gauss-newton)

#define RUN_SHAPE_ICP 0	   // 1-bunny file, 0-not bunny file
#define RUN_SEQUENCE_ICP 1 // 1-room file, 0-not room file

class VoxelBounds
{
private:
	float m_x_min;
	float m_x_max;
	float m_y_min;
	float m_y_max;
	float m_z_min;
	float m_z_max;

public:
	VoxelBounds(VirtualSensor sensor)
	{
		float *depthmap = sensor.getDepth();
		unsigned int height = sensor.getDepthImageHeight();
		unsigned int width = sensor.getDepthImageWidth();
		Matrix3f intrinsic_inv = sensor.getDepthIntrinsics().inverse();
		Matrix4f ir_camera2rgb_camera = sensor.getDepthExtrinsics().inverse();
		Matrix4f camera2world = sensor.getTrajectory().inverse();

		Vector3f left_upper = depthmap2world(0, 0, depthmap[0], intrinsic_inv, ir_camera2rgb_camera, camera2world);
		Vector3f right_lower = depthmap2world(height, width, depthmap[height * width], intrinsic_inv, ir_camera2rgb_camera, camera2world);
		m_x_min = left_upper.x();
		m_x_max = right_lower.x();
		m_y_min = left_upper.y();
		m_y_max = right_lower.y();
		m_z_max = *std::max_element(depthmap, depthmap + sizeof(depthmap) / sizeof(float));
		m_z_min = *std::min_element(depthmap, depthmap + sizeof(depthmap) / sizeof(float));
	}

	float getMinX()
	{
		return m_x_min;
	}
	float getMaxX()
	{
		return m_x_max;
	}
	float getMinY()
	{
		return m_y_min;
	}
	float getMaxY()
	{
		return m_y_max;
	}
	float getMinZ()
	{
		return m_z_min;
	}
	float getMaxZ()
	{
		return m_z_max;
	}
};

ICPOptimizer *init_optimizer()
{
	ICPOptimizer *optimizer = nullptr;
	if (USE_LINEAR_ICP)
	{
		optimizer = new LinearICPOptimizer();
	}
	else
	{
		optimizer = new CeresICPOptimizer();
	}

	optimizer->setMatchingMaxDistance(0.1f);
	if (USE_POINT_TO_PLANE)
	{
		optimizer->usePointToPlaneConstraints(true);
		optimizer->setNbOfIterations(5);
	}
	else
	{
		optimizer->usePointToPlaneConstraints(false);
		optimizer->setNbOfIterations(10);
	}
	return optimizer;
}

Vector3f depthmap2world(int x, int y, int z, Matrix3f intrinsic, Matrix4f ir_camera2rgb_camera, Matrix4f camera2world)
{
	Vector3f p_image = Vector3f(x, y, 1) * z;
	Vector4f p_camera;
	p_camera << intrinsic * p_image, 1;
	Vector4f p_world = camera2world * ir_camera2rgb_camera * p_camera;
	return p_world.head(3);
}

int reconstructRoom()
{
	std::string filenameIn = std::string("../../Data/rgbd_dataset_freiburg1_xyz/");
	std::string filenameBaseOut = std::string("mesh_");

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	PointCloud target{
		sensor.getDepth(),
		sensor.getDepthIntrinsics(),
		sensor.getDepthExtrinsics(),
		sensor.getDepthImageWidth(),
		sensor.getDepthImageHeight()};

	// Setup the optimizer.
	ICPOptimizer *optimizer = init_optimizer();

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());

	int i = 0;
	const int iMax = 50;
	while (sensor.processNextFrame() && i <= iMax)
	{
		float *depthMap = sensor.getDepth();
		Matrix3f ir_camera2depthmap = sensor.getDepthIntrinsics();
		Matrix4f rgb_camera2ir_camera = sensor.getDepthExtrinsics();
		Matrix4f world2rgb_camera = sensor.getTrajectory();

		sensor.getTrajectory();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{
			sensor.getDepth(),
			sensor.getDepthIntrinsics(),
			sensor.getDepthExtrinsics(),
			sensor.getDepthImageWidth(),
			sensor.getDepthImageHeight(),
			8};

		// convert sensor data to SDF
		// general parameters

		VoxelBounds voxelBounds = VoxelBounds(sensor);

		// Vector3f min_point(voxelBounds.getMinX(), voxelBounds.getMinY(), voxelBounds.getMinZ());
		// Vector3f max_point(voxelBounds.getMaxX(), voxelBounds.getMaxY(), voxelBounds.getMaxX());
		// unsigned int resolution = 50;
		//
		// DiscreteSurface discreteSurface(min_point, max_point, resolution);

		// std::vector<float> distances;

		// for (int x = voxelBounds.getMinX(); x < voxelBounds.getMaxX(); x++) {
		//	for (int y = voxelBounds.getMinY(); y < voxelBounds.getMaxY(); y++) {
		//		for (int z = voxelBounds.getMinZ(); z < voxelBounds.getMaxX(); z++) {
		//
		//			VectorXd p;
		//			p << discreteSurface.get_world_position(x, y, z), 1;
		//			p = ir_camera2depthmap * (rgb_camera2ir_camera * world2rgb_camera * p).head(3);
		//			VectorXi pi = p.template cast<int>();

		//			if (pi.x() > 0 && pi.y() > 0 && pi.z() > 0
		//				&& pi.x() < sensor.getDepthImageWidth()
		//				&& pi.y() < sensor.getDepthImageHeight()) {

		//				float d = depthMap[x + y * sensor.getDepthImageWidth()];
		//				float sdf = d - pi.z();

		//				discreteSurface(x, y, z)->sdf = sdf;
		//				discreteSurface(x, y, z)->weight = 1;
		//
		//			}
		//			// compute distance
		//			// distances.push_back(0);

		//		}
		//	}
		//}
		optimizer->estimatePose(source, target, currentCameraToWorld);

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		if (i % 20 == 0)
		{
			// We write out the mesh to file for debugging.
			SimpleMesh currentDepthMesh{sensor, currentCameraPose, 0.1f};
			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

			std::stringstream ss;
			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
			std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
			if (!resultingMesh.writeMesh(ss.str()))
			{
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			}
		}

		i++;
	}

	delete optimizer;

	return 0;
}

int main()
{
	int result = 0;
	if (RUN_SEQUENCE_ICP)
		result += reconstructRoom();
	return result;
}
