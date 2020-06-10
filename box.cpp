
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
 #include <algorithm>
#include <math.h>
#include <vector>
#include<ctime>
#include <Eigen/Dense>  
using namespace Eigen;
using namespace std;
typedef pcl::PointXYZ PointType;
#define pi 3.1415926
 float DirecAngle(float y,float x)
{
   if(y<0&&x>0) return atan(y/x)+2*pi;
   if(y>0&&x>0) return atan(y/x);
   return atan(y/x)+pi;
}
Vector3f rotationMatrixToEulerAngles(MatrixXf &R)
{
     
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0));
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;	
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Vector3f(x, y, z);
             
}
int main(int argc, char **argv)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
 	pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>());
	 pcl::PointCloud<PointType>::Ptr cloud1_origin(new pcl::PointCloud<PointType>());
	pcl::io::loadPCDFile(argv[1], *cloud);
 	pcl::io::loadPCDFile(argv[2], *cloud1);
	 pcl::io::loadPCDFile(argv[2], *cloud1_origin);
	clock_t start,end;
    start = clock();

	//**********RANSAC提取地面计算正视滚转角
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置阈值
	seg.setDistanceThreshold(1.5);
	//输入点云
	seg.setInputCloud(cloud);
	//分割点云，获得平面和法向量
	seg.segment(*inliers, *coefficients);
cout<<coefficients->values[0]<<endl<<coefficients->values[1]<<endl<<coefficients->values[2]<<endl<<endl;
	//×××××

	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);

	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg1;

	seg1.setOptimizeCoefficients(true);

	seg1.setModelType(pcl::SACMODEL_PLANE);

	seg1.setMethodType(pcl::SAC_RANSAC);

	seg1.setDistanceThreshold(1.5);

	seg1.setInputCloud(cloud1);

	seg1.segment(*inliers1, *coefficients1);
cout<<coefficients1->values[0]<<endl<<coefficients1->values[1]<<endl<<coefficients1->values[2]<<endl;

	float new_ry, new_ry1, new_rx, new_rx1;
	float yaw, pitch, roll;
	Eigen::Matrix4f RZ = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f RY = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f RX = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f Ra = Eigen::Matrix4f::Identity();
	////////////////////////////////////
	new_ry = DirecAngle(coefficients->values[2], coefficients->values[0]);
	new_rx = DirecAngle(coefficients->values[2], coefficients->values[1]);
				//正视旋转只做一次
	new_ry1 = DirecAngle(coefficients1->values[2], coefficients1->values[0]);

	// if (new_ry1 > new_ry)
	// 	pitch = new_ry1 - new_ry;
	// else
	// 	pitch = 2 * pi - (new_ry - new_ry1);
		
	pitch = new_ry1 - new_ry;
	if (abs(new_ry1 - new_ry) < 0.15)
		pitch = 0;
//cout<<"pitch = "<<pitch<<endl;
			//		pitch=new_ry1-new_ry;
	RY(0, 0) = cos(pitch);
	RY(0, 2) = sin(pitch);
	RY(2, 0) = -sin(pitch);
	RY(2, 2) = cos(pitch);

	Ra = RY * Ra;

	pcl::transformPointCloud(*cloud1, *cloud1, RY);
			//侧视旋转只做一次
	seg1.setInputCloud(cloud1);

	seg1.segment(*inliers1, *coefficients1);

	new_rx1 = DirecAngle(coefficients1->values[2], coefficients1->values[1]);

			// if(new_rx>new_rx1) roll=new_rx-new_rx1;
			// else roll=2*pi-(new_rx1-new_rx);

	if (abs(new_rx1 - new_rx) < 0.15)
		roll = 0;

	roll = new_rx - new_rx1;
//cout<<"roll = "<<roll<<endl;
	RX(1, 1) = cos(roll);
	RX(1, 2) = -sin(roll);
	RX(2, 1) = sin(roll);
	RX(2, 2) = cos(roll);

	Ra = RX * Ra;

	pcl::transformPointCloud(*cloud1, *cloud1, RX);

///////PCA计算俯视角部分
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
 	Eigen::Vector4f pcaCentroid1;
	pcl::compute3DCentroid(*cloud1, pcaCentroid1);
	Eigen::Matrix3f covariance1;
	pcl::computeCovarianceMatrixNormalized(*cloud1, pcaCentroid1, covariance1);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver1(covariance1, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA1 = eigen_solver1.eigenvectors();
	Eigen::Vector3f eigenValuesPCA1 = eigen_solver1.eigenvalues();
	eigenVectorsPCA1.col(2) = eigenVectorsPCA1.col(0).cross(eigenVectorsPCA1.col(1)); //校正主方向间垂直
	eigenVectorsPCA1.col(0) = eigenVectorsPCA1.col(1).cross(eigenVectorsPCA1.col(2));
	eigenVectorsPCA1.col(1) = eigenVectorsPCA1.col(2).cross(eigenVectorsPCA1.col(0));
	// std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	// std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	// std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
	/*
	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
 
	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
 
 
	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
 
	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
 
	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT1(c1);
 
	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);
 
//第二组点云

 	Eigen::Matrix4f tm1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv1 = Eigen::Matrix4f::Identity();
	tm1.block<3, 3>(0, 0) = eigenVectorsPCA1.transpose();   //R.
	tm1.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA1.transpose()) *(pcaCentroid1.head<3>());//  -R*t
	tm_inv1 = tm1.inverse();
 
	pcl::PointCloud<PointType>::Ptr transformedCloud1(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud1, *transformedCloud1, tm1);
 
	PointType min_p2, max_p2;
	Eigen::Vector3f c3, c2;
	pcl::getMinMax3D(*transformedCloud1, min_p2, max_p2);
	c3 = 0.5f*(min_p2.getVector3fMap() + max_p2.getVector3fMap());
 
 
	Eigen::Affine3f tm_inv_aff1(tm_inv1);
	pcl::transformPoint(c3, c2, tm_inv_aff1);
 
	Eigen::Vector3f whd2, whd3;
	whd3 = max_p2.getVector3fMap() - min_p2.getVector3fMap();
	whd2 = whd3;
	float sc3 = (whd3(0) + whd3(1) + whd3(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
 
	const Eigen::Quaternionf bboxQ3(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT3(c3);
 
	const Eigen::Quaternionf bboxQ2(tm_inv1.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT2(c2);
 
	//变换到原点的点云主方向
	// PointType op;
	// op.x = 0.0;
	// op.y = 0.0;
	// op.z = 0.0;
	// Eigen::Vector3f px, py, pz;
	// Eigen::Affine3f tm_aff(tm);
	// pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	// pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	// pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	// PointType pcaX;
	// pcaX.x = sc1 * px(0);
	// pcaX.y = sc1 * px(1);
	// pcaX.z = sc1 * px(2);
	// PointType pcaY;
	// pcaY.x = sc1 * py(0);
	// pcaY.y = sc1 * py(1);
	// pcaY.z = sc1 * py(2);
	// PointType pcaZ;
	// pcaZ.x = sc1 * pz(0);
	// pcaZ.y = sc1 * pz(1);
	// pcaZ.z = sc1 * pz(2);
 
 
	//初始点云的主方向
	PointType cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	PointType pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	PointType pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	PointType pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;
 
 	//第二组初始点云的主方向
	PointType cp1;
	cp1.x = pcaCentroid1(0);
	cp1.y = pcaCentroid1(1);
	cp1.z = pcaCentroid1(2);
	PointType pcX1;
	pcX1.x = sc3 * eigenVectorsPCA1(0, 0) + cp1.x;
	pcX1.y = sc3 * eigenVectorsPCA1(1, 0) + cp1.y;
	pcX1.z = sc3 * eigenVectorsPCA1(2, 0) + cp1.z;
	PointType pcY1;
	pcY1.x = sc3 * eigenVectorsPCA1(0, 1) + cp1.x;
	pcY1.y = sc3 * eigenVectorsPCA1(1, 1) + cp1.y;
	pcY1.z = sc3 * eigenVectorsPCA1(2, 1) + cp1.z;
	PointType pcZ1;
	pcZ1.x = sc3 * eigenVectorsPCA1(0, 2) + cp1.x;
	pcZ1.y = sc3 * eigenVectorsPCA1(1, 2) + cp1.y;
	pcZ1.z = sc3 * eigenVectorsPCA1(2, 2) + cp1.z;
//将雷达的45度角旋转回去
// 	Matrix3f Rc;
// 	Vector3f vx,vy,vz;
// 	vx(0)=eigenVectorsPCA1(0, 0);
// 	vx(1)=eigenVectorsPCA1(1, 0);
// 	vx(2)=eigenVectorsPCA1(2, 0);
// 	vy(0)=eigenVectorsPCA1(0, 1);
// 	vy(1)=eigenVectorsPCA1(1, 1);
// 	vy(2)=eigenVectorsPCA1(2, 1);
// 	vz(0)=eigenVectorsPCA1(0, 2);
// 	vz(1)=eigenVectorsPCA1(1, 2);
// 	vz(2)=eigenVectorsPCA1(2, 2);
//        Rc<< 1,         0,         0,
//         0 , 0.707107,  -0.707107,
//         0 ,0.707107,  0.707107;
// 	Vector3f vx1=vx.transpose()*Rc;
// 	Vector3f vy1=vy.transpose()*Rc;
// 	Vector3f vz1=vz.transpose()*Rc;
// pcX1.x=sc3 *vx1(0)+cp1.x;
// pcX1.y=sc3 *vx1(1)+cp1.y;
// pcX1.z=sc3 *vx1(2)+cp1.z;
// pcY1.x=sc3 *vy1(0)+cp1.x;
// pcY1.y=sc3 *vy1(1)+cp1.y;
// pcY1.z=sc3 *vy1(2)+cp1.z;
// pcZ1.x=sc3 *vz1(0)+cp1.x;
// pcZ1.y=sc3 *vz1(1)+cp1.y;
// pcZ1.z=sc3 *vz1(2)+cp1.z;
	// float yaw = pi /16;
	// float pitch = pi /4;
	// float roll = pi /4;

	float yaw1=atan(eigenVectorsPCA1(0, 0)/eigenVectorsPCA1(1, 0));
	float yaw2=atan(eigenVectorsPCA1(0, 1)/eigenVectorsPCA1(1, 1));
	 yaw=0.5*(yaw1+yaw2);

	// float pitch1=atan(eigenVectorsPCA1(2, 1)/eigenVectorsPCA1(1, 1));
	// float pitch2=atan(eigenVectorsPCA1(2, 2)/eigenVectorsPCA1(1, 2));
	// float pitch=0.5*(pitch1+pitch2);

	// float roll1=atan(eigenVectorsPCA1(2, 0)/eigenVectorsPCA1(0, 0));
	// float roll2=atan(eigenVectorsPCA1(2, 2)/eigenVectorsPCA1(0, 2));
	// float roll=0.5*(roll1+roll2);

	// float temp=pitch;
	// pitch=roll;
	// roll=temp;

	RZ(0,0)=cos(yaw);
	RZ(0,1)=-sin(yaw);
	RZ(1,0)=sin(yaw);
	RZ(1,1)=cos(yaw);

	Ra = RZ * Ra;
	// Eigen::Matrix4f transform_1;
	// transform_1 =RZ*RY*RX;
	
	// transform_1(0,3)=0;
	// transform_1(1,3)=0;
	// transform_1(2,3)=0;
	// //Eigen::Matrix3f R= transform_1.block<3,3>(0,0);

	// cout<<R<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudt(new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::transformPointCloud(*cloud1, *cloudt, RZ);

//误差计算
	Matrix3f Re = Ra.block<3,3>(0,0);

	Matrix3f Rt;

// 	Rt<<      0.707107 ,-0.707107    ,     0,
//  0.707107  ,0.707107   ,      0,
//         0 ,        0   ,      1;

//  Rt<< 0.92388, -0.382683         ,0,
//  0.382683  , 0.92388     ,    0,
//         0  ,       0     ,    1;

    Rt<<  0.5 ,-0.146447,  0.853553,
      0.5,  0.853553, -0.146447,
-0.707107,       0.5 ,      0.5;

	Matrix3f Rt1=Rt.inverse();
	MatrixXf R=Rt1.inverse()*Re;
	Vector3f v=rotationMatrixToEulerAngles(R);
	float Er=abs(v(0))+abs(v(1))+abs(v(2));

// 	Vector3f Tt(-40,-40,-40);
// 	Vector3f Te=icp.getFinalTransformation().block<3,1>(0,3);
// 	float Et=sqrt((Tt(0)-Te(0))*(Tt(0)-Te(0))+(Tt(1)-Te(1))*(Tt(1)-Te(1))+(Tt(2)-Te(2))*(Tt(2)-Te(2)));
	
// 	cout<<endl<<Rt1<<endl<<endl<<Re<<endl<<endl<<Tt<<endl<<endl<<Te<<endl<<endl;
	cout<<"Er:"<<endl<<Er<<endl;
//计时
	
 
    end = clock();
    double dur = (double)(end - start);
    cout<<"Use Time:"<<dur/CLOCKS_PER_SEC<<" s"<<endl;
	//visualization
	pcl::visualization::PCLVisualizer viewer;
     viewer.setCameraFieldOfView(0.785398);//fov 45°  视场角
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0);
	// pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); //转换到原点的点云相关
	// viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	// viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
 
	// viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	// viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	// viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");
 	int v1 = 1;//创建双窗口
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPortCamera(v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0);  //输入的初始点云相关
	viewer.addPointCloud(cloud, color_handler, "cloud",v1);
	viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
 
	viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

 	pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler1(cloud1_origin, 0, 255, 0);  //输入的初始点云相关
	viewer.addPointCloud(cloud1_origin, color_handler1, "cloud1",v1);
	viewer.addCube(bboxT2, bboxQ2, whd2(0), whd2(1), whd2(2), "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
 
	viewer.addArrow(pcX1, cp1, 1.0, 0.0, 0.0, false, "arrow_X");
	viewer.addArrow(pcY1, cp1, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcZ1, cp1, 0.0, 0.0, 1.0, false, "arrow_Z");
//第二窗口显示配准结果
    int v2 = 1;
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.createViewPortCamera(v2);
    viewer.setCameraFieldOfView(0.785398,v2);//fov 45°  视场角
    viewer.setBackgroundColor(0.0, 0.2, 1.0,v2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0,v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler1(cloud, 255, 0, 00);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> FinalcloudHandler1(cloudt, 0, 255, 0);

    viewer.addPointCloud(cloud, outcloudHandler1, "Out22",v2);
    viewer.addPointCloud(cloudt, FinalcloudHandler1, "Final22",v2);


	// viewer.addCoordinateSystem(0.5f*sc1);
	// viewer.setBackgroundColor(1.0, 1.0, 1.0);

	// cout<<pcX<<endl<<pcY<<endl<<pcZ<<endl;
	// cout<<pcX1<<endl<<pcY1<<endl<<pcZ1<<endl;
	//	cout<<eigenVectorsPCA<<endl<<endl<<eigenVectorsPCA1<<endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
 
	return 0;
}
