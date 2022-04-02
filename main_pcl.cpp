// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

#include<iostream>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

typedef ::PointCloud PointCloud_Ransac;


void outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int search_num = 50)
{
	//创建滤波器对象
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_src);
	sor.setMeanK(search_num);//寻找每个点的50个最近邻点
	sor.setStddevMulThresh(3.0);//一个点的最近邻距离超过全局平均距离的一个标准差以上，就会舍弃
	sor.filter(*cloud_filtered);
    std::cout << "cloud filterd: " << cloud_filtered->size() << std::endl;
}

void radius_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	//创建滤波器对象
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_src);
	outrem.setRadiusSearch(2);  //设置半径为0.8的范围内找临近点
    outrem.setMinNeighborsInRadius(3);  //设置查询点的邻域点集数小于5的删除
	outrem.filter (*cloud_filtered); 
    std::cout << "cloud filterd: " << cloud_filtered->size() << std::endl;
}


int main()
{
    std::string inputFileName("/home/fan/code/3d/data/middel_150_1.pcd");
    std::string outputFileName("/home/fan/code/3d/data/middel_150_1");

    std::cout << "Reading the point cloud..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(inputFileName, *cloud_input);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	// voxel_filter(cloud_src, cloud_filtered);
	outlier_filter(cloud_input, cloud_src);

	PointCloud pc;
    int pointNumCur = cloud_src->size();
    float xyzXYZ[6] = {10000, 10000, 10000, -10000, -10000, -10000};
    for ( auto& pt : cloud_src->points ) {
        pc.push_back(Point(Vec3f(pt.x, pt.y, pt.z)));
        xyzXYZ[0] = std::min(xyzXYZ[0], pt.x);
        xyzXYZ[1] = std::min(xyzXYZ[1], pt.y);
        xyzXYZ[2] = std::min(xyzXYZ[2], pt.z);
        xyzXYZ[3] = std::max(xyzXYZ[0], pt.x);
        xyzXYZ[4] = std::max(xyzXYZ[1], pt.y);
        xyzXYZ[5] = std::max(xyzXYZ[2], pt.z);
    }
	
	// set the bbox in pc
	pc.setBBox(Vec3f(xyzXYZ[0],xyzXYZ[1],xyzXYZ[2]), Vec3f(xyzXYZ[3],xyzXYZ[4],xyzXYZ[5]));
	//void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	pc.calcNormals(3);


	std::cout << "added " << pc.size() << " points" << std::endl;

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .005f * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
		// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .8f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 1000; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
	// detector.Add(new CylinderPrimitiveShapeConstructor());
	
	/*
	detector.Add(new SpherePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	detector.Add(new ConePrimitiveShapeConstructor());
	detector.Add(new TorusPrimitiveShapeConstructor());
	*/

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
		// returns number of unassigned points
		// the array shapes is filled with pointers to the detected shapes
		// the second element per shapes gives the number of points assigned to that primitive (the support)
		// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
		// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
		// the points of shape i are found in the range
		// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	std::cout << "remaining unassigned points " << remaining << std::endl;
	for(int i=0;i<shapes.size();i++)
	{
		std::string desc;
		shapes[i].first->Description(&desc);
		std::cout << "shape " << i << " consists of " << shapes[i].second << " points, it is a " << desc << std::endl;
	}


    PointCloud_Ransac::reverse_iterator start = pc.rbegin();
    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, std::size_t> >::const_iterator shape_itr = shapes.begin();

    // auto primitive_types = cloud->vertex_property<int>("v:primitive_type", PrimitivesRansac::UNKNOWN);
    // auto primitive_indices = cloud->vertex_property<int>("v:primitive_index", -1);
    // primitive_types.vector().assign(cloud->n_vertices(), PrimitivesRansac::UNKNOWN);
    // primitive_indices.vector().assign(cloud->n_vertices(), -1);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_planes;
    auto& src_pts = cloud_src->points;
    int index = 0;
    PointCloud_Ransac::reverse_iterator point_itr = start;
    for (; shape_itr != shapes.end(); ++shape_itr) {
        const PrimitiveShape *primitive = shape_itr->first;
        std::size_t num = shape_itr->second;

        // std::list<int> vts;
        // PointCloud_Ransac::reverse_iterator point_itr = start;
        // for (std::size_t count = 0; count < num; ++count) {
        //     int v = 0; //int(point_itr->index);
        //     vts.push_back(v);
        //     ++point_itr;
        // }
        // start = point_itr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>);
        plane_points->points.reserve(num);
        for (std::size_t count = 0; count < num; ++count) {
            plane_points->points.emplace_back(point_itr->pos[0], point_itr->pos[1], point_itr->pos[2]);
            ++point_itr;
        }
        pcl_planes.push_back(plane_points);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("boundary"));

    int v1(0);
    MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
    MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
    MView->addPointCloud<pcl::PointXYZ>(cloud_src, "sample cloud", v1);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);

	double colors[][3] = {{0, 0.5, 0}, {0, 0, 0.5}, {0.5, 0, 0}, {0.5, 0.5, 0}, {0.5, 0, 0.5}, {0, 0.5, 0.5},
                          {0.2, 0.5, 0.7}, {0.7, 0.5, 0.2}, {0.5, 0.2, 0.7}, {0.5, 0.7, 0.2}, {0.2, 0.7, 0.5}, {0.7, 0.2, 0.5}};
    int v2(1);
    MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
    MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
    MView->addText("Boudary point clouds", 80, 80, "v2_text", v2);

	for (int i = 0; i < pcl_planes.size(); ++i) {
		std::string plane_str = std::string("plane_") + std::to_string(i);
		std::string hull_str = std::string("hull_") + std::to_string(i);
		MView->addPointCloud<pcl::PointXYZ>(pcl_planes[i], plane_str, v2);
		MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[i][0], colors[i][1], colors[i][2], plane_str, v2);
		// MView->addPointCloud<pcl::PointXYZ>(hulls[i], hull_str, v2);
		// MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 1, hull_str, v2);
		// MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, hull_str, v2);
	}

    MView->spin();

}
