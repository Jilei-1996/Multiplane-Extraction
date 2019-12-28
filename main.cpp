#include <iostream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <unistd.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>

using namespace std;

/*int fileNameFilter(const struct dirent *cur)
{
    std::string str(cur->d_name);
    if (str.find(".pcd") != std::string::npos)
    {
        return 1;
    }
    return 0;
}*/


 
int main()
{
  // vector<string>name_list;
   ofstream mycout("20191218_2_1_result.txt");
   /*******************************************************顺序读取文件夹中的文档****************************************************************/
  
  
   /*struct dirent **namelist;
   int n = scandir("/home/jilei/data_sum/out2", &namelist, fileNameFilter, alphasort);
   //int n = scandir("/home/jilei/pcd", &namelist, fileNameFilter, alphasort);
   vector<string> name_list(n);
   if(n < 0)
            cerr << "memery error " << endl;
   else 
            cout << "total number is: " << n << endl;
            
   for( int i = 0; i < n; i++)
   {
            // /* skip . && .. */
            // if(namelist[i]->d_name[0] == '.')
            //     continue;
            /*name_list.push_back(namelist[i]->d_name);
	    //vector<string> instr;
	    //name_list.push_back(instr);//instr+namelist[i]->d_name);
            //cout<<namelist[i]->d_name<<endl;
            free(namelist[i]);
    }
    free(namelist);
    //delete name_list;
    
    cout << "the number of pcd doc is"<< " "<< n << endl;
    
    cout << name_list[0] << endl;*/
  /*******************************************************读入数据并进行处理*******************************************************************/
   ifstream in("/home/jilei/projects/getdoc/build/result.txt");
   string name;
   while(in >> name )
   {
    cout << name << endl;
     
 
    string filename=name;
    //string filename="/home/jilei/pcd/1573716329.127573000";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/jilei/data_sum/out2/"+filename, *cloud_ptr);
    cout << "PCDReader方式读取点个数: " << cloud_ptr->points.size() << endl;
    

    //pcl::PLYReader reader;
    //pcl::PCDReader reader;
    //reader.read("/home/jilei/pcd/1573716329.127573000.pcd", *cloud_ptr);
 
    //reader.read(filename+".pcd", *cloud_ptr);
    //reader.read(filename+".ply", *cloud_ptr);
    if(cloud_ptr==NULL)
    {
        cout<<"ply/pcd file read error"<<endl;
        return -1;
    }
    /***************************************使用区域生长法分割平面****************************************************************/
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//法线变量
    //pcl::PointCloud<pcl::Normal>::Ptr planeout (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTree初始化
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne; //法线估计
    pcl::PointCloud<pcl::PointXYZ>::Ptr outgr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr ingr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    /*求法线*/
    ne.setInputCloud (cloud_ptr);
    ne.setSearchMethod (tree);
    ne.setKSearch (10);//init = 15
    ne.setViewPoint(std::numeric_limits<double>::max (),std::numeric_limits<double>::max (), std::numeric_limits<double>::max ());
    ne.compute (*cloud_normals);
    //cout<<"cloud_normals size is "<< cloud_normals->size()<<endl;

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (100);  //500
    reg.setMaxClusterSize (10000); //10000
    reg.setSearchMethod (tree);   
    reg.setNumberOfNeighbours (30);  //ini =30
    reg.setInputCloud (cloud_ptr);        
    //reg.setIndices (indices);
    reg.setInputNormals (cloud_normals);    
    reg.setSmoothnessThreshold (3 / 180.0 * M_PI);  //ini =3.
    reg.setCurvatureThreshold (1.0); //1.0

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    int n = clusters.size ();
    for(int j = 0; j < n; j ++)
    {
      cout << j+1 << "'s cluster has " << clusters[j].indices.size () << " points." << clusters[j].indices[0] << endl;
    }
    

   /* cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    cout << "These are the indices of the points of the initial" <<
    endl << "cloud that belong to the first cluster:" << std::endl;*/
    
    /****************************************************用于将每簇点云保存的代码*****************************************************************/
   
    int currentClusterNum = 1;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        // ...add all its points to a new cloud...
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(cloud_ptr->points[*point]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
	/**********************************************RANSAC提取平面**************************************************************/
	
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers表示误差能容忍的点 记录的是点云的序号
      pcl::SACSegmentation<pcl::PointXYZ> seg;// 创建一个分割器
 
      seg.setOptimizeCoefficients (true);// Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
      
      seg.setModelType (pcl::SACMODEL_PLANE);// Mandatory-设置目标几何形状
      
      seg.setMethodType (pcl::SAC_RANSAC);//分割方法：随机采样法
      
      seg.setDistanceThreshold (0.03);//设置误差容忍范围，也就是我说过的阈值
     
      seg.setInputCloud (cluster);//输入点云
      
      seg.segment (*inliers, *coefficients);//分割点云
      
      if (inliers->indices.size () == 0)
	{
	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	    return (-1);
	}
	//打印出平面模型
	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
 
	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	mycout << filename.substr(0,filename.length()-4) << " " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " 
              << coefficients->values[3] << " " << inliers->indices.size ()<< std::endl;
	/*以下显示每簇点云中每个点的数据*/
	
        /*for (size_t i = 0; i < inliers->indices.size (); ++i)  
          std::cerr << inliers->indices[i] << "   " << cluster->points[inliers->indices[i]].x << " "
                                                 << cluster->points[inliers->indices[i]].y << " "
                                                << cluster->points[inliers->indices[i]].z << std::endl;*/
    /********************************************save plane points after RANSAC**********************************************************/
	
        pcl::PointCloud<pcl::PointXYZ>::Ptr planeafterransac(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < inliers->indices.size (); ++i)
            planeafterransac->points.push_back(cluster->points[inliers->indices[i]]);
        planeafterransac->width = cluster->points.size();
        planeafterransac->height = 1;
        planeafterransac->is_dense = true;
    
    /**********************************************显示**************************************************************/
	
	/*pcl::visualization::CloudViewer viewer ("Cluster viewer");
    
        //viewer.showCloud(cloud_ptr);
        viewer.showCloud(planeafterransac);
        while (!viewer.wasStopped ())
        {
       }*/
   /**********************************************输出**************************************************************/
        // ...and save it to disk.
        if (cluster->points.size() <= 0)
            break;
        //std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
       // std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
       // pcl::io::savePCDFileASCII(fileName, *cluster);
        currentClusterNum++;
	
	
    }
    
    
   /*******************************************************************************************************************************************/
   /* pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    
    //viewer.showCloud(cloud_ptr);
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }
    
    
    cout<<"all is well" << endl;
    pause();*/
  }
    mycout.close();
 
    return 0;
}
