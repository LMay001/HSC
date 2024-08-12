#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>
#include <iterator>
#include <fstream>
#include <cstdint>

// yaml
#include <yaml-cpp/yaml.h>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// KD tree and time
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using HSCPointType = pcl::PointXYZ;
using CloudType = pcl::PointCloud<HSCPointType>::Ptr; // point cloud type
using KeyMat = std::vector<std::vector<float> >;  // vector for key
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;  // KDTree

// Auxiliary function
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class HSCManager
{
public: 
    HSCManager(const std::string& config_path){
        frame_index = 0;  // init index
        cloud_in = CloudType(new pcl::PointCloud<HSCPointType>);  // init input point cloud
        GetConfigFromYAML(config_path);
    }

    // Tool function
    void GetConfigFromYAML(const std::string& config_path);
    void PrintParameter();
    void GetCloudFromBIN(const std::string& filename);
    void GetPointCloudFromNCLT(const std::string& filename);
    void ClearValue();

    // hierarchical algorithm
    void UpdateLevel( pcl::PointCloud<HSCPointType> & _scan_down );
    int CalculateLevel(const double& curr_pt_z);
    double GetDownHeight(int layer);
    double GetUpHeight(int layer);

    // Descriptor
    Eigen::MatrixXd HSC_Generate( pcl::PointCloud<HSCPointType> & _scan_down );
    Eigen::MatrixXi Mask_HSC( const Eigen::MatrixXd &_desc );
    Eigen::MatrixXd Multiple2Single_mask( const Eigen::MatrixXd &_desc, const Eigen::MatrixXi &_mask );
    Eigen::MatrixXd Ringkey_Generate( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd Sectorkey_Generate( Eigen::MatrixXd &_desc );
    
    // LCD
    std::pair<double, int> DistanceBtnDesc( MatrixXd &_sc1, MatrixXd &_sc2 );
    int FastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 );
    double Corr_Caculata ( MatrixXd &_sc1, MatrixXd &_sc2 );
    
    // Time series enhancement
    void TimeSeriesEnhance();
    void BeforeEnhance();
    void CandidateEnhance( std::vector<size_t>& candidate_final);
    std::pair<int, double> ReGetResult( int current_index );
    void ProportionMapping(double& input);

    // API
    void Descriptor_Generate( pcl::PointCloud<HSCPointType> & _scan_down );
    void LCD( void );

public:
    int frame_index;
    CloudType cloud_in;
    
    // data storage
    std::vector<Eigen::MatrixXd> des_dataset;
    std::vector<Eigen::MatrixXd> des_multi_dataset;
    std::vector<Eigen::MatrixXd> des_single_dataset;
    KeyMat ringkey_vector;

    // result
    std::vector<int> LCD_index_dataset;
    std::vector<double> LCD_corr_dataset;

    // KDTree
    KeyMat ringkey_kdtree;
    std::unique_ptr<InvKeyTree> search_kdtree;

public:
    // parameter
    std::string Sequence = "KITTI05";

    std::string Raw_Path = "/home/jlurobot/data/raw/";
    std::string Result_Path = "/home/jlurobot/HSC/result/";

    double LIDAR_HEIGHT = 2.0;
    int    RING_NUM = 20;
    int    SECTOR_NUM = 60;
    double MAX_RADIUS = 80.0;
    int    LEVEL = 4;
    double LEVEL_SKIP = 0.0;
    double MAX_HEIGHT = 5.0;
    double Edge_th = 0.95;
    bool   Use_TimeSeries = false;
    double TimeStart_th = 0.3;
    double TimeEnd_th = 0.4;
    int    last_TS_endindex = 0;
    int    out_num = 0;
    bool   current_out = false;

    bool   TS_flag = false;
    bool   Candidate_flag = false;
    bool   Before_flag = false;
    bool   After_flag = false;

    int    Ignore_Num_Before_Current = 50;
    int    Candidate_Num = 10;
    int    Tree_Rebuild_Period = 50;
    int    tree_making_period_conter = 0;

}; // HSCManager

