#include <HSC.h>

// ----------------------------------------API-----------------------------------------

// descriptor genaration
void HSCManager::Descriptor_Generate( pcl::PointCloud<HSCPointType> & _scan_down )
{
    UpdateLevel(_scan_down);
    
    // D_h,D_m,D_hsc
    Eigen::MatrixXd hsc_multiple = HSC_Generate(_scan_down);
    Eigen::MatrixXi hsc_mask = Mask_HSC(hsc_multiple);
    Eigen::MatrixXd hsc_single = Multiple2Single_mask(hsc_multiple, hsc_mask);

    // save D_hsc
    des_dataset.push_back( hsc_single );
    
    // ring_key
    Eigen::MatrixXd ringkey_mat = Ringkey_Generate( hsc_single );
    std::vector<float> ringkey_vec = eig2stdvec( ringkey_mat );
    ringkey_vector.push_back(ringkey_vec);
}

// LCD
void HSCManager::LCD( void )
{
    int loop_id { -1 };

    // step 1: KNN search

    // ignore 50 frames before current frame
    if(ringkey_vector.size() < Ignore_Num_Before_Current + 1)
    {
        LCD_index_dataset.push_back(loop_id);
        LCD_corr_dataset.push_back(10000000);
        return ;
    } 

    // KDTree generation
    if ( tree_making_period_conter % Tree_Rebuild_Period == 0 )
    {
        ringkey_kdtree.clear();
        ringkey_kdtree.assign( ringkey_vector.begin(), ringkey_vector.end() - Ignore_Num_Before_Current ) ;
        search_kdtree.reset(); 
        search_kdtree = std::make_unique<InvKeyTree>( RING_NUM, ringkey_kdtree, 10 );
    }
    tree_making_period_conter += 1;


    double min_dist = 10000000;
    int nn_align = 0;
    int nn_idx = 0;

    // KNN search
    TicToc knn_search;
    std::vector<size_t> candidate_final;  // final result C
    std::vector<size_t> candidate_indexes( Candidate_Num );
    std::vector<float> distances(Candidate_Num);
    auto curr_ringkey = ringkey_vector.back();

    nanoflann::KNNResultSet<float> knnsearch_result( Candidate_Num );
    knnsearch_result.init( &candidate_indexes[0], &distances[0] );
    search_kdtree->index->findNeighbors( knnsearch_result, &curr_ringkey[0], nanoflann::SearchParams(10) ); 
    for (int i = 0; i < candidate_indexes.size(); i++)
    {candidate_final.push_back(candidate_indexes[i]);}
    
    // candidate frame enhancement
    if ( Use_TimeSeries && Candidate_flag )
        CandidateEnhance(candidate_final);
    
    // find the result with the highest similarity
    auto curr_desc = des_dataset.back();
    for ( int candidate_iter_idx = 0; candidate_iter_idx < candidate_final.size(); candidate_iter_idx++ )
    {
        MatrixXd polarcontext_candidate = des_dataset[ candidate_final[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = DistanceBtnDesc( curr_desc, polarcontext_candidate); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_final[candidate_iter_idx];
            loop_id = nn_idx;
        }
    }
    
    // update time series enhancement
    current_out = (min_dist > TimeEnd_th)?true:false;
    
    // save result
    LCD_index_dataset.push_back(loop_id);
    LCD_corr_dataset.push_back(min_dist);
}


// ---------------------------------------Descriptor---------------------------------------

// generate D_h
Eigen::MatrixXd HSCManager::HSC_Generate( pcl::PointCloud<HSCPointType> & _scan_down )
{
    int num_pts_scan_down = _scan_down.points.size();

    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(RING_NUM * LEVEL, SECTOR_NUM);

    HSCPointType pt;
    float azim_angle, azim_range;
    int ring_idx, sctor_idx, layer_idx;

    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;
        
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);
        if( azim_range > MAX_RADIUS )
            continue;

        // bin index
        ring_idx = std::max( std::min( RING_NUM, int(ceil( (azim_range / MAX_RADIUS) * RING_NUM )) ), 1 ) - 1;
        sctor_idx = std::max( std::min( SECTOR_NUM, int(ceil( (azim_angle / 360.0) * SECTOR_NUM )) ), 1 ) - 1;
        layer_idx = CalculateLevel(pt.z);

        // save
        int ring_index = ring_idx + RING_NUM * (layer_idx-1);
        if ( desc(ring_index, sctor_idx) < pt.z )
            desc(ring_index, sctor_idx) = pt.z;
    }
    
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ ){
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) < 0 )
                desc(row_idx, col_idx) = 0;
    }

    return desc;
}

// generate D_m
Eigen::MatrixXi HSCManager::Mask_HSC( const Eigen::MatrixXd &_desc)
{
    MatrixXi hsc_mask_temp = MatrixXi::Zero(_desc.rows(), _desc.cols());
    MatrixXi hsc_mask = MatrixXi::Zero(_desc.rows(), _desc.cols());

    /**
     * 0 - no point in bin
     * 1 - has point, not edge
     * 2 - has point, edge
    */
   for (int row_idx = 0; row_idx < RING_NUM; row_idx++){
        for (int col_idx = 0; col_idx < SECTOR_NUM; col_idx++){
            for (int layer = 0; layer < LEVEL; layer++)
            {
                double temp_up = GetUpHeight(layer+1);
                double temp_down = GetDownHeight(layer+1);
                int row_curr = row_idx + RING_NUM * layer;
                if (std::fabs(_desc(row_curr,col_idx)) < 1e-15)
                    hsc_mask_temp(row_curr,col_idx) = 0;
                else if(std::abs(_desc(row_curr,col_idx)-temp_down)/std::abs(temp_up-temp_down) < Edge_th)
                    hsc_mask_temp(row_curr,col_idx) = 1;
                else
                    hsc_mask_temp(row_curr,col_idx) = 2;
            }

            bool first_flag = false;
            for (int layer = LEVEL-1; layer >= 0; layer--)
            {
                int row_curr = row_idx + RING_NUM * layer;
                int row_up = row_idx + RING_NUM * (layer+1);
                if (!first_flag && hsc_mask_temp(row_curr,col_idx) != 0)
                {
                    first_flag = true;
                    hsc_mask(row_curr,col_idx) = 1;
                }
                else if (first_flag && hsc_mask_temp(row_curr,col_idx) == 1)
                    hsc_mask(row_curr,col_idx) = 1;
                else if (first_flag && hsc_mask_temp(row_up,col_idx) == 0 && hsc_mask_temp(row_curr,col_idx) == 2)
                    hsc_mask(row_curr,col_idx) = 1;
            }
        }
    }
  
    return hsc_mask;
}

// generate D_hsc
Eigen::MatrixXd HSCManager::Multiple2Single_mask( const Eigen::MatrixXd &_desc, const Eigen::MatrixXi &_mask )
{
    MatrixXd hsc_single = MatrixXd::Zero(RING_NUM, SECTOR_NUM);

    for ( int row_idx = 0; row_idx < hsc_single.rows(); row_idx++ ){
        for ( int col_idx = 0; col_idx < hsc_single.cols(); col_idx++ ){
            for (int layer = 0; layer < LEVEL; layer++){
                int row_curr = row_idx + RING_NUM * layer;
                hsc_single(row_idx, col_idx) += _desc(row_curr, col_idx) * _mask(row_curr, col_idx);
            }
        }
    }

    return hsc_single;
}

// ring key
Eigen::MatrixXd HSCManager::Ringkey_Generate( Eigen::MatrixXd &_desc )
{
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }
    
    return invariant_key;
}

// sector key
Eigen::MatrixXd HSCManager::Sectorkey_Generate( Eigen::MatrixXd &_desc )
{
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
}

// ----------------------------------------hierarchical algorithm----------------------------------------

// update level range
void HSCManager::UpdateLevel( pcl::PointCloud<HSCPointType> & _scan_down )
{
    double max_height = -1000;
    double current_z;
    int num = _scan_down.points.size();

    // find h_max
    for (int pt_idx = 0; pt_idx < num; pt_idx++){
        current_z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;
        if(current_z > max_height)
            max_height = current_z;
    }
    MAX_HEIGHT = std::ceil(max_height);
    LEVEL_SKIP = (2 * (MAX_HEIGHT-2.0)) / (LEVEL * (LEVEL - 1));
}

// Calculate the layer to which the current point belongs
int HSCManager::CalculateLevel(const double& curr_pt_z)
{
    if (curr_pt_z <= 2.0)
        return 1;
    
    double layerStart = 2.0;
    double skip = LEVEL_SKIP;
    double layerEnd = layerStart + skip;
    int layernum = 2;
    
    while (curr_pt_z > layerEnd)
    {
        skip += LEVEL_SKIP;
        layerEnd += skip;
        layernum++;
    }
    
    return layernum;
}

// calculate h_dl
double HSCManager::GetDownHeight(int layer)
{
    if (1 == layer)
        return 0;
    else if (2 == layer)
        return 2.0;
    else
        return 2 + ((layer-1)*(layer-2)/2)*LEVEL_SKIP;
}

// calculate h_ul
double HSCManager::GetUpHeight(int layer)
{
    if (1 == layer)
        return 2.0;
    else
        return 2 + (layer*(layer-1)/2)*LEVEL_SKIP;
}


// --------------------------------------LCD---------------------------------------

// Calculate approximate distance(1-correlation coefficient)
std::pair<double, int> HSCManager::DistanceBtnDesc( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    // 1. align by variant key
    MatrixXd vkey_sc1 = Sectorkey_Generate( _sc1 );
    MatrixXd vkey_sc2 = Sectorkey_Generate( _sc2 );
    int argmin_vkey_shift = FastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * Search_Ratio * _sc1.cols() );
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = Corr_Caculata( _sc1, sc2_shifted );

        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);
}

// align by v
int HSCManager::FastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 )
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;

    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
    
        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
    
        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;
}

// calculate correlation coefficient
double HSCManager::Corr_Caculata ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    double r = 1000;
 
    double fenzi = 0, fenmu1 = 0, fenmu2 = 0;
    double temp1 = 0, temp2 = 0;
    for (int i = 0; i < _sc1.rows(); i++){
        double ave1 = _sc1.row(i).mean();
        double ave2 = _sc2.row(i).mean();
        for (int j = 0; j < _sc1.cols(); j++){
            temp1 = _sc1(i,j) - ave1;
            temp2 = _sc2(i,j) - ave2;
            fenzi += (temp1 * temp2);
            fenmu1 += (temp1 * temp1);
            fenmu2 += (temp2 * temp2);
        }
    }
   
    r = fenzi / std::sqrt(fenmu1 * fenmu2);
  
    return (1 - r);
}


// --------------------------------------Time series enhancement---------------------------------------

// time series enhancement
void HSCManager::TimeSeriesEnhance()
{
    if (TS_flag)
    {
        if (current_out)
        {
            out_num++;
            if (out_num >= 3)
            {
                TS_flag = false;
                Candidate_flag = false;
                last_TS_endindex = frame_index;
            }
        }
        else
            out_num = 0;
    }
    else{     
        if ( LCD_corr_dataset[frame_index-2] < TimeStart_th && LCD_corr_dataset[frame_index-1] < TimeStart_th && LCD_corr_dataset[frame_index] < TimeStart_th )
        {
            TS_flag = true;
            out_num = 0;
            BeforeEnhance();
            Candidate_flag = true;
        }
    }
}

// candidate frame enhancement
void HSCManager::CandidateEnhance(std::vector<size_t>& candidate_final)
{
    int last_index = LCD_index_dataset.back();
    // N_e=5,N_e/2=3
    for (int i = -3; i <= 3; i++){
        if (last_index+i < 0)
            continue;
        candidate_final.push_back(last_index+i);
    }
    std::sort(candidate_final.begin(), candidate_final.end());
    candidate_final.erase(std::unique(candidate_final.begin(),candidate_final.end()),candidate_final.end());
}

// get reference results for forward enhancement
std::pair<int, double> HSCManager::ReGetResult( int current_index )
{
    int base_index;
    if (Before_flag)
        base_index = LCD_index_dataset[current_index+1];
    else
        base_index = LCD_index_dataset[current_index-1];
    std::vector<size_t> candidate_final;
    
    // NC=10
    for (int i = -4; i <= 4; i++){
        if (base_index+i < 0)
            continue;
        
        candidate_final.push_back(base_index+i);
    }

    // find new result
    int loop_id = -1;
    double min_dist = 10000000;
    int nn_idx = 0;
    auto curr_desc = des_dataset[current_index];
    for ( int candidate_iter_idx = 0; candidate_iter_idx < candidate_final.size(); candidate_iter_idx++ )
    {
        MatrixXd polarcontext_candidate = des_dataset[ candidate_final[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = DistanceBtnDesc( curr_desc, polarcontext_candidate); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            // nn_align = candidate_align;

            nn_idx = candidate_final[candidate_iter_idx];
            loop_id = nn_idx;
        }
    }
    
    std::pair<int, double> result {loop_id, min_dist};
    return result;
}

// forward enhancement
void HSCManager::BeforeEnhance()
{
    Before_flag = true;
    int temp_index = frame_index-1;

    while (LCD_corr_dataset[temp_index] < 0.6 && temp_index > last_TS_endindex)
    {    
        std::pair<int, double> new_result = ReGetResult(temp_index);
        
        ProportionMapping(new_result.second);
        
        // update result
        LCD_index_dataset[temp_index] = new_result.first;
        LCD_corr_dataset[temp_index] = new_result.second;

        temp_index -= 1;
        if (temp_index < 0)
            break;
    }
    Before_flag = false;
}

// mapping from [w_s,w_e] to [w_s',w_e']
void HSCManager::ProportionMapping(double& input)
{
    double start = 0.3, end = 0.6, end_new = 0.4;
    input = (end_new-start)*(input-start)/(end-start) + start;
}

// ------------------------------------------Tool function------------------------------------------

// get parameter settings
void HSCManager::GetConfigFromYAML(const std::string& config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    Sequence = config["Seq"]["Sequence"].as<std::string>();

    Raw_Path = config["Path"]["Raw_Path"].as<std::string>();
    Result_Path = config["Path"]["Result_Path"].as<std::string>();

    LIDAR_HEIGHT = config["Descriptor"]["LIDAR_HEIGHT"].as<double>();
    RING_NUM = config["Descriptor"]["RING_NUM"].as<int>();
    SECTOR_NUM = config["Descriptor"]["SECTOR_NUM"].as<int>();
    MAX_RADIUS = config["Descriptor"]["MAX_RADIUS"].as<double>();
    LEVEL = config["Descriptor"]["LEVEL"].as<int>();
    Edge_th = config["Descriptor"]["Edge_th"].as<double>();
    Use_TimeSeries = config["Descriptor"]["Use_TimeSeries"].as<bool>();
    TimeStart_th = config["Descriptor"]["TimeStart_th"].as<double>();
    TimeEnd_th = config["Descriptor"]["TimeEnd_th"].as<double>();

    Ignore_Num_Before_Current = config["KNN"]["Ignore_Num_Before_Current"].as<int>();
    Candidate_Num = config["KNN"]["Candidate_Num"].as<int>();
    Tree_Rebuild_Period = config["KNN"]["Tree_Rebuild_Period"].as<int>();
}

// print parameter settings
void HSCManager::PrintParameter()
{
    std::string timeSeries = Use_TimeSeries?"True":"False";
    std::cerr << "current config settings: " << std::endl;
    std::cerr << "Sequence: " << Sequence << std::endl;
    std::cerr << "Nr-Ns: " << RING_NUM << "-" << SECTOR_NUM << std::endl;
    std::cerr << "Use_TimeSeries: " << timeSeries << std::endl;
}

// get point choud from bin files
void HSCManager::GetCloudFromBIN(const std::string& filename)
{
    FILE* stream = fopen(filename.c_str(), "rb");
    if (!stream) {
        std::cerr << "error path: " + filename << std::endl;
        return ;
    }

    float temp_point[4];
    HSCPointType point;
    while (fread(temp_point, sizeof(double), 4, stream) == 4) {
        point.x = temp_point[0];
        point.y = temp_point[1];
        point.z = temp_point[2];
        cloud_in->push_back(point);
    }
    fclose(stream);
}

// get point choud from NCLT dataset
void HSCManager::GetPointCloudFromNCLT(const std::string& filename)
{
    FILE* stream = fopen(filename.c_str(), "rb");
    if (!stream) {
        std::cerr << "error path: " + filename << std::endl;
        return ;
    }

    float temp_point[4];
    HSCPointType point;
    while (fread(temp_point, sizeof(float), 4, stream) == 4) {
        point.x = temp_point[0];
        point.y = temp_point[1];
        point.z = -temp_point[2];
        cloud_in->push_back(point);
    }
    fclose(stream);
}

// clear
void HSCManager::ClearValue()
{
    cloud_in->clear();
    if (Use_TimeSeries)
        TimeSeriesEnhance();
    frame_index++;
}

// ------------------------------------------Auxiliary function------------------------------------------

float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
}

MatrixXd circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

}

// eigen to vector
std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
}
