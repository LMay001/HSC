# include <iostream>
# include <fstream>
#include <filesystem>

#include "HSC.h"

namespace fs = std::filesystem;

int main(void)
{
    TicToc timer(true);

    // Modify it to your path
    std::string config_path = "/home/XXX/HSC/config/parameter.yaml";
    HSCManager loop(config_path);
    loop.PrintParameter();

    std::string folder_path = loop.Raw_Path + loop.Sequence;
    std::vector<fs::path> file_paths;
    for (const auto& entry : fs::recursive_directory_iterator(folder_path)) {
        if (fs::is_regular_file(entry.path()))
            file_paths.push_back(entry.path());
    }
    std::sort(file_paths.begin(), file_paths.end());

    for (auto it = file_paths.begin(); it != file_paths.end(); ++it) {
        std::string filename = *it;

        // get point cloud
        if (loop.Sequence.substr(0,4) == "NCLT")
            loop.GetPointCloudFromNCLT(filename);
        else
            loop.GetCloudFromBIN(filename);

        // get descriptor
        loop.Descriptor_Generate(*(loop.cloud_in));

        // fing loop from history frames
        loop.LCD();

        std::cerr << "frame " << loop.frame_index << " is done." << std::endl;

        loop.ClearValue(); 
    }
    loop.occlusionRate.close();

    // save result
    std::string result_txt = loop.Result_Path + loop.Sequence + "/result_" + loop.Sequence + ".txt";
    std::ofstream outputFile(result_txt, std::ios::trunc);
    if (!outputFile.is_open()) {
        std::cerr << "open file error, error path: " << result_txt << std::endl;
        return -1;
    }
    for (int i = 0; i < loop.LCD_index_dataset.size(); i++)
    {
        if (outputFile.is_open()) {
            outputFile << i << " " << loop.LCD_index_dataset[i] << " " << loop.LCD_corr_dataset[i] << std::endl;
        } else {
            std::cerr << "Can't save result to: " << result_txt << std::endl;
            return -1;
        }
    }
    
    outputFile.close();

    // show the total time
    timer.toc("Total time");

    return 0;
}