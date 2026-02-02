#include <chrono> // NOLINT
#include <cctype>
#include <fstream>
#include <iostream>
#include <pcl/common/transforms.h>
#include <sstream>
#include <thread> // NOLINT
#include <time.h>

#include "calibration.hpp"

void SaveExtrinsic(Eigen::Matrix4f T)
{
    std::string file_name = "extrinsic.txt";
    
    std::ofstream ofs(file_name);
    if (!ofs.is_open())
    {
        std::cerr << "open file " << file_name << " failed. Cannot write calib result." << std::endl;
        exit(1);
    }
    ofs << "Extrinsic = " << std::endl;
    ofs << "[" << T(0, 0) << "," << T(0, 1) << "," << T(0, 2) << "," << T(0, 3) << "]," << std::endl
        << "[" << T(1, 0) << "," << T(1, 1) << "," << T(1, 2) << "," << T(1, 3) << "]," << std::endl
        << "[" << T(2, 0) << "," << T(2, 1) << "," << T(2, 2) << "," << T(2, 3) << "]," << std::endl
        << "[" << T(3, 0) << "," << T(3, 1) << "," << T(3, 2) << "," << T(3, 3) << "]" << std::endl;

    ofs << "Roll = " << RAD2DEG(Util::GetRoll(T)) << std::endl
        << "Pitch = " << RAD2DEG(Util::GetPitch(T)) << std::endl
        << "Yaw = " << RAD2DEG(Util::GetYaw(T)) << std::endl
        << "x = " << Util::GetX(T) << std::endl
        << "y = " << Util::GetY(T) << std::endl
        << "z = " << Util::GetZ(T) << std::endl;
    ofs.close();

    std::cout << "Calibration result was saved to file calib_result.txt" << std::endl;
}

bool LoadExtrinsicFromFile(const std::string &file_path, Eigen::Matrix4f &T_custom)
{
    std::ifstream ifs(file_path);
    if (!ifs.is_open())
    {
        std::cerr << "open file " << file_path << " failed. Cannot read extrinsic." << std::endl;
        return false;
    }
    std::ostringstream buffer;
    buffer << ifs.rdbuf();
    std::string content = buffer.str();
    for (char &ch : content)
    {
        if (!(std::isdigit(ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E'))
        {
            ch = ' ';
        }
    }
    std::istringstream iss(content);
    float value = 0.0F;
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            if (!(iss >> value))
            {
                std::cerr << "Failed to parse 4x4 extrinsic from " << file_path << std::endl;
                return false;
            }
            T_custom(row, col) = value;
        }
    }
    return true;
}

int main(int argc, char *argv[]) {
    if (argc != 2 && argc != 4) {
        std::cout << "Usage: ./bin/run_lidar2camera <data_folder> [--extrinsic <file>]\n"
                     "example:\n\t"
                     "./bin/run_lidar2camera data/st/1\n"
                     "./bin/run_lidar2camera data/kitti/1" << std::endl;
        return 0;
    }

    std::string data_folder = argv[1];
    std::string extrinsic_path;
    if (argc == 4) {
        if (std::string(argv[2]) != "--extrinsic") {
            std::cout << "Unknown option: " << argv[2] << std::endl;
            return 0;
        }
        extrinsic_path = argv[3];
    }
    std::string lidar_file, img_file, calib_file;
    std::string mask_dir = data_folder + "/masks/";
    std::string error_file = "./data/initial_error.txt";

    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(data_folder.c_str())) == NULL)
    {
        std::cout << "Open dir " << mask_dir << " error !" << std::endl;
        exit(1);
    }
    std::string extrinsic_basename;
    if (!extrinsic_path.empty())
    {
        size_t pos = extrinsic_path.find_last_of("/\\");
        extrinsic_basename = (pos == std::string::npos) ? extrinsic_path : extrinsic_path.substr(pos + 1);
    }
    while ((ptr = readdir(dir)) != NULL)
    {
        std::string name = ptr->d_name;
        auto n = name.find_last_of('.');
        if(name == "." || name == ".." || n == std::string::npos){
            ptr++;
            continue;
        }
        std::string suffix = name.substr(n);
        if (suffix == ".png" || suffix == ".jpg" || suffix == ".jpeg")
            img_file = data_folder + '/' + ptr->d_name;
        else if (suffix == ".pcd")
            lidar_file = data_folder + '/' + ptr->d_name;
        else if (suffix == ".txt")
        {
            std::string candidate_calib = data_folder + '/' + ptr->d_name;
            if (!extrinsic_path.empty()
                && (candidate_calib == extrinsic_path || ptr->d_name == extrinsic_basename))
            {
                ptr++;
                continue;
            }
            calib_file = data_folder + '/' + ptr->d_name;
        }
        ptr++;
    }

    Eigen::Matrix4f T_custom = Eigen::Matrix4f::Identity();
    bool has_custom_extrinsic = false;
    if (!extrinsic_path.empty())
    {
        has_custom_extrinsic = LoadExtrinsicFromFile(extrinsic_path, T_custom);
        if (!has_custom_extrinsic)
        {
            return 1;
        }
    }

    auto time_begin = std::chrono::steady_clock::now();
    Calibrator calibrator(mask_dir, lidar_file, calib_file, img_file, error_file);
    if (has_custom_extrinsic)
    {
        float score = 0.0F;
        calibrator.CalScore(T_custom, score, true);
        std::cout << "Custom extrinsic score (coarse): " << score << std::endl;
    }
    calibrator.Calibrate();
    Eigen::Matrix4f refined_extrinsic = calibrator.GetFinalTransformation();
    SaveExtrinsic(refined_extrinsic);
    auto time_end = std::chrono::steady_clock::now();
    std::cout << "Total calib time: "
                << std::chrono::duration<double>(time_end - time_begin).count()
                << "s" << std::endl;
                
    return 0;
}
