/**
* @birth: created by admin on 2024/09/26 08：30
* 
* @desc: 本代码用于读取并可视化点云文件，支持的文件格式包括PCD、LAS、TXT、PLY
* @desc: 本代码使用PCL库和PDAL库，需要在项目中引入PCL和PDAL库
* 
* @version: 1.2
* 
* @update: 2024/10/07 11:11
* @update_desc: 增加了高程匀质区的计算功能，并优化了点云数据的可视化效果
* 
* @example: 示例数据位于 "d:/Users/admin/Downloads/chromedownload/dotcloud" 目录下
* @example: 示例数据包括 "AA.las"、"rabbit.pcd"、"stgallencathedral_station1_intensity_rgb.txt"、"xyzrgb_dragon.ply"
* @example: 示例数据可以通过修改 filePath 变量来选择不同的文件进行测试
* @example: 示例数据目前.las,.pcd实现功能较多，可以修改颜色选择，如强度和色彩标签，具有一定的鲁棒性。
* @example：而.txt,.ply只能根据z值设置颜色，功能较少，后续可以继续完善。
* 
* @todo: 优化点云数据的加载速度，减少内存占用
* @todo: 增加对点云数据的滤波和降噪功能
* @todo：目前代码比较屎山，后续需要重构和优化
* 
* @note: 使用本代码时，请确保已安装PCL和PDAL库，并正确配置环境变量
* 
* @see: PCL文档：https://pointclouds.org/documentation/
* @see: PDAL文档：https://pdal.io/
* 
* * @author: 地信221 AA胖虎
*/
#include <iostream>
#include <memory>
#include <string>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/Options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pdal/Metadata.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pcl/kdtree/kdtree_flann.h>

/**
 * @brief 读取PCD文件，判断是否包含RGB字段
 *
 * @param pcdFilePath PCD文件路径
 * @return 是否包含RGB字段
 * @retval true 包含RGB字段
 * @retval false 不包含RBG字段
 */
bool hasRGBField(const std::string& pcdFilePath)
{
    std::ifstream file(pcdFilePath);
    if (!file.is_open())
    {
        std::cerr << "Couldn't open file " << pcdFilePath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.find("FIELDS") != std::string::npos)
        {
            if (line.find("rgb") != std::string::npos)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

/**
 * @brief 读取PCD文件，判断是否包含Intensity字段
 *
 * @param pcdFilePath PCD文件路径
 * @return 是否包含Intensity字段
 * @retval true 包含Intensity字段
 * @retval false 不包含Intensity字段
 */
bool hasIntensityField(const std::string& pcdFilePath)
{
    std::ifstream file(pcdFilePath);
    if (!file.is_open())
    {
        std::cerr << "Couldn't open file " << pcdFilePath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.find("FIELDS") != std::string::npos)
        {
            if (line.find("intensity") != std::string::npos)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

/**
 * @brief 可视化PCD文件
 *
 * @param pcdFilePath PCD文件路径
 * @param field 可视化字段，可选值为 "rgb" 或 "intensity"
 * @param coordinateType 坐标类型，可选值为 "shifted box center" 或 "global box center"
 */
void visualizePCD(const std::string& pcdFilePath, const std::string& field, const std::string& coordinateType, const std::string& iselevationHomogeneity)
{
    bool hasRGB = hasRGBField(pcdFilePath);
    bool hasIntensity = hasIntensityField(pcdFilePath);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI(new pcl::PointCloud<pcl::PointXYZI>);

    bool success = false;

    // 读取偏移量信息
    double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
    std::ifstream txtFile(pcdFilePath);
    std::string line;
    int lineCount = 0;

    // 打印前十行
    while (std::getline(txtFile, line) && lineCount < 10)
    {
        std::cout << line << std::endl;
        lineCount++;
    }

    // 重置文件流位置
    txtFile.clear();
    txtFile.seekg(0, std::ios::beg);

    // 查找偏移量信息
    while (std::getline(txtFile, line))
    {
        // 处理包含偏移量的行
        if (line.find("# OFFSETS") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string temp;
            iss >> temp; // 跳过 "#"
            iss >> temp; // 跳过 "OFFSETS"
            iss >> offsetX >> offsetY >> offsetZ; // 读取偏移量
            break;
        }
    }
    txtFile.close();
    std::cout << "OFFSETS, x: " << offsetX << ", y: " << offsetY << ", z: " << offsetZ << std::endl;

    if (field == "rgb" && hasRGB)
    {
        // 读取PCD文件（XYZRGB）
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdFilePath, *cloudRGB) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", pcdFilePath.c_str());
            return;
        }

        std::cout << "Start loading..." << std::endl;
        std::cout << "Loaded " << cloudRGB->width * cloudRGB->height << " data points from " << pcdFilePath << std::endl;

        // 计算点云的边界值和中心点
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloudRGB, minPt, maxPt);
        pcl::PointXYZRGB center;
        center.x = (minPt.x + maxPt.x) / 2;
        center.y = (minPt.y + maxPt.y) / 2;
        center.z = (minPt.z + maxPt.z) / 2;

        if (coordinateType == "shifted box center")
        {
            // 调整点云的坐标
            for (auto& point : cloudRGB->points)
            {
                point.x -= offsetX;
                point.y -= offsetY;
                //point.z -= offsetZ;
            }
            // 重新计算边界值和中心点
            pcl::getMinMax3D(*cloudRGB, minPt, maxPt);
            center.x = (minPt.x + maxPt.x) / 2;
            center.y = (minPt.y + maxPt.y) / 2;
            center.z = (minPt.z + maxPt.z) / 2;
        }

        // 打印点云范围
        std::cout << "Point cloud range: X[" << minPt.x << ", " << maxPt.x << "], Y[" << minPt.y << ", " << maxPt.y << "], Z[" << minPt.z << ", " << maxPt.z << "]" << std::endl;

        // 计算高程匀质区
        if (iselevationHomogeneity == "true") {
            // 创建 KD-Tree 对象用于邻域搜索
            pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
            kdtree.setInputCloud(cloudRGB);
            // 定义高程匀质区的标准差阈值
            const float elevationHomogeneityThreshold = 0.1f; // 根据实际情况调整

            // 计算高程匀质区
            for (size_t i = 0; i < cloudRGB->points.size(); ++i) {
                pcl::PointXYZRGB searchPoint = cloudRGB->points[i];

                float radius = 1.0f; // 搜索半径
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    // 获取邻域点的 Z 值
                    std::vector<float> zValues;
                    for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                        zValues.push_back(cloudRGB->points[pointIdxRadiusSearch[j]].z);
                    }

                    // 计算 Z 值的标准差
                    float meanZ = std::accumulate(zValues.begin(), zValues.end(), 0.0f) / zValues.size();
                    float varianceZ = 0.0f;
                    for (const auto& z : zValues) {
                        varianceZ += (z - meanZ) * (z - meanZ);
                    }
                    varianceZ /= zValues.size();
                    float stddevZ = std::sqrt(varianceZ);

                    // 判断是否为高程匀质区
                    if (stddevZ < elevationHomogeneityThreshold) {
                        // 标记该点为高程匀质区
                        cloudRGB->points[i].r = 0;
                        cloudRGB->points[i].g = 255;
                        cloudRGB->points[i].b = 0;
                    }
                }
            }
        }

        // 创建PCL可视化对象
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

        try
        {
            // 使用RGB颜色字段
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(cloudRGB);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGB, rgb_handler, "sample cloud");
            success = true;
            std::cout << "Using RGB color field for visualization." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error using RGB color field: " << e.what() << std::endl;
        }

        if (!success)
        {
            std::cout << "Using Z value for color visualization." << std::endl;
            // 使用Z值进行彩虹色渲染
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> z_handler(cloudRGB, "z");
            if (z_handler.isCapable())
            {
                viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGB, z_handler, "sample cloud");
            }
            else
            {
                PCL_ERROR("Cannot create color handler!\n");
                return;
            }
        }

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大小
        viewer->addCoordinateSystem(1.0); // 添加坐标系
        viewer->initCameraParameters(); // 初始化相机参数

        // 设置相机位置
        viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

        // 打印前10个点的坐标，使用高精度和表格格式
        std::cout << "------------First 1000 Points:------------" << std::endl;

        // Define column widths
        const int pointColWidth = 15;
        const int coordColWidth = 15;
        const int rgbColWidth = 6;

        // Print header
        std::cout << std::left << std::setw(pointColWidth) << "Point" << " | "
            << std::left << std::setw(coordColWidth) << "X" << " | "
            << std::left << std::setw(coordColWidth) << "Y" << " | "
            << std::left << std::setw(coordColWidth) << "Z" << " | "
            << std::left << std::setw(rgbColWidth) << "R" << " | "
            << std::left << std::setw(rgbColWidth) << "G" << " | "
            << std::left << std::setw(rgbColWidth) << "B" << std::endl;

        // Print separator
        std::cout << std::string(pointColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << std::endl;

        // Print points
        for (size_t i = 0; i < std::min<size_t>(1000, cloudRGB->points.size()); ++i)
        {
            std::cout << std::left << std::setw(pointColWidth) << i << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloudRGB->points[i].x << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloudRGB->points[i].y << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloudRGB->points[i].z << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloudRGB->points[i].r) << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloudRGB->points[i].g) << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloudRGB->points[i].b) << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;

        // 主循环，保持窗口打开
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100); // 每100毫秒刷新一次
            boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // 休眠100毫秒
        }
    }
    else if (field == "intensity" && hasIntensity)
    {
        // 读取PCD文件（XYZI）
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcdFilePath, *cloudI) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", pcdFilePath.c_str());
            return;
        }

        std::cout << "Start loading..." << std::endl;
        std::cout << "Loaded " << cloudI->width * cloudI->height << " data points from " << pcdFilePath << std::endl;

        // 计算点云的边界值和中心点
        pcl::PointXYZI minPt, maxPt;
        pcl::getMinMax3D(*cloudI, minPt, maxPt);
        pcl::PointXYZI center = { (minPt.x + maxPt.x) / 2, (minPt.y + maxPt.y) / 2, (minPt.z + maxPt.z) / 2 };

        if (coordinateType == "shifted box center")
        {
            for (auto& point : cloudI->points)
            {
                point.x -= offsetX;
                point.y -= offsetY;
                //point.z -= offsetZ;
            }
            pcl::getMinMax3D(*cloudI, minPt, maxPt);
            center = { (minPt.x + maxPt.x) / 2, (minPt.y + maxPt.y) / 2, (minPt.z + maxPt.z) / 2 };
        }

        // 打印点云范围
        std::cout << "Point cloud range: X[" << minPt.x << ", " << maxPt.x << "], Y[" << minPt.y << ", " << maxPt.y << "], Z[" << minPt.z << ", " << maxPt.z << "]" << std::endl;

        // 计算强度的最小值和最大值
        float minIntensity = std::numeric_limits<float>::max();
        float maxIntensity = std::numeric_limits<float>::lowest();

        std::vector<float> intensities; // 用于存储强度值

        for (const auto& point : cloudI->points)
        {
            if (point.intensity < minIntensity) minIntensity = point.intensity;
            if (point.intensity > maxIntensity) maxIntensity = point.intensity;
            intensities.push_back(point.intensity); // 存储强度值
        }

        std::cout << "Min intensity: " << minIntensity << ", Max intensity: " << maxIntensity << std::endl;

        // 计算强度值的区间
        int numBins = 20;
        float interval = (maxIntensity - minIntensity) / static_cast<float>(numBins);
        std::vector<int> counts(numBins, 0); // 用于存储每个区间的数量

        for (const auto& intensity : intensities)
        {
            int index = std::min(static_cast<int>((intensity - minIntensity) / interval), numBins - 1);
            counts[index]++;
        }

        // 打印强度统计信息
        std::cout << "------------Intensity Statistics:------------" << std::endl;
        std::cout << std::left << std::setw(20) << "Range" << " | "
            << std::left << std::setw(20) << "Count" << " | "
            << std::left << std::setw(12) << "Percentage" << " | "
            << std::left << std::setw(20) << "Cumulative Percentage" << std::endl;
        std::cout << std::string(20, '-') << "-|-"
            << std::string(20, '-') << "-|-"
            << std::string(12, '-') << "-|-"
            << std::string(20, '-') << std::endl;

        int totalPoints = intensities.size();
        int cumulativeCount = 0;

        for (int i = 0; i < numBins; ++i)
        {
            float rangeStart = minIntensity + i * interval;
            float rangeEnd = rangeStart + interval;
            int count = counts[i];
            float percentage = (static_cast<float>(count) / totalPoints) * 100.0f;
            cumulativeCount += count;
            float cumulativePercentage = (static_cast<float>(cumulativeCount) / totalPoints) * 100.0f;

            std::cout << std::fixed << std::setprecision(2)
                << "[" << std::setw(6) << rangeStart << ", " << std::setw(6) << rangeEnd << ") | "
                << std::setw(10) << count << " | "
                << std::setw(12) << percentage << "% | "
                << std::setw(20) << cumulativePercentage << "%" << std::endl;
        }
        std::cout << "---------------------------------------------" << std::endl;

        // 创建PCL可视化对象
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

        // 将强度映射到0-255范围内并转换为RGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGBI(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& point : cloudI->points)
        {
            pcl::PointXYZRGB pointRGB;
            pointRGB.x = point.x;
            pointRGB.y = point.y;
            pointRGB.z = point.z;

            // 归一化强度值并转换为RGB
            uint8_t intensityColor = static_cast<uint8_t>(255.0f * (point.intensity - minIntensity) / (maxIntensity - minIntensity));
            pointRGB.r = intensityColor;
            pointRGB.g = intensityColor;
            pointRGB.b = intensityColor;

            cloudRGBI->points.push_back(pointRGB);
        }

        cloudRGBI->width = static_cast<uint32_t>(cloudI->points.size());
        cloudRGBI->height = 1;
        cloudRGBI->is_dense = cloudI->is_dense;

        if (iselevationHomogeneity == "true") {
            // 创建 KD-Tree 对象用于邻域搜索
            pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
            kdtree.setInputCloud(cloudRGBI);
            // 定义高程匀质区的标准差阈值
            const float elevationHomogeneityThreshold = 0.1f; // 根据实际情况调整

            // 计算高程匀质区
            for (size_t i = 0; i < cloudRGBI->points.size(); ++i) {
                pcl::PointXYZRGB searchPoint = cloudRGBI->points[i];

                float radius = 1.0f; // 搜索半径
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    // 获取邻域点的 Z 值
                    std::vector<float> zValues;
                    for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                        zValues.push_back(cloudRGBI->points[pointIdxRadiusSearch[j]].z);
                    }

                    // 计算 Z 值的标准差
                    float meanZ = std::accumulate(zValues.begin(), zValues.end(), 0.0f) / zValues.size();
                    float varianceZ = 0.0f;
                    for (const auto& z : zValues) {
                        varianceZ += (z - meanZ) * (z - meanZ);
                    }
                    varianceZ /= zValues.size();
                    float stddevZ = std::sqrt(varianceZ);

                    // 判断是否为高程匀质区
                    if (stddevZ < elevationHomogeneityThreshold) {
                        // 标记该点为高程匀质区
                        cloudRGBI->points[i].r = 0;
                        cloudRGBI->points[i].g = 255;
                        cloudRGBI->points[i].b = 0;
                    }
                }
            }
        }

        try
        {
            // 使用RGB颜色字段
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(cloudRGBI);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGBI, rgb_handler, "sample cloud");
            success = true;
            std::cout << "Using Intensity field for visualization as RGB." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error using Intensity field: " << e.what() << std::endl;
        }

        if (!success)
        {
            std::cout << "Using Z value for color visualization." << std::endl;
            // 使用Z值进行彩虹色渲染
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> z_handler(cloudRGBI, "z");
            if (z_handler.isCapable())
            {
                viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGBI, z_handler, "sample cloud");
            }
            else
            {
                PCL_ERROR("Cannot create color handler!\n");
                return;
            }
        }

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大小
        viewer->addCoordinateSystem(1.0); // 添加坐标系
        viewer->initCameraParameters(); // 初始化相机参数

        // 设置相机位置
        viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

        // 打印前10个点的坐标，使用高精度和表格格式
        std::cout << "------------First 1000 Points:------------" << std::endl;

		// 默认列宽
        const int pointColWidthI = 15;
        const int coordColWidthI = 15;
        const int intensityColWidthI = 15;

		// 打印表头
        std::cout << std::left << std::setw(pointColWidthI) << "Point" << " | "
            << std::left << std::setw(coordColWidthI) << "X" << " | "
            << std::left << std::setw(coordColWidthI) << "Y" << " | "
            << std::left << std::setw(coordColWidthI) << "Z" << " | "
            << std::left << std::setw(intensityColWidthI) << "Intensity" << std::endl;

		// 打印分隔符
        std::cout << std::string(pointColWidthI, '-') << "-|-"
            << std::string(coordColWidthI, '-') << "-|-"
            << std::string(coordColWidthI, '-') << "-|-"
            << std::string(coordColWidthI, '-') << "-|-"
            << std::string(intensityColWidthI, '-') << std::endl;

		// 打印点
        for (size_t i = 0; i < std::min<size_t>(1000, cloudI->points.size()); ++i)
        {
            std::cout << std::left << std::setw(pointColWidthI) << i << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidthI) << cloudI->points[i].x << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidthI) << cloudI->points[i].y << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidthI) << cloudI->points[i].z << " | "
                << std::fixed << std::setprecision(6) << std::setw(intensityColWidthI) << cloudI->points[i].intensity << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;

        // 主循环，保持窗口打开
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100); // 每100毫秒刷新一次
            boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // 休眠100毫秒
        }
    }
    else
    {
        std::cerr << "Unsupported field or field not found in the PCD file: " << field << std::endl;
    }
}

 /**
  * @brief 可视化LAS文件
  *
  * @param lasFilePath LAS文件路径
  * @param field 可视化字段，可选值为 "intensity" 或 "rgb"
  * @param coordinateType 坐标表现类型，可选值为 "shifted box center", "global box center"
  *
  * @details
  * - 如果LAS文件包含RGB字段，将使用RGB字段进行可视化
  * - 如果LAS文件包含Intensity字段，将使用Intensity字段进行可视化，并显示原始强度值
  * - 根据coordinateType参数选择不同的坐标表现方式：
  *   1. "shifted box center"：将坐标平移，使包围盒中心位于原点
  *   2. "global box center"：使用原始全局坐标
  * - 如果LAS文件不包含RGB或Intensity字段，将使用Z值进行彩虹色渲染
  * - 如果LAS文件不包含RGB或Intensity字段，且Z值无法使用彩虹色渲染，将使用Z值进行灰度渲染
  */
void visualizeLAS(const std::string& lasFilePath, const std::string& field, const std::string& coordinateType, const std::string& iselevationHomogeneity)
{
    std::cout << "Starting visualization of LAS file: " << lasFilePath << std::endl;

    // 验证 coordinateType 参数
    std::vector<std::string> validCoordinateTypes = { "shifted box center", "global box center" };
    if (std::find(validCoordinateTypes.begin(), validCoordinateTypes.end(), coordinateType) == validCoordinateTypes.end()) {
        std::cerr << "Invalid coordinateType: " << coordinateType << std::endl;
        std::cerr << "Valid options are: \"shifted box center\", \"global box center\"" << std::endl;
        return;
    }

    // 设置 PDAL 读取器选项
    pdal::Options options;
    options.add("filename", lasFilePath);

    // 创建 PDAL LasReader
    pdal::LasReader reader;
    reader.setOptions(options);

    // 准备 PDAL 管道
    pdal::PointTable table;
    try {
        std::cout << "Preparing PDAL reader..." << std::endl;
        reader.prepare(table);
    }
    catch (const std::exception& e) {
        std::cerr << "Error preparing PDAL reader: " << e.what() << std::endl;
        return;
    }

    // 执行 PDAL 管道
    pdal::PointViewSet viewSet;
    try {
        std::cout << "Executing PDAL reader..." << std::endl;
        viewSet = reader.execute(table);
    }
    catch (const std::exception& e) {
        std::cerr << "Error executing PDAL reader: " << e.what() << std::endl;
        return;
    }

    if (viewSet.empty()) {
        std::cerr << "Error: No point views returned by PDAL reader." << std::endl;
        return;
    }

    // 获取点云视图
    pdal::PointViewPtr view = *viewSet.begin();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 读取LAS头信息
    try {
        const pdal::LasHeader& header = reader.header();
        std::cout << "LAS Header Information:" << std::endl;
        std::cout << "File Signature: " << header.fileSignature() << std::endl;
        std::cout << "Version: " << header.versionMajor() << "." << header.versionMinor() << std::endl;
        std::cout << "System Identifier: " << header.systemId() << std::endl;
        std::cout << "Generating Software: " << header.softwareId() << std::endl;
        std::cout << "Creation Date: " << header.creationDOY() << "/" << header.creationYear() << std::endl;
        std::cout << "Point Format: " << static_cast<int>(header.pointFormat()) << std::endl;
		std::cout << "Coordinate Reference System: " << header.srs() << std::endl;
        std::cout << "Projected id:" << header.projectId() << endl;
        std::cout << "Number of Points: " << header.pointCount() << std::endl;
        std::cout << "Scale Factors: " << header.scaleX() << ", " << header.scaleY() << ", " << header.scaleZ() << std::endl;
        std::cout << "Offsets: " << header.offsetX() << ", " << header.offsetY() << ", " << header.offsetZ() << std::endl;
        std::cout << "Min Bounds: " << header.minX() << ", " << header.minY() << ", " << header.minZ() << std::endl;
        std::cout << "Max Bounds: " << header.maxX() << ", " << header.maxY() << ", " << header.maxZ() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error retrieving LAS header information: " << e.what() << std::endl;
    }

    // 初始化原始边界值
    float orig_minX = std::numeric_limits<float>::max();
    float orig_maxX = std::numeric_limits<float>::lowest();
    float orig_minY = std::numeric_limits<float>::max();
    float orig_maxY = std::numeric_limits<float>::lowest();
    float orig_minZ = std::numeric_limits<float>::max();
    float orig_maxZ = std::numeric_limits<float>::lowest();

    std::cout << "Calculating original Bounding Box..." << std::endl;

    // 第一遍遍历，计算原始的最小和最大值
    for (pdal::PointId i = 0; i < view->size(); ++i) {
        float x = view->getFieldAs<float>(pdal::Dimension::Id::X, i);
        float y = view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
        float z = view->getFieldAs<float>(pdal::Dimension::Id::Z, i);

        orig_minX = std::min(orig_minX, x);
        orig_maxX = std::max(orig_maxX, x);
        orig_minY = std::min(orig_minY, y);
        orig_maxY = std::max(orig_maxY, y);
        orig_minZ = std::min(orig_minZ, z);
        orig_maxZ = std::max(orig_maxZ, z);
    }

    // 计算中心和尺寸
    float centerX = (orig_minX + orig_maxX) / 2.0f;
    float centerY = (orig_minY + orig_maxY) / 2.0f;
    float centerZ = (orig_minZ + orig_maxZ) / 2.0f;

    float sizeX = orig_maxX - orig_minX;
    float sizeY = orig_maxY - orig_minY;
    float sizeZ = orig_maxZ - orig_minZ;

    // 初始化转换后的边界值
    float trans_minX = std::numeric_limits<float>::max();
    float trans_maxX = std::numeric_limits<float>::lowest();
    float trans_minY = std::numeric_limits<float>::max();
    float trans_maxY = std::numeric_limits<float>::lowest();
    float trans_minZ = std::numeric_limits<float>::max();
    float trans_maxZ = std::numeric_limits<float>::lowest();

    // 如果使用强度字段，第一次遍历计算强度的最小值和最大值
    float minIntensity = std::numeric_limits<float>::max();
    float maxIntensity = std::numeric_limits<float>::lowest();
    std::vector<float> intensities; // 用于存储强度值

    if (field == "intensity") {
        std::cout << "Calculating intensity min and max..." << std::endl;
        for (pdal::PointId i = 0; i < view->size(); ++i) {
            float intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);
            minIntensity = std::min(minIntensity, intensity);
            maxIntensity = std::max(maxIntensity, intensity);
            intensities.push_back(intensity); // 存储强度值
        }
        std::cout << "Min intensity: " << minIntensity << ", Max intensity: " << maxIntensity << std::endl;
    }

    // 读取LAS头偏移量
    double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
    try {
        const pdal::LasHeader& header = reader.header();
        // 读取偏移量
        offsetX = header.offsetX();
        offsetY = header.offsetY();
        offsetZ = header.offsetZ();

        // 验证是否成功读取偏移量
        if (offsetX == 0.0 && offsetY == 0.0 && offsetZ == 0.0) {
            std::cerr << "Could not retrieve LAS header offsets. Ensure the LAS file contains offset information." << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error retrieving LAS header offsets: " << e.what() << std::endl;
    }

    // 输出偏移量，设置输出精度为6位小数
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Offset X: " << offsetX << std::endl;
    std::cout << "Offset Y: " << offsetY << std::endl;
    std::cout << "Offset Z: " << offsetZ << std::endl;


    std::cout << "Converting PDAL point cloud data to PCL point cloud data and applying coordinate transformations..." << std::endl;

    // 创建 CSV 文件
    std::ofstream csvFile(R"(d:\Users\admin\Downloads\chromedownload\dotcloud\point_cloud_data.csv)");
    csvFile << "PointID,X,Y,Z,Intensity\n";

    // 第二遍遍历，应用坐标转换，填充PCL点云，并计算转换后的最小和最大值
    for (pdal::PointId i = 0; i < view->size(); ++i) {
        pcl::PointXYZRGB point;

        // 获取原始坐标
        float x = view->getFieldAs<float>(pdal::Dimension::Id::X, i);
        float y = view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
        float z = view->getFieldAs<float>(pdal::Dimension::Id::Z, i);

        // 根据 coordinateType 应用转换
        if (coordinateType == "shifted box center") {
            // 平移坐标，使包围盒中心位于原点
            // 这里使用 LAS 头部的偏移量
            x -= offsetX;
            y -= offsetY;
            //z -= offsetZ;
        }
        else if (coordinateType == "global box center") {
            // 不进行任何转换，使用原始坐标
        }

        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        // 更新转换后的边界值
        trans_minX = std::min(trans_minX, x);
        trans_maxX = std::max(trans_maxX, x);
        trans_minY = std::min(trans_minY, y);
        trans_maxY = std::max(trans_maxY, y);
        trans_minZ = std::min(trans_minZ, z);
        trans_maxZ = std::max(trans_maxZ, z);

        // 根据 field 处理颜色
        if (field == "intensity") {
            // 读取强度信息
            float intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);

            // 将强度值映射到0-255范围内，避免除以0
            uint8_t intensityColor = 0;
            if (maxIntensity != minIntensity) {
                intensityColor = static_cast<uint8_t>(255.0f * (intensity - minIntensity) / (maxIntensity - minIntensity));
            }
            point.r = intensityColor;
            point.g = intensityColor;
            point.b = intensityColor;

            // 写入 CSV 文件
            csvFile << i << "," << x << "," << y << "," << z << "," << intensity << "\n";
        }
        else if (field == "rgb") {
            // 检查RGB字段是否存在
            if (!view->hasDim(pdal::Dimension::Id::Red) ||
                !view->hasDim(pdal::Dimension::Id::Green) ||
                !view->hasDim(pdal::Dimension::Id::Blue)) {
                std::cerr << "RGB fields are missing in the LAS file." << std::endl;
                return;
            }

            // 读取颜色信息
            uint16_t red = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Red, i);
            uint16_t green = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Green, i);
            uint16_t blue = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Blue, i);

            // 将颜色值转换为8位
            point.r = static_cast<uint8_t>(red / 256);
            point.g = static_cast<uint8_t>(green / 256);
            point.b = static_cast<uint8_t>(blue / 256);
        }

        cloud->points.push_back(point);
    }

    // 设置点云参数
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    std::cout << "Number of points in cloud: " << cloud->points.size() << std::endl;

    if (iselevationHomogeneity == "true") {
        // 创建 KD-Tree 对象用于邻域搜索
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud);
        // 定义高程匀质区的标准差阈值
        const float elevationHomogeneityThreshold = 0.1f; // 根据实际情况调整

        // 计算高程匀质区
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            pcl::PointXYZRGB searchPoint = cloud->points[i];

            float radius = 1.0f; // 搜索半径
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                // 获取邻域点的 Z 值
                std::vector<float> zValues;
                for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                    zValues.push_back(cloud->points[pointIdxRadiusSearch[j]].z);
                }

                // 计算 Z 值的标准差
                float meanZ = std::accumulate(zValues.begin(), zValues.end(), 0.0f) / zValues.size();
                float varianceZ = 0.0f;
                for (const auto& z : zValues) {
                    varianceZ += (z - meanZ) * (z - meanZ);
                }
                varianceZ /= zValues.size();
                float stddevZ = std::sqrt(varianceZ);

                // 判断是否为高程匀质区
                if (stddevZ < elevationHomogeneityThreshold) {
                    // 标记该点为高程匀质区
                    cloud->points[i].r = 0;
                    cloud->points[i].g = 255;
                    cloud->points[i].b = 0;
                }
            }
        }
    }

    // 打印转换后的 Bounding Box
    std::cout << "Transformed Bounding Box:" << std::endl;
    std::cout << "X: [" << trans_minX << ", " << trans_maxX << "]" << std::endl;
    std::cout << "Y: [" << trans_minY << ", " << trans_maxY << "]" << std::endl;
    std::cout << "Z: [" << trans_minZ << ", " << trans_maxZ << "]" << std::endl;

    // 打印前30个点的坐标
    std::cout << "------------First 3000 Points:------------" << std::endl;

    // 定义列宽
    const int pointColWidth = 15;
    const int coordColWidth = 15;
    const int intensityColWidth = 15;
    const int rgbColWidth = 6;

    if (field == "intensity") {
        // 打印表头
        std::cout << std::left << std::setw(pointColWidth) << "Point" << " | "
            << std::left << std::setw(coordColWidth) << "X" << " | "
            << std::left << std::setw(coordColWidth) << "Y" << " | "
            << std::left << std::setw(coordColWidth) << "Z" << " | "
            << std::left << std::setw(intensityColWidth) << "Intensity" << std::endl;
        // 打印分隔符
        std::cout << std::string(pointColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(intensityColWidth, '-') << std::endl;

        for (size_t i = 0; i < std::min<size_t>(3000, cloud->points.size()); ++i)
        {
            std::cout << std::left << std::setw(pointColWidth) << i << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].x << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].y << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].z << " | "
                << std::fixed << std::setprecision(6) << std::setw(intensityColWidth) << intensities[i] << std::endl;
        }
    }
    else if (field == "rgb") {
        // 打印表头
        std::cout << std::left << std::setw(pointColWidth) << "Point" << " | "
            << std::left << std::setw(coordColWidth) << "X" << " | "
            << std::left << std::setw(coordColWidth) << "Y" << " | "
            << std::left << std::setw(coordColWidth) << "Z" << " | "
            << std::left << std::setw(rgbColWidth) << "R" << " | "
            << std::left << std::setw(rgbColWidth) << "G" << " | "
            << std::left << std::setw(rgbColWidth) << "B" << std::endl;
        // 打印分隔符
        std::cout << std::string(pointColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << "-|-"
            << std::string(rgbColWidth, '-') << std::endl;

        for (size_t i = 0; i < std::min<size_t>(30, cloud->points.size()); ++i)
        {
            std::cout << std::left << std::setw(pointColWidth) << i << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].x << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].y << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].z << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloud->points[i].r) << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloud->points[i].g) << " | "
                << std::setw(rgbColWidth) << static_cast<int>(cloud->points[i].b) << std::endl;
        }
    }
    else {
        // 打印表头（无强度或RGB）
        std::cout << std::left << std::setw(pointColWidth) << "Point" << " | "
            << std::left << std::setw(coordColWidth) << "X" << " | "
            << std::left << std::setw(coordColWidth) << "Y" << " | "
            << std::left << std::setw(coordColWidth) << "Z" << std::endl;
        // 打印分隔符
        std::cout << std::string(pointColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << "-|-"
            << std::string(coordColWidth, '-') << std::endl;

        for (size_t i = 0; i < std::min<size_t>(30, cloud->points.size()); ++i)
        {
            std::cout << std::left << std::setw(pointColWidth) << i << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].x << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].y << " | "
                << std::fixed << std::setprecision(6) << std::setw(coordColWidth) << cloud->points[i].z << std::endl;
        }
    }

    // 如果使用强度字段，计算并打印强度统计信息
    if (field == "intensity") {
        if (intensities.empty()) {
            std::cerr << "No intensity data available for statistics." << std::endl;
        }
        else {
            // 计算强度值的区间
            int numBins = 20;
            float interval = (maxIntensity - minIntensity) / static_cast<float>(numBins);
            std::vector<int> counts(numBins, 0); // 用于存储每个区间的数量

            for (const auto& intensity : intensities)
            {
                int index = std::min(static_cast<int>((intensity - minIntensity) / interval), numBins - 1);
                counts[index]++;
            }

            // 打印强度统计信息
            std::cout << "------------Intensity Statistics:------------" << std::endl;
            std::cout << std::left << std::setw(15) << "Range" << " | "
                << std::left << std::setw(10) << "Count" << " | "
                << std::left << std::setw(12) << "Percentage" << " | "
                << std::left << std::setw(20) << "Cumulative Percentage" << std::endl;
            std::cout << std::string(15, '-') << "-|-"
                << std::string(10, '-') << "-|-"
                << std::string(12, '-') << "-|-"
                << std::string(20, '-') << std::endl;

            int totalPoints = intensities.size();
            int cumulativeCount = 0;

            for (int i = 0; i < numBins; ++i)
            {
                float rangeStart = minIntensity + i * interval;
                float rangeEnd = rangeStart + interval;
                int count = counts[i];
                float percentage = (static_cast<float>(count) / totalPoints) * 100.0f;
                cumulativeCount += count;
                float cumulativePercentage = (static_cast<float>(cumulativeCount) / totalPoints) * 100.0f;

                std::cout << std::fixed << std::setprecision(2)
                    << "[" << std::setw(6) << rangeStart << ", " << std::setw(6) << rangeEnd << ") | "
                    << std::setw(10) << count << " | "
                    << std::setw(12) << percentage << "% | "
                    << std::setw(20) << cumulativePercentage << "%" << std::endl;
            }
            std::cout << "---------------------------------------------" << std::endl;
        }
    }

    // 创建 PCL 可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

    try
    {
        if (field == "intensity") {
            // 使用 Intensity 映射为 RGB
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb_handler, "sample cloud");
            std::cout << "Using Intensity field for visualization as RGB." << std::endl;
        }
        else if (field == "rgb") {
            // 使用 RGB 颜色字段
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb_handler, "sample cloud");
            std::cout << "Using RGB color field for visualization." << std::endl;
        }
        else {
            // 使用 Z 值进行彩虹色渲染
            std::cout << "Using Z value for color visualization." << std::endl;
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> z_handler(cloud, "z");
            if (z_handler.isCapable())
            {
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud, z_handler, "sample cloud");
            }
            else
            {
                PCL_ERROR("Cannot create color handler!\n");
                return;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error using color field: " << e.what() << std::endl;
        // 尝试使用 Z 值进行彩虹色渲染
        std::cout << "Using Z value for color visualization." << std::endl;
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> z_handler(cloud, "z");
        if (z_handler.isCapable())
        {
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, z_handler, "sample cloud");
        }
        else
        {
            PCL_ERROR("Cannot create color handler!\n");
            return;
        }
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大小
    viewer->addCoordinateSystem(1.0); // 添加坐标系
    viewer->initCameraParameters(); // 初始化相机参数

    // 计算点云的边界值和中心点
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    pcl::PointXYZRGB center;
    center.x = (minPt.x + maxPt.x) / 2.0f;
    center.y = (minPt.y + maxPt.y) / 2.0f;
    center.z = (minPt.z + maxPt.z) / 2.0f;

    std::cout << "Setting camera position..." << std::endl;

    // 设置相机位置
    viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

    std::cout << "Starting visualization loop..." << std::endl;

    // 主循环，保持窗口打开
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // 休眠100毫秒
    }

    std::cout << "Visualization finished." << std::endl;
}

/**
 * @brief 可视化TXT文件
 *
 * @param txtFilePath TXT文件路径
 *
 * @details
 * - 读取TXT文件
 * - 根据z值设置颜色
*/
void visualizeTXT(const std::string& txtFilePath)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 打开TXT文件
    std::ifstream infile(txtFilePath);
    if (!infile.is_open())
    {
        std::cerr << "Couldn't open file " << txtFilePath << std::endl;
        return;
    }

    // 读取TXT文件中的点云数据
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        pcl::PointXYZRGB point;
        float intensity;
        if (!(iss >> point.x >> point.y >> point.z >> intensity >> point.r >> point.g >> point.b))
        {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        cloud->points.push_back(point);
    }

    // 设置点云参数
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    std::cout << "Number of points in cloud: " << cloud->points.size() << std::endl;

    // 创建PCL可视化对象
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"); // 设置点云大小

    // 计算点云的边界值和中心点
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    pcl::PointXYZRGB center;
    center.x = (minPt.x + maxPt.x) / 2;
    center.y = (minPt.y + maxPt.y) / 2;
    center.z = (minPt.z + maxPt.z) / 2;

    // 设置相机位置
    viewer.setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

    // 主循环，保持窗口打开
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

/**
 * @brief 可视化PLY文件
 * 
 * @param plyFilePath PLY文件路径
 * 
 * @details
 * - 读取PLY文件
 * - 根据z值设置颜色
*/
void visualizePLY(const std::string& plyFilePath)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PLY文件
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFilePath, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", plyFilePath.c_str());
        return;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << plyFilePath << std::endl;

    // 创建PCL可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

    // 根据z值设置颜色
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
    if (!color_handler.isCapable())
    {
        PCL_ERROR("Cannot create color handler!\n");
        return;
    }

    // 添加点云到可视化对象
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"); // 设置点云大小

    // 计算点云的边界值和中心点
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    pcl::PointXYZ center;
    center.x = (minPt.x + maxPt.x) / 2;
    center.y = (minPt.y + maxPt.y) / 2;
    center.z = (minPt.z + maxPt.z) / 2;

    // 设置相机位置
    viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

    // 主循环，保持窗口打开
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // 休眠100毫秒
    }
}

/**
 * @brief 可视化点云文件
 *
 * @param filePath 点云文件路径
 * @param showType 显示类型，可选值为 "intensity" 或 "rgb"
 *
 * @details
 * - 支持的文件格式：PCD、LAS、TXT、PLY
 * - 如果文件格式为PCD，将根据showType显示类型进行可视化
 * - 如果文件格式为LAS，将根据showType显示类型进行可视化
 * - 如果文件格式为TXT，将根据z值设置颜色进行可视化
 * - 如果文件格式为PLY，将根据z值设置颜色进行可视化
*/
void visualizePointCloud(const std::string& filePath, const std::string& showType, const std::string& coordinateType, const std::string& iselevationHomogeneity)
{
    // 获取文件后缀
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);
    if (extension == "pcd")
    {
        std::cout << "The file is pcd file" << endl;
        visualizePCD(filePath, showType, coordinateType, iselevationHomogeneity);
    }
    else if (extension == "las")
    {
        std::cout << "The file is las file" << endl;
		visualizeLAS(filePath, showType, coordinateType, iselevationHomogeneity);
    }
    else if (extension == "txt")
    {
        std::cout << "The file is txt file" << endl;
        visualizeTXT(filePath);
    }
    else if (extension == "ply")
    {
        std::cout << "The file is ply file" << endl;
        visualizePLY(filePath);
    }
    else
    {
        std::cerr << "Unsupported file format!" << std::endl;
    }
}

/**
 * @brief 将PLS文件转换为PCD文件
 *
 * @param plsFilePath PLS文件路径
 * @param format 转换格式，可选值为 "intensity" 或 "rgb"
 * @return 转换后的PCD文件路径
 */
std::string convertPLSToPCD(const std::string& plsFilePath, const std::string& format)
{
    // 创建PDAL读取器
    pdal::LasReader reader;
    pdal::Options options;
    options.add("filename", plsFilePath);
    reader.setOptions(options);

    // 创建PDAL表和视图
    pdal::PointTable table;
    reader.prepare(table);
    pdal::PointViewSet viewSet = reader.execute(table);
    pdal::PointViewPtr view = *viewSet.begin();

    // 读取LAS头偏移量
    double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
    try {
        const pdal::LasHeader& header = reader.header();
        // 读取偏移量
        offsetX = header.offsetX();
        offsetY = header.offsetY();
        offsetZ = header.offsetZ();

        std::cout << "Offset X: " << offsetX << ", Offset Y: " << offsetY << ", Offset Z: " << offsetZ << std::endl;
        // 验证是否成功读取偏移量
        if (offsetX == 0.0 && offsetY == 0.0 && offsetZ == 0.0) {
            std::cerr << "Could not retrieve LAS header offsets. Ensure the LAS file contains offset information." << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error retrieving LAS header offsets: " << e.what() << std::endl;
    }

    if (format == "intensity")
    {
        // 创建PCL点云对象（XYZI）
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->width = static_cast<uint32_t>(view->size());
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(view->size());

        // 将PDAL点云数据转换为PCL点云数据（XYZI）
        for (size_t i = 0; i < view->size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = view->getFieldAs<float>(pdal::Dimension::Id::X, i);
            point.y = view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
            point.z = view->getFieldAs<float>(pdal::Dimension::Id::Z, i);
            point.intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);

            cloud->points[i] = point;
        }

        // 生成新的PCD文件路径，添加 "_converted_XYZI" 后缀
        std::string pcdFilePath = plsFilePath.substr(0, plsFilePath.find_last_of(".")) + "_converted_XYZI.pcd";

        // 保存为PCD文件 (使用 binary format to preserve full precision)
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(pcdFilePath, *cloud);

        // 打开PCD文件并在文件头部添加偏移量信息
        std::ifstream inFile(pcdFilePath, std::ios::binary);
        std::ofstream outFile(pcdFilePath + ".tmp", std::ios::binary);
        outFile << "# OFFSETS " << offsetX << " " << offsetY << " " << offsetZ << "\n";
        outFile << inFile.rdbuf();
        inFile.close();
        outFile.close();
        std::remove(pcdFilePath.c_str());
        std::rename((pcdFilePath + ".tmp").c_str(), pcdFilePath.c_str());

        std::cout << "Number of points in cloud: " << cloud->points.size() << std::endl;
        std::cout << "Convert success!" << std::endl;
        std::cout << "Converted " << plsFilePath << " to " << pcdFilePath << std::endl;
        return pcdFilePath;
    }
    else if (format == "rgb")
    {
        // 创建PCL点云对象（XYZRGB）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->width = static_cast<uint32_t>(view->size());
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(view->size());

        // 将PDAL点云数据转换为PCL点云数据（XYZRGB）
        for (size_t i = 0; i < view->size(); ++i)
        {
            pcl::PointXYZRGB point;
            point.x = view->getFieldAs<float>(pdal::Dimension::Id::X, i);
            point.y = view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
            point.z = view->getFieldAs<float>(pdal::Dimension::Id::Z, i);

            // 检查RGB字段是否存在
            if (!view->hasDim(pdal::Dimension::Id::Red) ||
                !view->hasDim(pdal::Dimension::Id::Green) ||
                !view->hasDim(pdal::Dimension::Id::Blue)) {
                std::cerr << "RGB fields are missing in the LAS file." << std::endl;
                return "";
            }

            // 读取颜色信息
            uint16_t red = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Red, i);
            uint16_t green = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Green, i);
            uint16_t blue = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Blue, i);

            // 将颜色值转换为8位
            point.r = static_cast<uint8_t>(red / 256);
            point.g = static_cast<uint8_t>(green / 256);
            point.b = static_cast<uint8_t>(blue / 256);

            cloud->points[i] = point;
        }

        // 生成新的PCD文件路径，添加 "_converted_XYZRGB" 后缀
        std::string pcdFilePath = plsFilePath.substr(0, plsFilePath.find_last_of(".")) + "_converted_XYZRGB.pcd";

        // 保存为PCD文件 (使用 binary format to preserve full precision)
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(pcdFilePath, *cloud);

        // 打开PCD文件并在文件头部添加偏移量信息
        std::ifstream inFile(pcdFilePath, std::ios::binary);
        std::ofstream outFile(pcdFilePath + ".tmp", std::ios::binary);
        outFile << "# OFFSETS " << offsetX << " " << offsetY << " " << offsetZ << "\n";
        outFile << inFile.rdbuf();
        inFile.close();
        outFile.close();
        std::remove(pcdFilePath.c_str());
        std::rename((pcdFilePath + ".tmp").c_str(), pcdFilePath.c_str());

        std::cout << "Number of points in cloud: " << cloud->points.size() << std::endl;
        std::cout << "Convert success!" << std::endl;
        std::cout << "Converted " << plsFilePath << " to " << pcdFilePath << std::endl;
        return pcdFilePath;
    }
    else
    {
        std::cerr << "Unsupported format: " << format << std::endl;
        return "";
    }
}

/**
 * @brief 主函数
 *
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出状态
 */
int main(int argc, char** argv)
{
    // 硬编码文件路径，测试用例
    std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/BB.las";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/rabbit.pcd";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/stgallencathedral_station1_intensity_rgb.txt";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/xyzrgb_dragon.ply";

    // 是否进行PLS到PCD的转换
    // 设置为 true 进行转换，设置为 false 跳过转换
    //bool convertPLS = true; 
	bool convertPLS = false;

    // 获取文件后缀
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);

	//当文件为.las时可选择显示类型
    // 可选值为 "intensity" 或 "rgb"
	//std::string showType = "intensity";
    std::string showType = "rgb";
    
	// 可选值为 "shifted box center", "global box center"，也就是是否进行偏移
	std::string coordinateType = "global box center";

	// 是否进行高程匀质区标记
	std::string iselevationHomogeneity = "true";

    if (extension == "las" && convertPLS)
    {
        std::cout << "Start converting pls to pcd..." << std::endl;
		std::string pcdFilePath = convertPLSToPCD(filePath, showType); // 转换PLS文件为PCD文件, 使用Intensity字段
        if (!pcdFilePath.empty())
        {
            // 可视化转换后的点云文件
			visualizePointCloud(pcdFilePath, showType, coordinateType, iselevationHomogeneity);
        }
    }
    else
    {
        // 可视化点云文件（不进行转换）
		visualizePointCloud(filePath, showType, coordinateType, iselevationHomogeneity);
    }

    return 0;
}