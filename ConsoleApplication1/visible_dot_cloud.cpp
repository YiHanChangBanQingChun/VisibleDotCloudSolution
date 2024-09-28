/**
* @birth: created by admin on 2024/09/26 08：30
* 
* @desc: 本代码用于读取并可视化点云文件，支持的文件格式包括PCD、LAS、TXT、PLY
* @desc: 本代码使用PCL库和PDAL库，需要在项目中引入PCL和PDAL库
* 
* @author: 地信221 黄华杰 32216160165
* 
* @version: 1.0
* 
* @update: 2024/09/28 14:38
* 
* @example: 示例数据位于 "d:/Users/admin/Downloads/chromedownload/dotcloud" 目录下
* @example: 示例数据包括 "AA.las"、"rabbit.pcd"、"stgallencathedral_station1_intensity_rgb.txt"、"xyzrgb_dragon.ply"
* @example: 示例数据可以通过修改 filePath 变量来选择不同的文件进行测试
* @example: 示例数据目前.las,.pcd实现功能较多，可以修改颜色选择，如强度和色彩标签，具有一定的鲁棒性。
* @example：而.txt,.ply只能根据z值设置颜色，功能较少，后续可以继续完善。
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
 *
 * @details
 * - 如果PCD文件包含RGB字段，将使用RGB字段进行可视化
 * - 如果PCD文件包含Intensity字段，将使用Intensity字段进行可视化，并进行灰度渲染
 * - 如果PCD文件不包含RGB或Intensity字段，将使用Z值进行彩虹色渲染
 * - 如果PCD文件不包含RGB或Intensity字段，且Z值无法使用彩虹色渲染，将使用Z值进行灰度渲染
 */
void visualizePCD(const std::string& pcdFilePath, const std::string& field)
{
    bool hasRGB = hasRGBField(pcdFilePath);
    bool hasIntensity = hasIntensityField(pcdFilePath);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI(new pcl::PointCloud<pcl::PointXYZI>);

    bool success = false;

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

        // 计算点云的边界值和中心点
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloudRGB, minPt, maxPt);
        pcl::PointXYZRGB center;
        center.x = (minPt.x + maxPt.x) / 2;
        center.y = (minPt.y + maxPt.y) / 2;
        center.z = (minPt.z + maxPt.z) / 2;

        // 设置相机位置
        viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

        // 打印前10个点的坐标
        std::cout << "First 10 points:" << std::endl;
        for (size_t i = 0; i < std::min<size_t>(10, cloudRGB->points.size()); ++i)
        {
            std::cout << "Point " << i << ": (" << cloudRGB->points[i].x << ", " << cloudRGB->points[i].y << ", " << cloudRGB->points[i].z << ")" << std::endl;
        }

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

        // 计算强度的最小值和最大值
        float minIntensity = std::numeric_limits<float>::max();
        float maxIntensity = std::numeric_limits<float>::lowest();
        for (const auto& point : cloudI->points)
        {
            if (point.intensity < minIntensity) minIntensity = point.intensity;
            if (point.intensity > maxIntensity) maxIntensity = point.intensity;
        }

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
            uint8_t intensityColor = static_cast<uint8_t>(255.0 * (point.intensity - minIntensity) / (maxIntensity - minIntensity));
            pointRGB.r = intensityColor;
            pointRGB.g = intensityColor;
            pointRGB.b = intensityColor;

            cloudRGBI->points.push_back(pointRGB);
        }

        cloudRGBI->width = cloudI->width;
        cloudRGBI->height = cloudI->height;
        cloudRGBI->is_dense = cloudI->is_dense;

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

        // 计算点云的边界值和中心点
        pcl::PointXYZI minPt, maxPt;
        pcl::getMinMax3D(*cloudI, minPt, maxPt);
        pcl::PointXYZI center;
        center.x = (minPt.x + maxPt.x) / 2;
        center.y = (minPt.y + maxPt.y) / 2;
        center.z = (minPt.z + maxPt.z) / 2;

        // 设置相机位置
        viewer->setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

        // 打印前10个点的坐标
        std::cout << "First 10 points:" << std::endl;
        for (size_t i = 0; i < std::min<size_t>(10, cloudI->points.size()); ++i)
        {
            std::cout << "Point " << i << ": (" << cloudI->points[i].x << ", " << cloudI->points[i].y << ", " << cloudI->points[i].z << ")" << std::endl;
        }

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
 *
 * @details
 * - 如果LAS文件包含RGB字段，将使用RGB字段进行可视化
 * - 如果LAS文件包含Intensity字段，将使用Intensity字段进行可视化，并进行灰度渲染
 * - 如果LAS文件不包含RGB或Intensity字段，将使用Z值进行彩虹色渲染
 * - 如果LAS文件不包含RGB或Intensity字段，且Z值无法使用彩虹色渲染，将使用Z值进行灰度渲染
 */
void visualizeLAS(const std::string& lasFilePath, const std::string& field)
{
    std::cout << "Starting visualization of LAS file: " << lasFilePath << std::endl;

    // 设置PDAL读取器选项
    pdal::Options options;
    options.add("filename", lasFilePath);

    // 创建PDAL LasReader
    pdal::LasReader reader;
    reader.setOptions(options);

    // 准备PDAL管道
    pdal::PointTable table;
    try {
        std::cout << "Preparing PDAL reader..." << std::endl;
        reader.prepare(table);
    }
    catch (const std::exception& e) {
        std::cerr << "Error preparing PDAL reader: " << e.what() << std::endl;
        return;
    }

    // 执行PDAL管道
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

    // 初始化边界值
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    std::cout << "Converting PDAL point cloud data to PCL point cloud data..." << std::endl;

    // 初始化强度值的最小值和最大值
    float minIntensity = std::numeric_limits<float>::max();
    float maxIntensity = std::numeric_limits<float>::lowest();

    // 第一次遍历，计算强度值的最小值和最大值
    if (field == "intensity") {
        for (pdal::PointId i = 0; i < view->size(); ++i) {
            float intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);
            minIntensity = std::min(minIntensity, intensity);
            maxIntensity = std::max(maxIntensity, intensity);
        }
        std::cout << "Min intensity: " << minIntensity << ", Max intensity: " << maxIntensity << std::endl;
    }

    // 将PDAL点云数据转换为PCL点云数据
    for (pdal::PointId i = 0; i < view->size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = view->getFieldAs<float>(pdal::Dimension::Id::X, i);
        point.y = view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
        point.z = view->getFieldAs<float>(pdal::Dimension::Id::Z, i);

        // 更新边界值
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);

        if (field == "intensity") {
            // 读取强度信息
            float intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, i);

            // 将强度值映射到0-255范围内
            uint8_t intensityColor = static_cast<uint8_t>(255.0 * (intensity - minIntensity) / (maxIntensity - minIntensity));
            point.r = intensityColor;
            point.g = intensityColor;
            point.b = intensityColor;
        }
        else if (field == "rgb") {
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

    // 打印前10个点的坐标
    std::cout << "First 10 points:" << std::endl;
    for (size_t i = 0; i < std::min<size_t>(10, cloud->points.size()); ++i)
    {
        std::cout << "Point " << i << ": (" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << ")" << std::endl;
    }

    // 创建PCL可视化对象
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"); // 设置点云大小

    // 计算点云的边界值和中心点，设置相机位置
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    pcl::PointXYZRGB center;
    center.x = (minPt.x + maxPt.x) / 2;
    center.y = (minPt.y + maxPt.y) / 2;
    center.z = (minPt.z + maxPt.z) / 2;

    std::cout << "Setting camera position..." << std::endl;

    // 设置相机位置
    viewer.setCameraPosition(center.x, center.y, maxPt.z + (maxPt.z - minPt.z), center.x, center.y, center.z, 0, 1, 0);

    std::cout << "Starting visualization loop..." << std::endl;

    // 主循环，保持窗口打开
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
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
void visualizePointCloud(const std::string& filePath, const std::string& showType)
{
    // 获取文件后缀
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);
    if (extension == "pcd")
    {
        std::cout << "The file is pcd file" << endl;
		visualizePCD(filePath, showType); 
    }
    else if (extension == "las")
    {
        std::cout << "The file is las file" << endl;
		visualizeLAS(filePath, showType);
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
        std::string pcdFilePath = plsFilePath.substr(0, plsFilePath.find_last_of(".")) + "_converted.pcd";

        // 保存为PCD文件
        if (pcl::io::savePCDFileASCII(pcdFilePath, *cloud) == -1)
        {
            PCL_ERROR("Couldn't write file %s \n", pcdFilePath.c_str());
            return "";
        }

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
        std::string pcdFilePath = plsFilePath.substr(0, plsFilePath.find_last_of(".")) + "_converted.pcd";

        // 保存为PCD文件
        if (pcl::io::savePCDFileASCII(pcdFilePath, *cloud) == -1)
        {
            PCL_ERROR("Couldn't write file %s \n", pcdFilePath.c_str());
            return "";
        }

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
    std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/AA.las";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/rabbit.pcd";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/stgallencathedral_station1_intensity_rgb.txt";
    //std::string filePath = "d:/Users/admin/Downloads/chromedownload/dotcloud/xyzrgb_dragon.ply";

    // 是否进行PLS到PCD的转换
    // 设置为 true 进行转换，设置为 false 跳过转换
    bool convertPLS = false; 

    // 获取文件后缀
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);

	//当文件为.las时可选择显示类型
    // 可选值为 "intensity" 或 "rgb"
	std::string showType = "intensity"; 

    if (extension == "las" && convertPLS)
    {
        std::cout << "Start converting pls to pcd..." << std::endl;
		std::string pcdFilePath = convertPLSToPCD(filePath, showType); // 转换PLS文件为PCD文件, 使用Intensity字段
        if (!pcdFilePath.empty())
        {
            // 可视化转换后的点云文件
			visualizePointCloud(pcdFilePath, showType);
        }
    }
    else
    {
        // 可视化点云文件（不进行转换）
        visualizePointCloud(filePath, showType);
    }

    return 0;
}