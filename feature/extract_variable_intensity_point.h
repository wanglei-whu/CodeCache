#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

int main() {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("input.pcd", *cloud) == -1) {
        std::cerr << "无法读取input.pcd文件" << std::endl;
        return -1;
    }

    // 设置参数
    double radius = 0.1;  // 邻域搜索半径（单位：米）
    double threshold = 5.0;  // 梯度幅度阈值

    // 创建KdTree用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    // 存储提取结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 遍历点云，计算梯度并提取
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> indices;  // 邻域点索引
        std::vector<float> distances;  // 邻域点距离
        tree->radiusSearch(i, radius, indices, distances);

        if (indices.size() < 3) continue;  // 邻域点太少，跳过

        // 计算邻域内强度的加权梯度
        Eigen::Vector3d gradient(0, 0, 0);
        double sum_weights = 0;
        for (size_t j = 0; j < indices.size(); ++j) {
            if (indices[j] == static_cast<int>(i)) continue;  // 跳过自身
            Eigen::Vector3d diff(
                cloud->points[indices[j]].x - cloud->points[i].x,
                cloud->points[indices[j]].y - cloud->points[i].y,
                cloud->points[indices[j]].z - cloud->points[i].z
            );
            double weight = 1.0 / (distances[j] + 1e-6);  // 距离加权，避免除零
            double intensity_diff = cloud->points[indices[j]].intensity - cloud->points[i].intensity;
            gradient += weight * intensity_diff * diff.normalized();
            sum_weights += weight;
        }
        if (sum_weights > 0) gradient /= sum_weights;

        // 计算梯度幅度
        double gradient_magnitude = gradient.norm();

        // 筛选强度变化较大的点
        if (gradient_magnitude > threshold) {
            extracted_cloud->points.push_back(cloud->points[i]);
        }
    }

    // 保存提取的点云
    extracted_cloud->width = extracted_cloud->points.size();
    extracted_cloud->height = 1;
    extracted_cloud->is_dense = true;
    pcl::io::savePCDFileASCII("output.pcd", *extracted_cloud);

    std::cout << "已提取" << extracted_cloud->points.size() << "个强度梯度较大的点，保存至output.pcd" << std::endl;

    return 0;
}
