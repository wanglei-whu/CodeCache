class FisheyeCamera {
public:
    virtual ~FisheyeCamera() = default;
    
    virtual Eigen::Vector2d project(const Eigen::Vector3d& point3d) const = 0;
    
    virtual Eigen::Vector3d unproject(const Eigen::Vector2d& pixel) const = 0;
};
