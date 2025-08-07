class MeiFisheyeCamera : public FisheyeCamera {
private:
    double fx, fy; // 焦距（像素单位）
    double cx, cy; // 主点坐标
    double xi;     // 镜面参数
    double k1, k2, p1, p2; // 畸变参数

public:
    MeiFisheyeCamera(double fx_, double fy_, double cx_, double cy_, double xi_,
                     double k1_, double k2_, double p1_, double p2_)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_), xi(xi_),
          k1(k1_), k2(k2_), p1(p1_), p2(p2_) {}

    Eigen::Vector2d project(const Eigen::Vector3d& point3d) const override {
        if (point3d.z() <= 0) {
            throw std::runtime_error("Point is behind camera");
        }

        // 归一化坐标
        double X = point3d.x() / point3d.z();
        double Y = point3d.y() / point3d.z();
        double Z = point3d.z();

        // MEI模型投影
        double d = std::sqrt(X*X + Y*Y + Z*Z);
        double factor = (Z + xi * d) / (X*X + Y*Y + (Z + xi * d) * (Z + xi * d));
        
        double x = X * factor;
        double y = Y * factor;

        // 畸变模型
        double r2 = x*x + y*y;
        double r4 = r2*r2;
        double xy = x*y;
        
        double x_distorted = x * (1 + k1*r2 + k2*r4) + 2*p1*xy + p2*(r2 + 2*x*x);
        double y_distorted = y * (1 + k1*r2 + k2*r4) + p1*(r2 + 2*y*y) + 2*p2*xy;

        // 投影到像素坐标
        Eigen::Vector2d pixel;
        pixel.x() = fx * x_distorted + cx;
        pixel.y() = fy * y_distorted + cy;

        return pixel;
    }

    Eigen::Vector3d unproject(const Eigen::Vector2d& pixel) const override {
        // 像素坐标到归一化坐标
        double mx = (pixel.x() - cx) / fx;
        double my = (pixel.y() - cy) / fy;

        // 逆畸变（迭代法）
        double x = mx;
        double y = my;
        for (int i = 0; i < 5; ++i) {
            double r2 = x*x + y*y;
            double r4 = r2*r2;
            double xy = x*y;
            
            double x_distorted = x * (1 + k1*r2 + k2*r4) + 2*p1*xy + p2*(r2 + 2*x*x);
            double y_distorted = y * (1 + k1*r2 + k2*r4) + p1*(r2 + 2*y*y) + 2*p2*xy;
            
            x -= (x_distorted - mx);
            y -= (y_distorted - my);
        }

        // MEI模型反投影
        double r = std::sqrt(x*x + y*y);
        double denom = r*r + 1;
        double factor = (xi + std::sqrt(1 + (1 - xi*xi)*r*r)) / denom;

        Eigen::Vector3d ray;
        ray.x() = x * factor;
        ray.y() = y * factor;
        ray.z() = factor - xi * std::sqrt(ray.x()*ray.x() + ray.y()*ray.y() + 1);
        
        return ray.normalized();
    }
};
