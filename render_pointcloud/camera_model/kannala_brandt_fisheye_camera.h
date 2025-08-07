class KannalaBrandtCamera : public FisheyeCamera {
private:
    double fx, fy; // 焦距（像素单位）
    double cx, cy; // 主点坐标
    double k1, k2, k3, k4; // 径向畸变参数

public:
    KannalaBrandtCamera(double fx_, double fy_, double cx_, double cy_, 
                        double k1_, double k2_, double k3_, double k4_)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_), 
          k1(k1_), k2(k2_), k3(k3_), k4(k4_) {}

    Eigen::Vector2d project(const Eigen::Vector3d& point3d) const override {
        if (point3d.z() <= 0) {
            throw std::runtime_error("Point is behind camera");
        }

        // 归一化坐标
        double X = point3d.x() / point3d.z();
        double Y = point3d.y() / point3d.z();

        // KB模型投影
        double r = std::sqrt(X*X + Y*Y);
        double theta = std::atan(r);
        
        // 径向畸变
        double theta_d = theta * (1 + k1*theta*theta + k2*std::pow(theta, 4) + 
                                k3*std::pow(theta, 6) + k4*std::pow(theta, 8));
        
        double factor = theta_d / r;
        double x = X * factor;
        double y = Y * factor;

        // 投影到像素坐标
        Eigen::Vector2d pixel;
        pixel.x() = fx * x + cx;
        pixel.y() = fy * y + cy;

        return pixel;
    }

    Eigen::Vector3d unproject(const Eigen::Vector2d& pixel) const override {
        // 像素坐标到归一化坐标
        double mx = (pixel.x() - cx) / fx;
        double my = (pixel.y() - cy) / fy;

        // KB模型逆畸变
        double r_d = std::sqrt(mx*mx + my*my);
        double theta_d = r_d;
        for (int i = 0; i < 10; ++i) {
            double theta_d_approx = theta_d * (1 + k1*theta_d*theta_d + 
                                            k2*std::pow(theta_d, 4) + 
                                            k3*std::pow(theta_d, 6) + 
                                            k4*std::pow(theta_d, 8));
            double deriv = 1 + 3*k1*theta_d*theta_d + 
                          5*k2*std::pow(theta_d, 4) + 
                          7*k3*std::pow(theta_d, 6) + 
                          9*k4*std::pow(theta_d, 8);
            theta_d -= (theta_d_approx - r_d) / deriv;
            if (std::abs(theta_d_approx - r_d) < 1e-8) break;
        }

        double theta = theta_d;
        double factor = std::tan(theta) / r_d;
        double x = mx * factor;
        double y = my * factor;

        // 反投影到3D单位射线
        Eigen::Vector3d ray;
        ray.x() = x;
        ray.y() = y;
        ray.z() = 1.0;
        return ray.normalized();
    }
};
