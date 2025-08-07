class ScaramuzzaZhangCamera : public FisheyeCamera {
private:
    double fx, fy; // 焦距（像素单位）
    double cx, cy; // 主点坐标
    double a0, a2, a3, a4; // 多项式畸变参数

public:
    ScaramuzzaZhangCamera(double fx_, double fy_, double cx_, double cy_, 
                          double a0_, double a2_, double a3_, double a4_)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_), 
          a0(a0_), a2(a2_), a3(a3_), a4(a4_) {}

    Eigen::Vector2d project(const Eigen::Vector3d& point3d) const override {
        if (point3d.z() <= 0) {
            throw std::runtime_error("Point is behind camera");
        }

        // 归一化坐标
        double X = point3d.x() / point3d.z();
        double Y = point3d.y() / point3d.z();

        // SZ模型投影
        double r = std::sqrt(X*X + Y*Y);
        double theta = std::atan(r);
        
        // 多项式畸变
        double rho = a0 + a2*theta*theta + a3*std::pow(theta, 3) + a4*std::pow(theta, 4);
        
        double factor = rho / r;
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

        // SZ模型逆畸变
        double rho = std::sqrt(mx*mx + my*my);
        double theta = rho;
        for (int i = 0; i < 10; ++i) {
            double theta2 = theta * theta;
            double rho_approx = a0 + a2*theta2 + a3*theta2*theta + a4*theta2*theta2;
            double deriv = 2*a2*theta + 3*a3*theta2 + 4*a4*theta2*theta;
            theta -= (rho_approx - rho) / deriv;
            if (std::abs(rho_approx - rho) < 1e-8) break;
        }

        double factor = std::tan(theta) / rho;
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

