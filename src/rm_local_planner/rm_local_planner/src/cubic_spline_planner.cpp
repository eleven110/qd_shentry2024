#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;
using namespace ros;

class CubicSplinePlanner {
private:
    vector<double> x, y;
    double ds;
    vector<double> t;
    MatrixXd A, b;
    VectorXd Cx, Cy;
    vector<Vector3d> coeff;

public:
    CubicSplinePlanner(const vector<double>& _x, const vector<double>& _y, double _ds)
        : x(_x), y(_y), ds(_ds) {
        buildSpline();
    }

    void buildSpline() {
        int n = x.size();
        t.resize(n - 1);
        A.resize(n, n);
        b.resize(n, 3);

        // Calculate the time allocation
        double dx, dy, dist;
        for (int i = 0; i < n - 1; ++i) {
            dx = x[i + 1] - x[i];
            dy = y[i + 1] - y[i];
            dist = sqrt(dx * dx + dy * dy);
            t[i] = dist;
        }

        // Set up the A matrix and b vector
        A(0, 0) = 1.0;
        for (int i = 0; i < n - 2; ++i) {
            A(i + 1, i + 1) = 2.0 * (t[i] + t[i + 1]);
            A(i + 1, i) = t[i];
            A(i, i + 1) = t[i];
        }
        A(n - 2, n - 1) = t[n - 2];
        A(n - 1, n - 2) = t[n - 2];
        A(n - 1, n - 1) = 1.0;

        // Set up the b vector
        b(0, 0) = 0.0;
        b(n - 1, 0) = 0.0;
        for (int i = 0; i < n - 2; ++i) {
            b(i + 1, 0) = 3.0 * ((x[i + 2] - x[i + 1]) / t[i + 1] -
                                 (2.0 * (x[i + 1] - x[i]) / t[i]));
            b(i + 1, 1) = 3.0 * ((y[i + 2] - y[i + 1]) / t[i + 1] -
                                 (2.0 * (y[i + 1] - y[i]) / t[i]));
        }

        // Solve for the coefficients
        Cx = A.lu().solve(b.leftCols(2));
        Cy = A.lu().solve(b.rightCols(2));

        // Calculate the spline coefficients
        for (int i = 0; i < n - 1; ++i) {
            double tbx = (x[i + 1] - x[i]) / t[i] - t[i] * (Cx[i + 1] + 2.0 * Cx[i]) / 3.0;
            double tby = (y[i + 1] - y[i]) / t[i] - t[i] * (Cy[i + 1] + 2.0 * Cy[i]) / 3.0;
            double tdx = (Cx[i + 1] - Cx[i]) / (3.0 * t[i]);
            double tdy = (Cy[i + 1] - Cy[i]) / (3.0 * t[i]);
            Vector3d temp;
            temp << x[i], tbx, Cx[i], tdx, y[i], tby, Cy[i], tdy;
            coeff.push_back(temp);
        }
    }

    Vector3d getState(double time) {
        int i = lower_bound(t.begin(), t.end(), time) - t.begin();
        if (i == t.size()) {
            i--;
        }
        double dt = time - t[i];

        // Calculate position and yaw
        double pos_x = coeff[i][0] + coeff[i][1] * dt + coeff[i][2] * dt * dt + coeff[i][3] * dt * dt * dt;
        double pos_y = coeff[i][4] + coeff[i][5] * dt + coeff[i][6] * dt * dt + coeff[i][7] * dt * dt * dt;
        double dx = coeff[i][1] + 2.0 * coeff[i][2] * dt + 3.0 * coeff[i][3] * dt * dt;
        double dy = coeff[i][5] + 2.0 * coeff[i][6] * dt + 3.0 * coeff[i][7] * dt * dt;
        double yaw = atan2(dy, dx);

        return Vector3d(pos_x, pos_y, yaw);
    }

    nav_msgs::Path getPath() {
        nav_msgs::Path path;
        path.header.frame_id = "map"; // Assuming the path is in the map frame

        // Generate the path
        for (double i = 0; i <= t.back(); i += ds) {
            Vector3d state = getState(i);
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = state(0);
            pose.pose.position.y = state(1);
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(state(2));
            path.poses.push_back(pose);
        }

        return path;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cubic_spline_planner");
    ros::NodeHandle nh;

    // Example usage
    vector<double> x = {0, 1, 2, 3, 4, 5};
    vector<double> y = {0, 1, 4, 9, 16, 25};
    double ds = 0.1;
    CubicSplinePlanner planner(x, y, ds);

    nav_msgs::Path path = planner.getPath();
    // Publish the path or use it as needed

    ros::spin();

    return 0;
}