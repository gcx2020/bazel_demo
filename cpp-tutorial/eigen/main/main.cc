#include <cmath>
#include <iostream>
// eigen的核心部分
#include <Eigen/Core>
#include <Eigen/Dense>
#define MATRIX_SIZE 100


using namespace Eigen;
using namespace std;

int main0(int argc, char *argv[]) ;

int main(int argc, char *argv[])
{
    // Eigen::MatrixXd m = Eigen::MatrixXd::Random(3,3);
    // m = (m + Eigen::MatrixXd::Constant(3,3,1.2)) * 50;
    // std::cout << "m =" << std::endl << m << std::endl;
    // Eigen::VectorXd v(3);
    // v << 1, 2, 3;
    // std::cout << "m * v =" << std::endl << m * v << std::endl;

    main0(argc,argv);
}


int main0(int argc, char *argv[]) {
    // 向量(列向量)
    Eigen::Vector3d v1(0, 0, 0); // 声明并定义
    v1.y() = 1;
    v1[2] = 2;
    std::cout << "v1: " << v1.transpose() << std::endl;

    Eigen::Vector3d v2;
    v2 << 2, 2, 2; // 先声明后定义
    std::cout << "v2: " << v2.transpose() << std::endl;

    Eigen::Vector3d t;
    t.setZero(); // 各分量设为0
    // t = Eigen::Vector3d::Zero();
    std::cout << "t: " << t.transpose() << std::endl;
    t.setOnes(); // 各分量设为1
    // t = Eigen::Vector3d::Ones();
    std::cout << "t: " << t.transpose() << std::endl;

    // 矩阵
    Eigen::Matrix<double,3,4> M;
    M << 1,0,0,1,
        0,2,0,1,
        0,0,1,1;
    M(1,1) = 1;
    std::cout << "M:\n" << M << std::endl;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    std::cout << "R:\n" << R << std::endl;

    // 变换矩阵(4x4)
    Eigen::Matrix4d T;
    T << R, t, 0, 0, 0, 1;
    std::cout << "T:\n" << T << std::endl;

    // 数学运算
    v2 = R.inverse()*v2 - t;
    std::cout << "v2: " << v2.transpose() << std::endl;
    std::cout << "v2模长: " << v2.norm() << std::endl;
    std::cout << "v2单位向量: " << v2.normalized().transpose() << std::endl;
    std::cout << "v1点乘v2: " << v1.dot(v2) << std::endl;
    std::cout << "v1叉乘v2: " << v1.cross(v2).transpose() << std::endl; // 叉乘只能用于长度为3的向量

    // 块操作
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
    std::cout << "旋转R:\n" << T.topLeftCorner(3, 3) << std::endl;
    std::cout << "平移t: " << T.topRightCorner(3, 1).transpose() << std::endl;

    // 欧式变换矩阵(Isometry)
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity(); // 虽然称为3d，实质上是4x4的矩阵(旋转R+平移t)

    // 旋转部分赋值
    // T1.linear() = Eigen::Matrix3d::Identity();
    // T1.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    // T1.rotate(Eigen::Matrix3d::Identity());
    T1.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)));

    // 平移部分赋值
    // T1.pretranslate(Eigen::Vector3d(1, 1, 1));
    T1.translation() = Eigen::Vector3d(1, 1, 1);

    std::cout << "T1:\n" << T1.matrix() << std::endl; // 输出4x4变换矩阵
    std::cout << "R1:\n" << T1.linear().matrix() << std::endl; // 输出旋转部分
    std::cout << "t1:\n" << T1.translation().transpose() << std::endl; // 输出平移部分

    Eigen::Quaterniond q(T1.linear());
    std::cout << "q: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

    Eigen::Isometry3d T2(q);
    T2(0,3) = 1;
    T2(1,3) = 2;
    T2(2,3) = 3;
    std::cout << "T2:\n" << T2.matrix() << std::endl;

    Eigen::Vector3d v3(1,1,0);
    v3 = T1 * v3; // 相当于R1*v1+t1，隐含齐次坐标(1，1，0，1)
    std::cout << "v3: " << v3.transpose() << std::endl;

    //   // 仿射变换矩阵(Affine3d)
    //   Eigen::Translation3d t;
    //   Eigen::Quaterniond q;
    //   Eigen::Affine3d T = t * q;

    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    // 求解的方程matrix_NN * x = v_Nd
    // 定义matrix_NN矩阵  
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN= MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    // 保证半正定，矩阵和矩阵的转置相乘可以保证半正定
    matrix_NN = matrix_NN * matrix_NN.transpose();  
    // 定义v_Nd
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock(); // 计时
    // 方法一：直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 方法二：通常用矩阵分解来求，例如QR分解，速度会快很多
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of Qr decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 方法三：对于正定矩阵，还可以用cholesky分解来解方程
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of ldlt decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    return 0;
}