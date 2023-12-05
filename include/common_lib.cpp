#include "common_lib.h"

M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);

MeasureGroup::MeasureGroup()
{
    lidar_beg_time = 0.0;
    this->lidar.reset(new PointCloudXYZI());
}

StatesGroup::StatesGroup()
{
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
    this->bias_g = Zero3d;
    this->bias_a = Zero3d;
    this->gravity = Zero3d;
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
};

StatesGroup::StatesGroup(const StatesGroup &b)
{
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
};

StatesGroup &StatesGroup::operator=(const StatesGroup &b)
{
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
    return *this;
};

StatesGroup StatesGroup::operator+(const Matrix<double, DIM_STATE, 1> &state_add)
{
    StatesGroup a;
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
    a.cov = this->cov;
    return a;
};

StatesGroup &StatesGroup::operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
{
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->vel_end += state_add.block<3, 1>(6, 0);
    this->bias_g += state_add.block<3, 1>(9, 0);
    this->bias_a += state_add.block<3, 1>(12, 0);
    this->gravity += state_add.block<3, 1>(15, 0);
    return *this;
};

Matrix<double, DIM_STATE, 1> StatesGroup::operator-(const StatesGroup &b)
{
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(15, 0) = this->gravity - b.gravity;
    return a;
};

void StatesGroup::resetpose()
{
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
}

float calc_dist(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}
