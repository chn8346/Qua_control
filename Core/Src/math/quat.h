//
// Created by fanchunhui on 2022/11/23.
//

#ifndef QUA_CONTROL_QUAT_H
#define QUA_CONTROL_QUAT_H

// quat 转 旋转矩阵
void quat2RotMatrix(float q[4]);
void quat2RotMatrix(float q1, float q2, float q3, float q4);

// quat 进行坐标旋转
void quat_rot_vector(float q[4], float* vct);
void quat_rot_multi_vector(float q[4], float* vct, int vector_len);

#endif //QUA_CONTROL_QUAT_H
