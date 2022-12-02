//
// Created by fanchunhui on 2022/11/23.
//

#include "quat.h"

void quat2RotMatrix(float q[4]){

}

void quat2RotMatrix(float q1, float q2, float q3, float q4){

}

void quat_rot_vector(float q[4], float* vct){
    float qr[] = {1-2*q[2]*q[2]-2*q[3]*q[3], 2*(q[1]*q[2]-q[0]*q[3]),   2*(q[1]*q[3]+q[0]*q[2]),
                  2*(q[1]*q[2]+q[0]*q[3]),   1-2*q[1]*q[1]-2*q[3]*q[3], 2*(q[2]*q[3]-q[0]*q[1]),
                  2*(q[1]*q[3]-q[0]*q[2]),   2*(q[2]*q[3]+q[0]*q[1]),   1-2*q[1]*q[1]-2*q[2]*q[2]};

    float new_v[3] = {vct[0]*qr[0] + vct[1]*qr[1] + vct[2]*qr[2],
                      vct[0]*qr[3] + vct[1]*qr[4] + vct[2]*qr[5],
                      vct[0]*qr[6] + vct[1]*qr[7] + vct[2]*qr[8]};

    vct[0] = new_v[0];
    vct[1] = new_v[1];
    vct[2] = new_v[2];
}

void quat_rot_multi_vector(float *q, float* vct, int vector_len){

}


