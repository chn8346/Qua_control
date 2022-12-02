//
// Created by fanchunhui on 2022/11/22.
//

#include "pos_velo_estimate_testv1.h"

void int2velo(float* in, float* out){
    out[0] = out[0] + ((in[0]>0.1||in[0]<-0.1)?in[0]:0.0);
    out[1] = out[1] + ((in[1]>0.1||in[1]<-0.1)?in[1]:0.0);
    out[2] = out[2] + ((in[2]-1>0.1||in[2]-1<-0.1)?in[2]-1:0.0);
}

void int2pos(float* in, float* out){
    out[0] = out[0] + in[0]*DELTA_T_float;
    out[1] = out[1] + in[1]*DELTA_T_float;
    out[2] = out[2] + in[2]*DELTA_T_float;
}
