/*
 * File: EKF_update.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 14-Nov-2022 22:28:31
 */

/* Include Files */
#include "EKF_update.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 方差进行修正
 *  P = diag(P);
 *
 * Arguments    : const double ui[3]
 *                double yi[6]
 *                double theta[3]
 *                double P[9]
 *                const double Q[3]
 *                const double V[4]
 *                double dt
 *                double altitude[3]
 *                double P_[9]
 * Return Type  : void
 */
void EKF_update(const double ui[3], double yi[6], double theta[3], double P[9],
                const double Q[3], const double V[4], double dt,
                double altitude[3], double P_[9])
{
    static const signed char a[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    static const signed char iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double A[16];
    double b_V[16];
    double H[12];
    double K[12];
    double K_tmp[12];
    double b_H[12];
    double b_Q[9];
    double b_a[9];
    double c_a[9];
    double H_tmp;
    double b_H_tmp;
    double c_H_tmp;
    double d;
    double d1;
    double d2;
    double s;
    double smax;
    double yi_idx_0;
    int b_tmp;
    int i;
    int j;
    int jA;
    int jAcol;
    int jp1j;
    int k;
    int mmj_tmp;
    signed char ipiv[4];
    signed char i1;
    signed char i2;
    signed char i3;
    /* { */
    /*     EKF校正模块 */
    /*     ui -- 输入 */
    /*     yi -- 实际输出 */
    /*     theta -- 更新前的姿态 */
    /*     P  -- 状态方差 */
    /*     Q  -- 输入协方差 */
    /*     V  -- 输出协方差 */
    /*  */
    /*     altitude  -- 姿态 */
    /*     P_ -- 更新后的协方差 */
    /* } */
    memset(&b_Q[0], 0, 9U * sizeof(double));
    b_Q[0] = Q[0];
    b_Q[4] = Q[1];
    b_Q[8] = Q[2];
    memset(&b_V[0], 0, 16U * sizeof(double));
    b_V[0] = V[0];
    b_V[5] = V[1];
    b_V[10] = V[2];
    b_V[15] = V[3];
    /*  预测更新 */
    /*  方差更新 */
    for (i = 0; i < 3; i++) {
        theta[i] += ui[i] * dt * 0.65;
        i1 = a[i];
        i2 = a[i + 3];
        i3 = a[i + 6];
        for (k = 0; k < 3; k++) {
            jA = 3 * k + 1;
            jAcol = 3 * k + 2;
            jp1j = i + 3 * k;
            c_a[jp1j] = ((double)i1 * b_Q[3 * k] + (double)i2 * b_Q[jA]) +
                        (double)i3 * b_Q[jAcol];
            b_a[jp1j] =
                    ((double)i1 * P[3 * k] + (double)i2 * P[jA]) + (double)i3 * P[jAcol];
        }
    }
    for (i = 0; i < 3; i++) {
        d = b_a[i];
        d1 = b_a[i + 3];
        d2 = b_a[i + 6];
        for (k = 0; k < 3; k++) {
            P[i + 3 * k] = (d * (double)iv[3 * k] + d1 * (double)iv[3 * k + 1]) +
                           d2 * (double)iv[3 * k + 2];
        }
        d = c_a[i];
        d1 = c_a[i + 3];
        d2 = c_a[i + 6];
        for (k = 0; k < 3; k++) {
            b_a[i + 3 * k] = (d * (double)iv[3 * k] + d1 * (double)iv[3 * k + 1]) +
                             d2 * (double)iv[3 * k + 2];
        }
    }
    for (i = 0; i < 9; i++) {
        P[i] += b_a[i];
    }
    /*  卡尔曼增益计算 */
    H_tmp = cos(theta[0]);
    smax = cos(theta[1]);
    b_H_tmp = sin(theta[1]);
    H[0] = -H_tmp;
    H[4] = 0.0;
    H[8] = 0.0;
    c_H_tmp = -sin(theta[0]);
    H[1] = c_H_tmp * b_H_tmp;
    H[5] = H_tmp * smax;
    H[9] = 0.0;
    H[2] = c_H_tmp * smax;
    H[6] = -cos(theta[0]) * b_H_tmp;
    H[10] = 0.0;
    H[3] = 0.0;
    H[7] = 0.0;
    H[11] = 1.0;
    for (i = 0; i < 4; i++) {
        K_tmp[3 * i] = H[i];
        K_tmp[3 * i + 1] = H[i + 4];
        K_tmp[3 * i + 2] = H[i + 8];
    }
    for (i = 0; i < 3; i++) {
        d = P[i];
        d1 = P[i + 3];
        d2 = P[i + 6];
        for (k = 0; k < 4; k++) {
            K[i + 3 * k] =
                    (d * K_tmp[3 * k] + d1 * K_tmp[3 * k + 1]) + d2 * K_tmp[3 * k + 2];
        }
    }
    for (i = 0; i < 4; i++) {
        d = H[i];
        d1 = H[i + 4];
        d2 = H[i + 8];
        for (k = 0; k < 3; k++) {
            b_H[i + (k << 2)] =
                    (d * P[3 * k] + d1 * P[3 * k + 1]) + d2 * P[3 * k + 2];
        }
        d = b_H[i];
        d1 = b_H[i + 4];
        d2 = b_H[i + 8];
        for (k = 0; k < 4; k++) {
            jA = i + (k << 2);
            A[jA] =
                    ((d * K_tmp[3 * k] + d1 * K_tmp[3 * k + 1]) + d2 * K_tmp[3 * k + 2]) +
                    b_V[jA];
        }
        ipiv[i] = (signed char)(i + 1);
    }
    for (j = 0; j < 3; j++) {
        mmj_tmp = 2 - j;
        b_tmp = j * 5;
        jp1j = b_tmp + 2;
        jA = 4 - j;
        jAcol = 0;
        smax = fabs(A[b_tmp]);
        for (k = 2; k <= jA; k++) {
            s = fabs(A[(b_tmp + k) - 1]);
            if (s > smax) {
                jAcol = k - 1;
                smax = s;
            }
        }
        if (A[b_tmp + jAcol] != 0.0) {
            if (jAcol != 0) {
                jA = j + jAcol;
                ipiv[j] = (signed char)(jA + 1);
                smax = A[j];
                A[j] = A[jA];
                A[jA] = smax;
                smax = A[j + 4];
                A[j + 4] = A[jA + 4];
                A[jA + 4] = smax;
                smax = A[j + 8];
                A[j + 8] = A[jA + 8];
                A[jA + 8] = smax;
                smax = A[j + 12];
                A[j + 12] = A[jA + 12];
                A[jA + 12] = smax;
            }
            i = (b_tmp - j) + 4;
            for (jA = jp1j; jA <= i; jA++) {
                A[jA - 1] /= A[b_tmp];
            }
        }
        jA = b_tmp;
        for (jAcol = 0; jAcol <= mmj_tmp; jAcol++) {
            smax = A[(b_tmp + (jAcol << 2)) + 4];
            if (smax != 0.0) {
                i = jA + 6;
                k = (jA - j) + 8;
                for (jp1j = i; jp1j <= k; jp1j++) {
                    A[jp1j - 1] += A[((b_tmp + jp1j) - jA) - 5] * -smax;
                }
            }
            jA += 4;
        }
    }
    for (j = 0; j < 4; j++) {
        jA = 3 * j - 1;
        jAcol = j << 2;
        for (k = 0; k < j; k++) {
            jp1j = 3 * k;
            d = A[k + jAcol];
            if (d != 0.0) {
                K[jA + 1] -= d * K[jp1j];
                K[jA + 2] -= d * K[jp1j + 1];
                K[jA + 3] -= d * K[jp1j + 2];
            }
        }
        smax = 1.0 / A[j + jAcol];
        K[jA + 1] *= smax;
        K[jA + 2] *= smax;
        K[jA + 3] *= smax;
    }
    for (j = 3; j >= 0; j--) {
        jA = 3 * j - 1;
        jAcol = (j << 2) - 1;
        i = j + 2;
        for (k = i; k < 5; k++) {
            jp1j = 3 * (k - 1);
            d = A[k + jAcol];
            if (d != 0.0) {
                K[jA + 1] -= d * K[jp1j];
                K[jA + 2] -= d * K[jp1j + 1];
                K[jA + 3] -= d * K[jp1j + 2];
            }
        }
    }
    for (j = 2; j >= 0; j--) {
        i1 = ipiv[j];
        if (i1 != j + 1) {
            smax = K[3 * j];
            jA = 3 * (i1 - 1);
            K[3 * j] = K[jA];
            K[jA] = smax;
            jAcol = 3 * j + 1;
            smax = K[jAcol];
            K[jAcol] = K[jA + 1];
            K[jA + 1] = smax;
            jAcol = 3 * j + 2;
            smax = K[jAcol];
            K[jAcol] = K[jA + 2];
            K[jA + 2] = smax;
        }
    }
    /*  计算传感器数值 */
    yi[3] = atan(yi[3] / yi[4]);
    /*  计算预测值 */
    yi_idx_0 = yi[0] - c_H_tmp;
    smax = yi[1] - H_tmp * b_H_tmp;
    s = yi[2] - cos(theta[0]) * cos(theta[1]);
    c_H_tmp = yi[3] - theta[2];
    /*  更新方差 */
    for (i = 0; i < 3; i++) {
        d = K[i + 3];
        d1 = K[i + 6];
        d2 = K[i + 9];
        altitude[i] =
                theta[i] + (((K[i] * yi_idx_0 + d * smax) + d1 * s) + d2 * c_H_tmp);
        for (k = 0; k < 3; k++) {
            jA = k << 2;
            b_Q[i + 3 * k] =
                    ((K[i] * H[jA] + d * H[jA + 1]) + d1 * H[jA + 2]) + d2 * H[jA + 3];
        }
        d = b_Q[i];
        d1 = b_Q[i + 3];
        d2 = b_Q[i + 6];
        for (k = 0; k < 3; k++) {
            jA = i + 3 * k;
            P_[jA] = P[jA] - ((d * P[3 * k] + d1 * P[3 * k + 1]) + d2 * P[3 * k + 2]);
        }
    }
    /*  欧拉角输出 */
}

/*
 * File trailer for EKF_update.c
 *
 * [EOF]
 */


///*
// * File: EKF_update.c
// *
// * MATLAB Coder version            : 5.2
// * C/C++ source code generated on  : 06-Nov-2022 09:46:30
// */
//
///* Include Files */
//#include "EKF_update.h"
//#include <math.h>
//#include <string.h>
//
//float inv_Sqrt_EKF(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}
//
//void norm_v3(float* v)
//{
//	float d = inv_Sqrt_EKF(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
//	v[0] = v[0] * d;
//	v[1] = v[1] * d;
//	v[2] = v[2] * d;
//}
//
///* Function Definitions */
///*
// * EKF校正模块
// *
// * Arguments    : ui -- 输入        const float ui[3]
// *                yi -- 实际输出    float yi[6]
// *                theta -- 更新前的姿态  float theta[3]
// *                P  -- 状态方差    const float P[3]
// *                Q  -- 输入协方差  const float Q[3]
// *                V  -- 输出协方差  const float V[4]
// *                dt -- 采样时间    float dt
// *
// *                altitude  -- 姿态 float altitude[3]
// *                P_ -- 更新后的协方差 float P_[9]
// * Return Type  : void
// */
//
//void EKF_update(float ui[3], float yi[6], float theta[3],
//							 float P[3], float Q[3], float V[4],
//							 float dt, float altitude[3], float P_[9])
//{
//  static const signed char a[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//  static const signed char iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//  float A[16];
//  float b_V[16];
//  float H[12];
//  float K[12];
//  float K_tmp[12];
//  float b_H[12];
//  float b_P[9];
//  float b_Q[9];
//  float b_a[9];
//  float c_a[9];
//  float H_tmp;
//  float b_H_tmp;
//  float c_H_tmp;
//  float d;
//  float d1;
//  float d2;
//  float s;
//  float smax;
//  float yi_idx_0;
//  int b_tmp;
//  int i;
//  int j;
//  int jA;
//  int jAcol;
//  int jp1j;
//  int k;
//  int mmj_tmp;
//  signed char ipiv[4];
//  signed char i1;
//  signed char i2;
//  signed char i3;
//	float add;
//
//	// 归一化
//	norm_v3(yi);
//
//  memset(&b_P[0], 0, 9U * sizeof(float));
//  b_P[0] = P[0];
//  b_P[4] = P[1];
//  b_P[8] = P[2];
//  memset(&b_Q[0], 0, 9U * sizeof(float));
//  b_Q[0] = Q[0];
//  b_Q[4] = Q[1];
//  b_Q[8] = Q[2];
//  memset(&b_V[0], 0, 16U * sizeof(float));
//  b_V[0] = V[0];
//  b_V[5] = V[1];
//  b_V[10] = V[2];
//  b_V[15] = V[3];
//  /*  预测更新 */
//  /*  方差更新 */
//  for (i = 0; i < 3; i++) {
//    theta[i] += ui[i] * dt * 0.65;
//    i1 = a[i];
//    i2 = a[i + 3];
//    i3 = a[i + 6];
//    for (k = 0; k < 3; k++) {
//      jA = 3 * k + 1;
//      jAcol = 3 * k + 2;
//      jp1j = i + 3 * k;
//      c_a[jp1j] = ((float)i1 * b_Q[3 * k] + (float)i2 * b_Q[jA]) +
//                  (float)i3 * b_Q[jAcol];
//      b_a[jp1j] = ((float)i1 * b_P[3 * k] + (float)i2 * b_P[jA]) +
//                  (float)i3 * b_P[jAcol];
//    }
//  }
//  for (i = 0; i < 3; i++) {
//    d = b_a[i];
//    d1 = b_a[i + 3];
//    d2 = b_a[i + 6];
//    for (k = 0; k < 3; k++) {
//      b_P[i + 3 * k] = (d * (float)iv[3 * k] + d1 * (float)iv[3 * k + 1]) +
//                       d2 * (float)iv[3 * k + 2];
//    }
//    d = c_a[i];
//    d1 = c_a[i + 3];
//    d2 = c_a[i + 6];
//    for (k = 0; k < 3; k++) {
//      b_a[i + 3 * k] = (d * (float)iv[3 * k] + d1 * (float)iv[3 * k + 1]) +
//                       d2 * (float)iv[3 * k + 2];
//    }
//  }
//  for (i = 0; i < 9; i++) {
//    b_P[i] += b_a[i];
//  }
//  /*  卡尔曼增益计算 */
//  H_tmp = cos(theta[0]);
//  smax = cos(theta[1]);
//  b_H_tmp = sin(theta[1]);
//  H[0] = -H_tmp;
//  H[4] = 0.0;
//  H[8] = 0.0;
//  c_H_tmp = -sin(theta[0]);
//  H[1] = c_H_tmp * b_H_tmp;
//  H[5] = H_tmp * smax;
//  H[9] = 0.0;
//  H[2] = c_H_tmp * smax;
//  H[6] = -cos(theta[0]) * b_H_tmp;
//  H[10] = 0.0;
//  H[3] = 0.0;
//  H[7] = 0.0;
//  H[11] = 1.0;
//  for (i = 0; i < 4; i++) {
//    K_tmp[3 * i] = H[i];
//    K_tmp[3 * i + 1] = H[i + 4];
//    K_tmp[3 * i + 2] = H[i + 8];
//  }
//  for (i = 0; i < 3; i++) {
//    d = b_P[i];
//    d1 = b_P[i + 3];
//    d2 = b_P[i + 6];
//    for (k = 0; k < 4; k++) {
//      K[i + 3 * k] =
//          (d * K_tmp[3 * k] + d1 * K_tmp[3 * k + 1]) + d2 * K_tmp[3 * k + 2];
//    }
//  }
//  for (i = 0; i < 4; i++) {
//    d = H[i];
//    d1 = H[i + 4];
//    d2 = H[i + 8];
//    for (k = 0; k < 3; k++) {
//      b_H[i + (k << 2)] =
//          (d * b_P[3 * k] + d1 * b_P[3 * k + 1]) + d2 * b_P[3 * k + 2];
//    }
//    d = b_H[i];
//    d1 = b_H[i + 4];
//    d2 = b_H[i + 8];
//    for (k = 0; k < 4; k++) {
//      jA = i + (k << 2);
//      A[jA] =
//          ((d * K_tmp[3 * k] + d1 * K_tmp[3 * k + 1]) + d2 * K_tmp[3 * k + 2]) +
//          b_V[jA];
//    }
//    ipiv[i] = (signed char)(i + 1);
//  }
//  for (j = 0; j < 3; j++) {
//    mmj_tmp = 2 - j;
//    b_tmp = j * 5;
//    jp1j = b_tmp + 2;
//    jA = 4 - j;
//    jAcol = 0;
//    smax = fabs(A[b_tmp]);
//    for (k = 2; k <= jA; k++) {
//      s = fabs(A[(b_tmp + k) - 1]);
//      if (s > smax) {
//        jAcol = k - 1;
//        smax = s;
//      }
//    }
//    if (A[b_tmp + jAcol] != 0.0) {
//      if (jAcol != 0) {
//        jA = j + jAcol;
//        ipiv[j] = (signed char)(jA + 1);
//        smax = A[j];
//        A[j] = A[jA];
//        A[jA] = smax;
//        smax = A[j + 4];
//        A[j + 4] = A[jA + 4];
//        A[jA + 4] = smax;
//        smax = A[j + 8];
//        A[j + 8] = A[jA + 8];
//        A[jA + 8] = smax;
//        smax = A[j + 12];
//        A[j + 12] = A[jA + 12];
//        A[jA + 12] = smax;
//      }
//      i = (b_tmp - j) + 4;
//      for (jA = jp1j; jA <= i; jA++) {
//        A[jA - 1] /= A[b_tmp];
//      }
//    }
//    jA = b_tmp;
//    for (jAcol = 0; jAcol <= mmj_tmp; jAcol++) {
//      smax = A[(b_tmp + (jAcol << 2)) + 4];
//      if (smax != 0.0) {
//        i = jA + 6;
//        k = (jA - j) + 8;
//        for (jp1j = i; jp1j <= k; jp1j++) {
//          A[jp1j - 1] += A[((b_tmp + jp1j) - jA) - 5] * -smax;
//        }
//      }
//      jA += 4;
//    }
//  }
//  for (j = 0; j < 4; j++) {
//    jA = 3 * j - 1;
//    jAcol = j << 2;
//    for (k = 0; k < j; k++) {
//      jp1j = 3 * k;
//      d = A[k + jAcol];
//      if (d != 0.0) {
//        K[jA + 1] -= d * K[jp1j];
//        K[jA + 2] -= d * K[jp1j + 1];
//        K[jA + 3] -= d * K[jp1j + 2];
//      }
//    }
//    smax = 1.0 / A[j + jAcol];
//    K[jA + 1] *= smax;
//    K[jA + 2] *= smax;
//    K[jA + 3] *= smax;
//  }
//  for (j = 3; j >= 0; j--) {
//    jA = 3 * j - 1;
//    jAcol = (j << 2) - 1;
//    i = j + 2;
//    for (k = i; k < 5; k++) {
//      jp1j = 3 * (k - 1);
//      d = A[k + jAcol];
//      if (d != 0.0) {
//        K[jA + 1] -= d * K[jp1j];
//        K[jA + 2] -= d * K[jp1j + 1];
//        K[jA + 3] -= d * K[jp1j + 2];
//      }
//    }
//  }
//  for (j = 2; j >= 0; j--) {
//    i1 = ipiv[j];
//    if (i1 != j + 1) {
//      smax = K[3 * j];
//      jA = 3 * (i1 - 1);
//      K[3 * j] = K[jA];
//      K[jA] = smax;
//      jAcol = 3 * j + 1;
//      smax = K[jAcol];
//      K[jAcol] = K[jA + 1];
//      K[jA + 1] = smax;
//      jAcol = 3 * j + 2;
//      smax = K[jAcol];
//      K[jAcol] = K[jA + 2];
//      K[jA + 2] = smax;
//    }
//  }
//  /*  计算传感器数值 */
//  yi[3] = atan(yi[3] / yi[4]);
//  /*  计算预测值 */
//  yi_idx_0 = yi[0] - c_H_tmp;
//  smax = yi[1] - H_tmp * b_H_tmp;
//  s = yi[2] - cos(theta[0]) * cos(theta[1]);
//  c_H_tmp = yi[3] - theta[2];
//  /*  更新方差 */
//  for (i = 0; i < 3; i++) {
//    d = K[i + 3];
//    d1 = K[i + 6];
//    d2 = K[i + 9];
//		add = (((K[i] * yi_idx_0 + d * smax) + d1 * s) + d2 * c_H_tmp);
//    altitude[i] =
//        theta[i] + add;
//    for (k = 0; k < 3; k++) {
//      jA = k << 2;
//      b_Q[i + 3 * k] =
//          ((K[i] * H[jA] + d * H[jA + 1]) + d1 * H[jA + 2]) + d2 * H[jA + 3];
//    }
//    d = b_Q[i];
//    d1 = b_Q[i + 3];
//    d2 = b_Q[i + 6];
//    for (k = 0; k < 3; k++) {
//      jA = i + 3 * k;
//      P_[jA] = b_P[jA] -
//               ((d * b_P[3 * k] + d1 * b_P[3 * k + 1]) + d2 * b_P[3 * k + 2]);
//    }
//  }
//  /*  欧拉角输出 */
//}
//
//void EKF_data_redirect(float ui[3], float yi[6], float theta[3], float P[3], float Q[3], float V[4],
//	                     float imu[6], float mag[3], float q0, float q1, float q2, float q3)
//{
//		ui[1] = imu[3]/5.0;
//		ui[2] = imu[4]/5.0;
//		ui[3] = imu[5]/5.0;
//
//		yi[0] = imu[0];
//		yi[1] = imu[1];
//		yi[2] = imu[2];
//		yi[3] = mag[0];
//		yi[4] = mag[1];
//		yi[5] = mag[2];
//
//		//theta[3] = asin(2*(q0*q2-q1*q3))/3.1415*180;
//		//theta[3] = atan(2*(q0*q1+q2*q3)/(1-2*(q1*q1+q2*q2)))/3.1415*180;
//		//theta[3] = atan(2*(q0*q3+q1*q2)/(1-2*(q2*q2+q3*q3)))/3.1415*180;
//
//		P[0] = 0.001;
//		P[1] = 0.001;
//		P[2] = 0.001;
//
//		Q[0] = 0.001;
//		Q[1] = 0.001;
//		Q[2] = 0.001;
//
//		V[0] = 0.01;
//		V[1] = 0.01;
//		V[2] = 0.01;
//		V[3] = 0.01;
//}
//
///*
// * File trailer for EKF_update.c
// *
// * [EOF]
// */
