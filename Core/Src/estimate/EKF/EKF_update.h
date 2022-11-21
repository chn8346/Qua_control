/*
 * File: EKF_update.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 06-Nov-2022 09:46:30
 */

#ifndef EKF_UPDATE_H
#define EKF_UPDATE_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void EKF_update(const double ui[3], double yi[6], double theta[3], double P[9],
                const double Q[3], const double V[4], double dt,
                double altitude[3], double P_[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for EKF_update.h
 *
 * [EOF]
 */
