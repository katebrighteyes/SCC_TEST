#ifndef SCC_TEST_H
#define SCC_TEST_H

extern double v_set;
extern float default_spacing;
extern double verr_gain;
extern double xerr_gain;
extern double vx_gain;

int calculateTTC(double dist_rel, double rel_vel, float *TTC);

void calculateStopTime(double ego_vel, StopTime *st);

void fcwAebLogicFunc(float TTC, StopTime *st, AEBLogic *aeb);

double rtIntegration(ImuAccel *imu, double size, double step);

double accLogic(double dist_rel, double vel_x, double vrel_x);

double calEgoSpeed(double vel_x, double acceleration, double dTime);


#endif

