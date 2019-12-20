#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//#define MODEL_CAR_CALI 0.5 //1 //0.3

/*-----------------------------------------*/
/* Automatic Cruise Control (ACC) Controller Parameters */
/*-----------------------------------------*/
double v_set = 20;
double time_gap = 1;
float default_spacing = 5.0;
double verr_gain = 0.5;
double xerr_gain = 0.2;
double vx_gain = 0.1;

/*-----------------------------------------*/
/* AEB */
/*-----------------------------------------*/
double tau1     = 0.5;     // Longitudinal time constant (throttle)          (N/A)
double tau2     = 0.07;    // Longitudinal time constant (brake)             (N/A)
double v_set_aeb = 20;

/*-----------------------------------------*/
/* Speed Controller Parameters */
/*-----------------------------------------*/
double speedController_Kp = 1.1;           // Proportional Gain of speed controller
double speedController_Ki = 0.1;           // Integral Gain of speed controller
double speedController_Amax = 3;           // Maximum acceleration                  (m/s^2)
double speedController_Amin = -3;          // Minimum acceleration                  (m/s^2)

/*-----------------------------------------*/
/* FCW parameters */
/*-----------------------------------------*/
double FCW_timeToReact  = 1.2;         // driver reaction time                   (sec)
double FCW_driver_decel = 4.0;         // driver braking deceleration            (m/s^2)

/*-----------------------------------------*/
/* AEB controller parameters */
/*-----------------------------------------*/

double AEB_PB1_decel = (3.8);            // 1st stage Partial Braking deceleration (m/s^2)
double AEB_PB2_decel = (5.3);            // 2nd stage Partial Braking deceleration (m/s^2)
double AEB_FB_decel  = (9.8);            // Full Braking deceleration              (m/s^2)
double AEB_headwayOffset = 3.7;        // headway offset                         (m)
double AEB_timeMargin = 0;             // headway time margin                    (sec)


#define ST_DEFAULT	0
#define ST_FCW 		1
#define ST_Patial_Breaking1 		2
#define ST_Patial_Breaking2 		3
#define ST_Full_Breaking 		3

static int sEgoCarStop;

typedef struct StopTime {
	float fcw_stoptime;
	float pb1stoptime;
	float pb2stoptime;
	float fb_stoptime;
};

typedef struct AEBLogic {
	int fcw_active;
	int aeb_Status;
	float decel;
};

typedef struct ImuAccel {
	float xval;
	float yval;
};


int calculateTTC(double dist_rel, double rel_vel, float *TTC)
{
	int collision = 0;
	double TTC_vel = 0;
	//const double max_vel = 100; // -> need to fix
	const double max_vel = 50;
	const double min_vel = 0.01;

	double dist_rel2 = dist_rel-AEB_headwayOffset;
	
	if (dist_rel2 < 0.1)
		collision = 1;
	else {
		double rel_vel2 = fabs(rel_vel);
		rel_vel2 = (rel_vel2 > max_vel)? max_vel : rel_vel2;
		rel_vel2 = (rel_vel2 < min_vel)? min_vel : rel_vel2;
		TTC_vel = dist_rel2 / rel_vel2;
		*TTC = TTC_vel;
	}
	return collision;
}

void calculateStopTime(double ego_vel, StopTime *st)
{
	st->fcw_stoptime = (ego_vel / FCW_driver_decel) + FCW_timeToReact;
	st->pb1stoptime = (ego_vel / AEB_PB1_decel) + AEB_timeMargin;
	st->pb2stoptime = (ego_vel / AEB_PB2_decel) + AEB_timeMargin;
	st->fb_stoptime= (ego_vel / AEB_FB_decel) + AEB_timeMargin;
}

void fcwAebLogicFunc(float TTC, StopTime *st, AEBLogic *aeb)
{
	static int stFcwStatus;
	float ttc_val = fabs(TTC);

	if(stFcwStatus == ST_DEFAULT) {
		if(ttc_val < st->fcw_stoptime) {
			stFcwStatus = ST_FCW;
		}	
	}
	else if(stFcwStatus ==ST_FCW) {
		if(ttc_val < st->pb1stoptime) {
			stFcwStatus = ST_Patial_Breaking1;
		}
		else if(ttc_val >= (1.2*st->fcw_stoptime) ) {
			stFcwStatus = ST_DEFAULT;
		}		
	}
	else if(stFcwStatus ==ST_Patial_Breaking1) {
		if(sEgoCarStop == 1) {
			stFcwStatus = ST_DEFAULT;
		} 
		else {
			if(ttc_val< st->pb1stoptime) {
				stFcwStatus = ST_Patial_Breaking2;
			}
			else if(ttc_val >= (1.2*st->pb1stoptime) ) {
				stFcwStatus = ST_FCW;
			}
		}

	}
	else if(stFcwStatus ==ST_Patial_Breaking2) {
		if(sEgoCarStop == 1) {
			stFcwStatus = ST_DEFAULT;
		} 
		else {				
			if(ttc_val< st->pb2stoptime) {
				stFcwStatus = ST_Full_Breaking;
			}
			else if(ttc_val >= (1.2*st->pb2stoptime) ) {
				stFcwStatus = ST_Patial_Breaking1;
			}
		}
	}	
	else if(stFcwStatus ==ST_Full_Breaking) {
		if(sEgoCarStop == 1)
		{
			stFcwStatus = ST_DEFAULT;
		} else {	
			if(ttc_val >= (1.2*st->fb_stoptime) ) {
				stFcwStatus = ST_Patial_Breaking2;
			}	
		}
	}

	switch(stFcwStatus) {
	case ST_DEFAULT:
		aeb.aeb_Status = 0;
		aeb.fcw_active = 0;
		aeb.decel = 0;			
		break;
	case ST_FCW:
		aeb.aeb_Status = 0;
		aeb.fcw_active = 1;
		aeb.decel = 0;			
		break;
	case ST_Patial_Breaking1:
		aeb.aeb_Status = 1;
		aeb.fcw_active = 1;
		aeb.decel = AEB_PB1_decel;	//1.9		
		break;
	case ST_Patial_Breaking2:
		aeb.aeb_Status = 2;
		aeb.fcw_active = 1;
		aeb.decel = AEB_PB2_decel;	//2.9		
		break;
	case ST_Full_Breaking:
		aeb.aeb_Status = 3;
		aeb.fcw_active = 1;
		aeb.decel = AEB_FB_decel;	//4.9		
		break;	
	default:
		aeb.aeb_Status = 0;
		aeb.fcw_active = 0;
		aeb.decel = 0;			
		break;		
	}

}

double rtIntegration(ImuAccel *imu, double size, double step)
{
    double previous = 0;
    double res = 0;
    double semiStep = step / 2;

    for (int x = 1; x < size; x += step)
    {
        res += semiStep * (imu[previous] + imu[x]);
        previous = x;
    }

    res += (size - previous) / 2 * (imu[x] + imu[previous]);
    return res;
}



double accLogic(double dist_rel, double vel_x, double vrel_x)
{
	double dist_safe;
	double acceleration;
	
	dist_safe = default_spacing + time_gap*vel_x;
	
	if(dist_safe >dist_rel) //spacing control
	{
		acceleration = (v_set - vel_x)* verr_gain;
	}
	else //speed_control
	{
		double val1 = vrel_x * vx_gain- (dist_safe- dist_rel)*xerr_gain;
		double val2 = (v_set - vel_x)* verr_gain;

		acceleration = (val1 > val2)? val2 : val1;

	}
	return acceleration;	
	
}

double calEgoSpeed(double vel_x, double acceleration, double dTime)
{
	return (vel_x + acceleration * dTime);
}



