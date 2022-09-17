/**********************************************************************/
// File Name: {FM_project_loc}/../Sources/Config/PMSM_appconfig.h 
//
// Date:  14. March, 2017
//
// Automatically generated file by MCAT
// - static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __PMSM_APPCONFIG_H
#define __PMSM_APPCONFIG_H


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 4 [-]
//Stator resistance                     = 0.1498 [Ohms]
//Direct axis inductance                = 0.000131 [H]
//Quadrature axis inductance            = 0.000131 [H]
//Back-EMF constant                     = 0.001769 [V.sec/rad]
//Drive inertia                         = 0.5e-6 [kg.m2]
//Nominal current                       = 5.8 [A]

#define MOTOR_PP                        (4.0F)
#define MOTOR_PP_GAIN                   FRAC32(0.5)
#define MOTOR_PP_SHIFT                  (3)

//Application scales                    
//----------------------------------------------------------------------
#define I_MAX                           (31.25F)
#define U_DCB_MAX                       (45.0F)
#define U_MAX                           (26.5F)
#define N_MAX                           (10000.0F)
#define WEL_MAX                         (4188.79F)
#define FREQ_MAX                        (667.0F)
#define N_NOM                           (9350.0F)
#define I_PH_NOM                        (5.8F)
#define E_MAX                           (26.5F)

//Application Fault Triggers            
//----------------------------------------------------------------------
#define U_DCB_TRIP                      (16.0F)
#define U_DCB_UNDERVOLTAGE              (8.0F)
#define U_DCB_OVERVOLTAGE               (18.0F)
#define I_PH_OVER                       (9.3F)
#define TEMP_OVER                       (110.0F)
//DCB voltage IIR1 filter, 100Hz cut-off freq. @ Ts= 0.0001 [sec]
#define UDCB_IIR_B0                     (0.030459027951F)
#define UDCB_IIR_B1                     (0.030459027951F)
#define UDCB_IIR_A1                     (-0.939081944097F)
//Mechanical Alignment                  
#define ALIGN_VOLTAGE                   (2.0F)
#define ALIGN_DURATION                  (20000)

//Current Loop Control                  
//----------------------------------------------------------------------
//Loop Bandwidth                        = 200 [Hz]
//Loop Attenuation                      = 0.9 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Current Controller Output Limit       
#define CLOOP_LIMIT                     (0.9F)
//D-axis Controller - Recurrent type    
#define D_CC1SC                         (0.15685836449893098F)
#define D_CC2SC                         (-0.13617167367424765F)
//Q-axis Controller - Recurrent type    
#define Q_CC1SC                         (0.15685836449893098F)
#define Q_CC2SC                         (-0.13617167367424765F)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop Bandwidth                        = 10 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.001 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define SPEED_PI_PROP_GAIN              (0.005919714817391733F)
#define SPEED_PI_INTEG_GAIN             (0.00009298666290832256F)
#define SPEED_LOOP_HIGH_LIMIT           (5.8F)
#define SPEED_LOOP_LOW_LIMIT            (-5.8F)

#define SPEED_RAMP_UP                   (0.83776F)
#define SPEED_RAMP_DOWN                 (0.83776F)

#define SPEED_LOOP_CNTR                 (10)
#define SPEED_LOOP_FREQ                 (1000)

#define POSPE_SPEED_FILTER_MA_LAMBDA    (0.8F)

//Sensorless DQ BEMF Observer and Tracking Observer
//----------------------------------------------------------------------
//Loop Bandwidth                        = 200 [Hz]
//Loop Attenuation                      = 0.9 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//DQ Bemf - plant coefficients          
#define I_Gain                          (0.89183334536789660F)
#define U_Gain                          (0.36103689797097260F)
#define E_Gain                          (0.36103689797097260F)
#define WI_Gain                         (0.000047295833634197419F)

//DQ Bemf - PI controller parameters    
#define BEMF_DQ_CC1_GAIN                (0.15685836449893098F)
#define BEMF_DQ_CC2_GAIN                (-0.13617167367424765F)

//Tracking Observer - PI controller parameters
#define TO_CC1SC                        (322.21897945835496F)
#define TO_CC2SC                        (-318.6659218739628F)
//Tracking Observer - Integrator        
#define TO_THETA_GAIN                   (0.00005F)

//Observer speed output filter          
#define TO_SPEED_IIR_B0                 (0.013954401463F)
#define TO_SPEED_IIR_B1                 (0.013954401463F)
#define TO_SPEED_IIR_A1                 (-0.097091197074F)

//Open loop start-up                    
#define OL_START_RAMP_INC               (0.12566F)
#define OL_START_I                      (1.0F)
#define MERG_SPEED_1_TRH                (300.0F)
#define MERG_SPEED_2_TRH                (500.0F)

//Control Structure Module - Scalar Control
//----------------------------------------------------------------------
#define SCALAR_VHZ_FACTOR_GAIN          (0.004340589357051691F)
#define SCALAR_INTEG_GAIN               FRAC32(0.066666666667)
#define SCALAR_INTEG_SHIFT              (0)
#define SCALAR_RAMP_UP                  (0.08378F)
#define SCALAR_RAMP_DOWN                (0.08378F)

//FreeMASTER Scale Variables            
//----------------------------------------------------------------------
//Note: Scaled at input = 1000          
//----------------------------------------------------------------------
#define FM_SPEED_RAD_MEC_SCALE          (1000)
#define FM_SPEED_RPM_EL_SCALE           (9549)
#define FM_SPEED_RPM_MEC_SCALE          (2387)
#define FM_POSITION_DEG_SCALE           (57296)

#endif

//End of generated file                 
/**********************************************************************/
