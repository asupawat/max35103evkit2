/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#include "global.h"
#include "max3510x_regs.h"
#include "max3510x.h"

#include <math.h>

// this defines values used to program the MAX35103 according to the attached flow body

// the flow body described here is a Audiowell HS0014-000 plastic DN15
// the acoustic path includes these segments:
// 
// 1) From transducer A to the A-side reflector:  APL_TRANSDUCER (acoustic path segiment is perpendicular to flow)
// 2) From the A-side to the B-side reflector:  APL_COLINEAR (acoustic path segment is colinear with flow)
// 3) From the B-side reflector to transducer B:  APL_TRANSDUCER

// these values are estimates

#define APL_TRANSDUCER	0.01032	// 10.32mm - distance between sensor center and reflector center
#define APL_COLINEAR	0.0620	// 62mm - distance between reflector centers
#define APL_RADIUS		(0.0135/2.0)  // 13.5mm - diameter of the acoustic path

#define PI 				3.14159265358979323846

// these constants are specific to the 3-segment flow bodies like the HS0014-000
#define APL_C1	(-6.0*APL_COLINEAR - 8.0*APL_TRANSDUCER)
#define APL_C2	(APL_COLINEAR + 4.0*APL_TRANSDUCER)
#define APL_C3	(2.0*APL_COLINEAR*APL_COLINEAR - 16.0*APL_TRANSDUCER*APL_COLINEAR - 32.0*APL_TRANSDUCER*APL_TRANSDUCER)
#define APL_C4	(APL_COLINEAR*APL_COLINEAR + 8.0*APL_COLINEAR*APL_TRANSDUCER + 16.0*APL_TRANSDUCER*APL_TRANSDUCER)

// these definitions drive the MAX3510x register configuration

#define VCC					3.3
#define HIT_COUNT			6
#define XMIT_FREQ			MAX3510X_REG_TOF1_DPL_1MHZ	
#define RAMP_UP_COUNT		2			//	number of pulses before T1
#define T2_WAVE_COUNT		6			// >= 2
#define HIT_START_CYCLE		T2_WAVE_COUNT+1
#define PULSE_COUNT			HIT_START_CYCLE+HIT_COUNT+RAMP_UP_COUNT

#define STOP_POLARITY		MAX3510X_REG_TOF1_STOP_POL_POS_EDGE
#define BIAS_CHARGE_TIME	MAX3510X_REG_TOF1_CT_61US
#define TIMEOUT				MAX3510X_REG_TOF2_TIMOUT_128US
#define SQUELCH_TIME_US		52.0
#define UPDOWN_CYCLE_TIME	MAX3510X_REG_TOF2_TOF_CYC_0US

#define T1_THRESHOLD		0.050
#define T2_THRESHOLD		0

#define RATE_8XS			8		// must be 8X or 1X (see datasheet)
#define TOF_PERIOD			0.0625	// TOF Difference Measurement Rate (seconds)
#define TEMP_PERIOD			1.0		// Temperature Measurement Rate
#define TOF_COUNT			1
#define TEMP_COUNT			1

#define CALIBRATION			MAX3510X_REG_EVENT_TIMING_2_CAL_USE_DISABLED
#define CALIBRATION_CONFIG	MAX3510X_REG_EVENT_TIMING_2_CAL_CFG_SEQ_SEQ
#define TEMP_PORT_CONFIG	MAX3510X_REG_EVENT_TIMING_2_TP_1_3_2
#define	TEMP_PREAMBLE_COUNT	2
#define TEMP_TO_TEMP_PERIOD	MAX3510X_REG_EVENT_TIMING_2_PORTCYC_128US
#define CMP_EN				MAX3510X_REG_CALIBRATION_CONTROL_CMP_EN_ENABLED
#define CMP_SEL				MAX3510X_REG_CALIBRATION_CONTROL_CMP_SEL_CMP_EN
#define INT_EN				MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_ENABLED
#define ET_CONT				MAX3510X_REG_CALIBRATION_CONTROL_ET_CONT_DISABLED
#define CONT_INT			MAX3510X_REG_CALIBRATION_CONTROL_CONT_INT_DISABLED
#define CLK_S				MAX3510X_REG_CALIBRATION_CONTROL_CLK_S_CONTINUOUS
#define CAL_PERIODS			0

static const max3510x_registers_t s_config =
{
	// MAX3510x register configuration based on board and flow body parameters
  {
	MAX3510X_OPCODE_WRITE_REG(MAX3510X_REG_TOF1),							// address of the first register
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( TOF1_PL, PULSE_COUNT ) |       					// number of pulses transmitted
		MAX3510X_REG_SET( TOF1_DPL, XMIT_FREQ ) |							// pulse frequency
		MAX3510X_REG_SET( TOF1_STOP_POL, STOP_POLARITY ) |					// hit detect edge
		MAX3510X_REG_SET( TOF1_CT, BIAS_CHARGE_TIME )						// stop pin bias charge time
	),
	MAX3510X_ENDIAN( 
		MAX3510X_REG_SET( TOF2_STOP, MAX3510X_REG_TOF2_STOP(HIT_COUNT) ) |	// number of hits to be recorded
		MAX3510X_REG_SET( TOF2_TW2V, T2_WAVE_COUNT ) |						// wave select for t2
		MAX3510X_REG_SET( TOF2_TOF_CYC, UPDOWN_CYCLE_TIME ) |				// TOF_UP to TOF_DOWN time
		MAX3510X_REG_SET( TOF2_TIMOUT, TIMEOUT )								// wave propagation timeout
	),
#if defined(MAX35102)		
		0, 0, 0,
#else		
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( TOF3_HIT1WV, HIT_START_CYCLE ) |					// wave 1-6 hits
		MAX3510X_REG_SET( TOF3_HIT2WV, HIT_START_CYCLE+1 )
	),
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( TOF4_HIT3WV, HIT_START_CYCLE+2 ) |
		MAX3510X_REG_SET( TOF4_HIT4WV, HIT_START_CYCLE+3 )
	),
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( TOF5_HIT5WV, HIT_START_CYCLE+4 ) |
		MAX3510X_REG_SET( TOF5_HIT6WV, HIT_START_CYCLE+5 )
	),
#endif	
	MAX3510X_ENDIAN(
#if !defined(MAX35102)		
		MAX3510X_REG_SET( TOF6_C_OFFSETUPR, MAX3510X_REG_TOF_C_OFFSET_MV(VCC,T2_THRESHOLD) ) | 
#endif		
		MAX3510X_REG_SET( TOF6_C_OFFSETUP, MAX3510X_REG_TOF_C_OFFSET_MV(VCC,T1_THRESHOLD) )
	),
	MAX3510X_ENDIAN(
#if !defined(MAX35102)			
		MAX3510X_REG_SET( TOF7_C_OFFSETDNR, MAX3510X_REG_TOF_C_OFFSET_MV(VCC,T2_THRESHOLD) ) | 
#endif
		MAX3510X_REG_SET( TOF7_C_OFFSETDN,  MAX3510X_REG_TOF_C_OFFSET_MV(VCC,T1_THRESHOLD) )
	),
#if !defined(MAX35102)
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( EVENT_TIMING_1_TDF, MAX3510X_REG_EVENT_TIMING_1_TDF_S(RATE_8XS,TOF_PERIOD) ) |	// TOF_DIFF period
		MAX3510X_REG_SET( EVENT_TIMING_1_TDM, MAX3510X_REG_EVENT_TIMING_1_TDM(TOF_COUNT) ) |				// number of TOF_DIFF measuremnts per event
		MAX3510X_REG_SET( EVENT_TIMING_1_TMF, MAX3510X_REG_EVENT_TIMING_1_TMF_S(RATE_8XS,TEMP_PERIOD) ) |  // temperature measurement period
		MAX3510X_REG_SET( EVENT_TIMING_1_8XS, RATE_8XS/8 )
	),
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( EVENT_TIMING_2_TMM, MAX3510X_REG_EVENT_TIMING_2_TMM_C(TEMP_COUNT)) |				// number of temperature measurements per event
		MAX3510X_REG_SET( EVENT_TIMING_2_CAL_USE, CALIBRATION ) |	
		MAX3510X_REG_SET( EVENT_TIMING_2_CAL_CFG, CALIBRATION_CONFIG ) |
#ifdef MAX35103		
		MAX3510X_REG_SET( EVENT_TIMING_2_TP, TEMP_PORT_CONFIG ) |											// temperatures ports to sample
#endif		
		MAX3510X_REG_SET( EVENT_TIMING_2_PRECYC, TEMP_PREAMBLE_COUNT ) |	// temperature cap absorption cycles
		MAX3510X_REG_SET( EVENT_TIMING_2_PORTCYC, TEMP_TO_TEMP_PERIOD )										// temperature port-to-port time
	),
#endif // 	!defined(MAX35102)
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( TOF_MEASUREMENT_DELAY_DLY, MAX3510X_REG_TOF_MEASUREMENT_DELAY_DLY_US(SQUELCH_TIME_US) )	// squelch detector for this period after start of transmisstion
	),
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( CALIBRATION_CONTROL_CMP_EN, CMP_EN ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_CMP_SEL, CMP_SEL ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_INT_EN, INT_EN ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_ET_CONT, ET_CONT ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_CONT_INT, CONT_INT ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_CLK_S, CLK_S ) |
		MAX3510X_REG_SET( CALIBRATION_CONTROL_CAL_PERIOD, CAL_PERIODS )
	),
	MAX3510X_ENDIAN(
		MAX3510X_REG_SET( RTC_32K_BP, MAX3510X_REG_RTC_32K_BP_DISABLED ) |
		MAX3510X_REG_SET( RTC_32K_EN, MAX3510X_REG_RTC_32K_EN_DISABLED ) |	// do not enable on rev 1.0 boards!
		MAX3510X_REG_SET( RTC_EOSC, MAX3510X_REG_RTC_EOSC_DISABLED ) |
		MAX3510X_REG_SET( RTC_AM, MAX3510X_REG_RTC_AM_NONE ) |
		MAX3510X_REG_SET( RTC_WF, MAX3510X_REG_RTC_WF_CLEAR ) |
		MAX3510X_REG_SET( RTC_WD_EN, MAX3510X_REG_RTC_WD_EN_DISABLED )
	)
  }
};

uint8_t transducer_hit_count( void )
{
	// number of hits that the MAX3510x is configured to capture.
	return HIT_COUNT;
}

const max3510x_registers_t * transducer_config( void )
{
	// returns the transducer configuration array
	return &s_config;
}

void transducer_init(void)
{
}

float_t transducer_sos( float_t up, float_t down)
{
	// returns the speed of sound
	return 0;
}

double_t tranducer_propagation_time( double_t hit_average )
{
	// returns the propagation time given the average hit
	// value returned by the chip

	static const double_t c1 = (5.0/2.0 + RAMP_UP_COUNT+T2_WAVE_COUNT)/1000000.0;
	return hit_average - c1;
}
double_t transducer_flow( double_t up, double_t down )
{
	// returns flow rate through the flow body
	// double precision is required, but some operations
	// may be possible with single precision.

	double flow; // meters per second
	
	double tpu = tranducer_propagation_time(up);
	double tpd = tranducer_propagation_time(down);
	static const double c1 = APL_C1;
	static const double c2 = APL_C2;
	static const double c3 = APL_C3;
	static const double c4 = APL_C4;

	double prod = tpu * tpd;
	double sum = tpu + tpd;
	double diff = tpu - tpd;
	double tpu2 = tpu * tpu;
	double tpd2 = tpd * tpd;
	double sos = tpu2 + tpd2;

	double d = ( 4.0f * prod * diff );
	
	double s = sqrt( c3 * prod + c4 * sos );
	// operations decomposed for future 32-bit float optimization
	double a = c2 * sos;
	double b = sum * s;
	double c = c1 * prod;
	double n =  a + b + c;	
	if( d )
		flow = n / d;
	else
		flow = 0.0; // as d approaches 0 so does the numerator.

	// returns cubic meters per second
	return flow * APL_RADIUS*APL_RADIUS*PI;
}

