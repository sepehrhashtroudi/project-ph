/*	
Modified version of http://www.geekfactory.mx
PID library for STM32 microcontrollers.
(Originial version can be found at 
https://github.com/geekfactory/PID)

*** Important note ***
======================

It is assumed that you call
pid_compute() in timer interrupt
callbacks equivalet to the sample
time given to the function.

======================
Developer: Mohammad Hussein Tavakoli Bina
*** mhtb32@gmail.com ***
*** www.mhtb.ir ***
 */
#include "PID.h"

pid_t pid_create(pid_t pid, float* in, float* out, float* set, float kp, float ki, float kd)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
	pid->automode = false;

	pid_limits(pid, 0, 100);

	// Set default sample time to 100 ms
	pid->sampletime = 100;

	pid_direction(pid, E_PID_DIRECT);
	pid_tune(pid, kp, ki, kd);

	return pid;
}

void pid_compute(pid_t pid)
{
	// Check if control is enabled
	if (!pid->automode)
	{
		return;
	}
	float in = *(pid->input);
	// Compute error
	float error = (*(pid->setpoint)) - in;
	// Compute integral
	pid->iterm += (pid->Ki * error);
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	float dinput = in - pid->lastin;
	// Compute PID output
	float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	(*pid->output) = out;
	// Keep track of some variables for next execution
	pid->lastin = in;
}

void pid_tune(pid_t pid, float kp, float ki, float kd)
{
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)
		return;
	
	//Compute sample time in seconds
	float ssec = ((float) pid->sampletime) / 1000;

	pid->Kp = kp;
	pid->Ki = ki * ssec;
	pid->Kd = kd / ssec;

	if (pid->direction == E_PID_REVERSE) {
		pid->Kp = 0 - pid->Kp;
		pid->Ki = 0 - pid->Ki;
		pid->Kd = 0 - pid->Kd;
	}
}

void pid_sample(pid_t pid, uint32_t time)
{
	if (time > 0) {
		float ratio = (float) (time) / (float) pid->sampletime;
		pid->Ki *= ratio;
		pid->Kd /= ratio;
		pid->sampletime = time;
	}
}

void pid_limits(pid_t pid, float min, float max)
{
	if (min >= max) return;
	pid->omin = min;
	pid->omax = max;
	//Adjust output to new limits
	if (pid->automode) {
		if (*(pid->output) > pid->omax)
			*(pid->output) = pid->omax;
		else if (*(pid->output) < pid->omin)
			*(pid->output) = pid->omin;

		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
	}
}

void pid_auto(pid_t pid)
{
	// If going from manual to auto
	if (!pid->automode) {
		pid->iterm = *(pid->output);
		pid->lastin = *(pid->input);
		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
		pid->automode = true;
	}
}

void pid_manual(pid_t pid)
{
	pid->automode = false;
}

void pid_direction(pid_t pid, enum pid_control_directions dir)
{
	if (pid->automode && pid->direction != dir) {
		pid->Kp = (0 - pid->Kp);
		pid->Ki = (0 - pid->Ki);
		pid->Kd = (0 - pid->Kd);
	}
	pid->direction = dir;
}
