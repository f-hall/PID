#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}


// Initialize the PID-values
void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    dpp = Kp/3;
    dpi = Ki/3;
    dpd = Kd/3;
}

// update the errors for p-i-d. Counting steps.
void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    if(steps>min_steps) sum_error += pow(cte,2) ;
	steps += 1;
}

// Calculate the summed error
double PID::TotalError() {
    return sum_error/(steps-min_steps);
}

// The getValue function for the pid. Differentiate between steering and speed. (steering has to be between -1 and 1)
double PID::getValue() {
    double steeringAngle = -Kp*p_error -Kd*d_error - Ki*i_error;
    if (steeringAngle < -1){
            steeringAngle = -1;}
    if (steeringAngle > 1){
            steeringAngle = 1;}
    return steeringAngle;
}

double PID::getValue_speed() {
    double steeringAngle = -Kp*p_error -Kd*d_error - Ki*i_error;
    return steeringAngle;
}

// The implementation of twiddle
void PID::twiddle() {
    // Set the error value and reset summed error
    double error = TotalError();
    sum_error = 0;

    // Set twiddle steps and reset step counter
    int steps_twiddle = steps;
    steps = 0;

    // Reset p-i-d errors
    i_error = 0;
	d_error = 0;
	p_error = 0;

	// Only used the first time to set error
    if(best_error > 1e12-1)
    {
        best_error = error;
        //std::cout << "empty" << std::endl;
        return;
    }

    // First step of twiddle
    if(steps_twiddle > 300 && help_1 == 0) {
        if (help_2 == 'p')
            {
            Kp += dpp;
            //std::cout << "dpp_1" << std::endl;
            }
        else if (help_2 == 'i')
        {
            Ki += dpi;
            //std::cout << "dpi_1" << std::endl;
        }
		else
        {
            Kd += dpd;
            //std::cout << "dpd_1" << std::endl;
        }
        help_1 = 1;
		return;
	}

	// Second step of twiddle
	if(help_1 == 1 && steps_twiddle > 300)
    {
        if (help_2 == 'p')
        {
            if (best_error > error)
            {
                best_error = error;
                dpp *= 1.1;
                help_2 = 'i';
                help_1 = 0;
            }
            else
            {
                Kp -= 2*dpp;
                help_1 = 2;
                //std::cout << "dpp_2" << std::endl;
                return;
            }
        }
        else if (help_2 == 'i')
        {
            if (best_error > error)
            {
                best_error = error;
                dpi*= 1.1;
                help_2 = 'd';
                help_1 = 0;
            }
            else
            {
                Ki -= 2*dpi;
                help_1 = 2;
                //std::cout << "dpi_2" << std::endl;
                return;
            }
        }
        else if (help_2 == 'd')
        {
            if (best_error > error)
            {
                best_error = error;
                dpd *= 1.1;
                help_2 = 'p';
                help_1 = 0;
            }
            else
            {
                Kd -= 2*dpd;
                help_1 = 2;
                //std::cout << "dpd_2" << std::endl;
                return;
            }
        }


    }

    // Third step of twiddle
    if(help_1 == 2 && steps_twiddle > 300)
    {
        if (help_2 == 'p')
        {
            if (best_error > error)
            {
                best_error = error;
                dpp *= 1.1;
                help_2 = 'i';
                help_1 = 0;
            }
            else
            {
                Kp += dpp;
                dpp *= 0.8;
                help_2 = 'i';
                help_1 = 0;
                //std::cout << "dpp_3" << std::endl;
                return;
            }
        }
        else if (help_2 == 'i')
        {
            if (best_error > error)
            {
                best_error = error;
                dpi *= 1.1;
                help_2 = 'd';
                help_1 = 0;
            }
            else
            {
                Ki += dpi;
                dpi *= 0.8;
                help_2 = 'd';
                help_1 = 0;
                //std::cout << "dpi_3" << std::endl;
                return;
            }
        }
        else
        {
            if (best_error > error)
            {
                best_error = error;
                dpd *= 1.1;
                help_2 = 'p';
                help_1 = 0;
            }
            else
            {
                Kd += dpd;
                dpd *= 0.8;
                help_2 = 'p';
                help_1 = 0;
                //std::cout << "dpd_3" << std::endl;
                return;
            }
        }
    }


}
