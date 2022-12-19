#include <math.h>
#include<vector>
#include<HDU/hduVector.h>
#include<chrono>

class Controller{
    public:
        double P = 0.0;
        double D = 0.0;
        double I = 0.0;

        hduVector3Dd old_error = {0.0, 0.0, 0.0};
        hduVector3Dd total_error = {0.0, 0.0, 0.0};
        
        std::chrono::nanoseconds t_old;
        bool first = true;

        Controller(double p){
            P = p;
        }
        Controller(double p, double d){
            P = p;
            D = d;
        }
        Controller(double p, double d, double i){
            P=p;
            D = d;
            I = i;
        }

        hduVector3Dd pid(hduVector3Dd current_position, hduVector3Dd goal_position){
            hduVector3Dd error = (current_position-goal_position);
            total_error += error;
            hduVector3Dd result;
            auto arg = std::chrono::high_resolution_clock::now();
            std::chrono::nanoseconds t = std::chrono::duration_cast<std::chrono::nanoseconds>(arg);
            if (first == true){
                result = P*error;
                first = false;
            } 
            else
                result = P*error + D/(t-t_old)*(error-old_error) + I*total_error;
            t_old = t;
            old_error = error;
            return result;
        }

};
