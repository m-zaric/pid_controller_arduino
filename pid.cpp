#include <math.h>
#include<vector>
#include<HDU/hduVector.h>

class Controller{
    public:
        double P = 0.0;
        double D = 0.0;
        double I = 0.0;

        hduVector3Dd old_error = {0.0, 0.0, 0.0};
        hduVector3Dd total_error = {0.0, 0.0, 0.0};;

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
            hduVector3Dd result = P*error + D*(error-old_error) + I*total_error;
            old_error = error;
            return result;
        }

};
