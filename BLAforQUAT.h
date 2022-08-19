#ifndef QuaternionForINS_h
#define QuaternionForINS_h

#include "Arduino.h"
#include "BasicLinearAlgebra.h" 
using namespace BLA;

class Quaternion {
    public:
        Quaternion(BLA::Matrix<4,1,Array<4,1,double>>);
        BLA::Matrix<4,1,Array<4,1,double>> Inverse();
        BLA::Matrix<3,1,Array<3,1,double>> Rotate( BLA::Matrix<3,1,Array<3,1,double>> input);
        double Norm2();
        void setQuaternion(BLA::Matrix<4,1,Array<4,1,double>> q);
        BLA::Matrix<4,1,Array<4,1,double>> getQuaternion();
        BLA::Matrix<4,1,Array<4,1,double>> Conjugate();
    private:
        BLA::Matrix<4,1,Array<4,1,double>> _q;
};

#endif

/*
    Usage Example
    Get quaterion Q matrix from the AHRS
    Quaternion RPYtoNED(Q)
    Quaternion NEDtoRPY(RPYtoNED.Inverse())
    gNED = {0,0, 0.0, 9.81}
    gRPY = NEDtoRPY.Rotate(gNED)
    accRPY -= gRPY
    u = RPYtoNED.Rotate(accRPY)
    ...
    x = f*x + b*u
*/