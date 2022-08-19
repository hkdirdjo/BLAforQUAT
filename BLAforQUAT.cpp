#include "Arduino.h"
#include "QuaternionForINS.h"

Quaternion::Quaternion( BLA::Matrix<4,1,Array<4,1,double>> q ) {
    _q = q;
}

// Returns the inverse of the instance's quaternion
BLA::Matrix<4,1,Array<4,1,double>> Quaternion::Inverse() {
    BLA::Matrix<4,1,Array<4,1,double>> temp = this->Conjugate();
    double _norm2 = this->Norm2();
    for (int i = 0; i <= 3; i++)
    {
        temp(i) /= _norm2;
    }
    return temp;
}
// Returns the input 3D vector rotated by the instance's quaternion
BLA::Matrix<3,1,Array<3,1,double>> Quaternion::Rotate( BLA::Matrix<3,1,Array<3,1,double>> input) {
    BLA::Matrix<3,1,Array<3,1,double>> output;

    // Using formula provided here; https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf
    double q0q0 = _q(0) * _q(0);
    double q0q1 = _q(0) * _q(1);
    double q0q2 = _q(0) * _q(2);
    double q0q3 = _q(0) * _q(3);
    double q1q1 = _q(1) * _q(1);
    double q1q2 = _q(1) * _q(2);
    double q1q3 = _q(1) * _q(3);
    double q2q2 = _q(2) * _q(2);
    double q2q3 = _q(2) * _q(3);
    double q3q3 = _q(3) * _q(3);

    output(0) =   input(0)*(q0q0+q1q1-q2q2-q3q3) + 2*input(1)*(q1q2-q0q3)           + 2*input(2)*(q0q2+q1q3)          ;
    output(1) = 2*input(0)*(q0q3+q1q2)           +   input(1)*(q0q0-q1q1+q2q2-q3q3) + 2*input(2)*(q2q3-q0q1)          ;
    output(2) = 2*input(0)*(q1q3-q0q2)           + 2*input(1)*(q0q1+q2q3)           +   input(2)*(q0q0-q1q1-q2q2+q3q3);
    return output;
}

// Returns the magnitude of the instance's quaternion
double Quaternion::Norm2() {
    return _q(0)*_q(0) + _q(1)*_q(1) + _q(2)*_q(2) + _q(3)*_q(3);
}

// Returns the conjugate of the instance's quaternion
BLA::Matrix<4,1,Array<4,1,double>> Quaternion::Conjugate() {
    BLA::Matrix<4,1,Array<4,1,double>> temp = _q;
    for (int i = 1; i <= 3; i++)
    {
        temp(i) *= -1;
    }
    return temp;
}

void Quaternion::setQuaternion(BLA::Matrix<4,1,Array<4,1,double>> q) {
    _q = q;
}

BLA::Matrix<4,1,Array<4,1,double>> Quaternion::getQuaternion() {
    return _q;
}
