#include <stdio.h>
#include <math.h>

#define PI 3.14159265359

// normalize to [-π, π]
double normalize(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

double Asymmetrical_Gaussian(double x, double y, double x0, double y0, double theta, double v) {
    // define parameter
    double sigmaHead = fmax(2 * v, 1.0);
    double sigmaRear = 2.0 / 7.0;
    double sigmaLarge = 3.0 / 5.0;
    double sigmaSmall = 2.0 / 7.0;

    // compute αmain, αside
    double alphaMain = normalize(atan2(y - y0, x - x0) - theta + PI / 2);
    double alphaSide1 = normalize(alphaMain + PI / 2);
    double alphaSide2 = normalize(alphaMain - PI / 2);

    // determin sigmaMain, sigmaSide
    double sigmaMain, sigmaSide;
    if (alphaMain > 0) {
        sigmaMain = sigmaHead;
    } else {
        sigmaMain = sigmaRear;
    }

    if (alphaSide1 > 0 || alphaSide2 > 0) {
        sigmaSide = sigmaLarge;
    } else {
        sigmaSide = sigmaSmall;
    }

    // determin G,、Gb, Gc
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    double Ga = (cosTheta * cosTheta) / (2 * sigmaMain * sigmaMain) + (sinTheta * sinTheta) / (2 * sigmaSide * sigmaSide);
    double Gb = sin(2 * theta) / (4 * sigmaMain * sigmaMain) - sin(2 * theta) / (4 * sigmaSide * sigmaSide);
    double Gc = (sinTheta * sinTheta) / (2 * sigmaMain * sigmaMain) + (cosTheta * cosTheta) / (2 * sigmaSide * sigmaSide);

    // compute social cost
    double result = exp(-1.0 * (Ga * (x - x0) * (x - x0) + 2 * Gb * (x - x0) * (y - y0) + Gc * (y - y0) * (y - y0)));

    return result;
}

// int main() {
//     // 输入行人信息
//     double x0 = 2.0;
//     double y0 = 3.0;
//     double theta = PI / 4;
//     double v = 1.5;

//     // 计算个人空间函数值
//     double x = 3.0;
//     double y = 4.0;
//     double result = socialAwareness(x, y, x0, y0, theta, v);

//     // 输出结果
//     printf("社交意识的个人空间函数值: %f\n", result);

//     return 0;
// }
