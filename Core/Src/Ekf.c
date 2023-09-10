#include "Ekf.h"

void to_quarternion(Attitude_TypeDef *att)
{
    double cr = cos(att->roll * 0.5);
    double sr = sin(att->roll * 0.5);
    double cp = cos(att->pitch * 0.5);
    double sp = sin(att->pitch * 0.5);
    double cy = cos(att->yaw * 0.5);
    double sy = sin(att->yaw * 0.5);

    att->q.w = cr * cp * cy + sr * sp * sy;
    att->q.x = sr * cp * cy - cr * sp * sy;
    att->q.y = cr * sp * cy + sr * cp * sy;
    att->q.z = cr * cp * sy - sr * sp * cy;
}

void to_euler(Attitude_TypeDef *att)
{
    // Roll
    double sinr_cosp = 2*((att->q.w*att->q.x) + (att->q.y*att->q.z));
    double cosr_cosp = 1 - 2*((att->q.x*att->q.x) + (att->q.y*att->q.y));

    att->roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch
    double sinp = sqrt(1 + 2*(att->q.w*att->q.y - att->q.x*att->q.z));
    double cosp = sqrt(1 - 2*(att->q.w*att->q.y - att->q.x*att->q.z));

    att->pitch = 2*atan2(sinp, cosp) - M_PI_2;

    // yaw
    double siny_cosp = 2*(att->q.w*att->q.z + att->q.x*att->q.y);
    double cosy_cosp = 1 - 2*(att->q.y*att->q.y + att->q.z*att->q.z);

    att->yaw = atan2(siny_cosp, cosy_cosp);
}
