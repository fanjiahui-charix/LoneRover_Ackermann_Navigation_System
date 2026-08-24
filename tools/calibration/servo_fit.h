#ifndef LONEROVER_SERVO_FIT_H
#define LONEROVER_SERVO_FIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Calibrated for the published RDK X5 vehicle profile. */
int Servo_AngleToPwm(float angle_rad);
float Servo_PwmToAngle(int servo_pwm);

#ifdef __cplusplus
}
#endif

#endif  /* LONEROVER_SERVO_FIT_H */
