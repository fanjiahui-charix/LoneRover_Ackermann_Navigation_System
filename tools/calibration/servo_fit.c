#include "servo_fit.h"

#include <stddef.h>

#define SERVO_PWM_MID 1500
#define ARRAY_SIZE(value) ((int)(sizeof(value) / sizeof((value)[0])))

typedef struct
{
  float x0;
  float x1;
  float c0;
  float c1;
  float c2;
  float c3;
} servo_segment_t;

/* y = c0 * (x - x0)^3 + c1 * (x - x0)^2 + c2 * (x - x0) + c3 */
static const servo_segment_t k_left_angle_to_pwm[] = {
  {0.000000f, 0.111701f, 787.525041f, -817.742896f, -813.729849f, 1500.0f},
  {0.111701f, 0.207258f, 24020.106696f, -3127.888935f, -966.937209f, 1400.0f},
  {0.207258f, 0.333707f, -9667.489661f, 2138.998758f, -906.729365f, 1300.0f},
  {0.333707f, 0.448550f, 5424.303418f, -982.082684f, -829.511792f, 1200.0f},
  {0.448550f, 0.571770f, -3153.056509f, 623.108917f, -840.461006f, 1100.0f},
  {0.571770f, 0.689405f, -33540.580791f, 3779.248031f, -830.522474f, 1000.0f},
  {0.689405f, 0.733038f, 298322.361850f, -34973.572767f, -1333.784866f, 900.0f},
};

static const servo_segment_t k_right_angle_to_pwm[] = {
  {-0.513127f, -0.461058f, 567.754811f, -865.037328f, -1877.026588f, 2200.0f},
  {-0.461058f, -0.411200f, -14862.361483f, -125.336977f, -1962.491971f, 2100.0f},
  {-0.411200f, -0.365123f, 290465.961739f, -15216.915444f, -2085.826669f, 2000.0f},
  {-0.365123f, -0.286234f, -44846.729763f, 8234.131928f, -1638.085568f, 1900.0f},
  {-0.286234f, -0.194779f, -13187.127664f, 2111.356212f, -1176.227553f, 1800.0f},
  {-0.194779f, -0.107774f, 19430.153951f, -2017.294767f, -1120.932231f, 1700.0f},
  {-0.107774f, 0.000000f, 1697.073811f, 771.370202f, -1030.712531f, 1600.0f},
};

static const servo_segment_t k_left_pwm_to_angle[] = {
  {800.0f, 900.0f, 0.000000016979f, -0.000005397978f, -0.000066407f, 0.733038f},
  {900.0f, 1000.0f, 0.000000051252f, -0.000010523180f, -0.000637092f, 0.689405f},
  {1000.0f, 1100.0f, 0.000000007193f, -0.000001005076f, -0.001204190f, 0.571770f},
  {1100.0f, 1200.0f, -0.000000009565f, 0.000001360685f, -0.001188629f, 0.448550f},
  {1200.0f, 1300.0f, 0.000000023678f, -0.000002976024f, -0.001203663f, 0.333707f},
  {1300.0f, 1400.0f, -0.000000020740f, 0.000003403700f, -0.001089496f, 0.207258f},
  {1400.0f, 1500.0f, 0.000000000629f, -0.000000932970f, -0.001030091f, 0.111701f},
};

static const servo_segment_t k_right_pwm_to_angle[] = {
  {1500.0f, 1600.0f, 0.000000001107f, 0.000000927738f, -0.001181923f, 0.000000f},
  {1600.0f, 1700.0f, -0.000000011447f, 0.000002072456f, -0.000962698f, -0.107774f},
  {1700.0f, 1800.0f, 0.000000009027f, -0.000001130828f, -0.000891535f, -0.194779f},
  {1800.0f, 1900.0f, 0.000000014894f, -0.000000907450f, -0.000847064f, -0.286234f},
  {1900.0f, 2000.0f, -0.000000013915f, 0.000002601294f, -0.000582108f, -0.365123f},
  {2000.0f, 2100.0f, 0.000000000884f, -0.000000284921f, -0.000478742f, -0.411200f},
  {2100.0f, 2200.0f, 0.000000000024f, -0.000000115333f, -0.000509267f, -0.461058f},
};

static float evaluate(const servo_segment_t *segments, int count, float x)
{
  int index;
  if (x <= segments[0].x0) {
    index = 0;
    x = segments[0].x0;
  } else if (x >= segments[count - 1].x1) {
    index = count - 1;
    x = segments[index].x1;
  } else {
    index = 0;
    while (index + 1 < count && x > segments[index].x1) {
      ++index;
    }
  }

  {
    const float t = x - segments[index].x0;
    return ((segments[index].c0 * t + segments[index].c1) * t +
            segments[index].c2) * t + segments[index].c3;
  }
}

int Servo_AngleToPwm(float angle_rad)
{
  float pwm;
  if (angle_rad > 0.0f) {
    pwm = evaluate(k_left_angle_to_pwm, ARRAY_SIZE(k_left_angle_to_pwm), angle_rad);
  } else if (angle_rad < 0.0f) {
    pwm = evaluate(k_right_angle_to_pwm, ARRAY_SIZE(k_right_angle_to_pwm), angle_rad);
  } else {
    return SERVO_PWM_MID;
  }
  return (int)(pwm + 0.5f);
}

float Servo_PwmToAngle(int servo_pwm)
{
  if (servo_pwm < SERVO_PWM_MID) {
    return evaluate(k_left_pwm_to_angle, ARRAY_SIZE(k_left_pwm_to_angle), (float)servo_pwm);
  }
  if (servo_pwm > SERVO_PWM_MID) {
    return evaluate(k_right_pwm_to_angle, ARRAY_SIZE(k_right_pwm_to_angle), (float)servo_pwm);
  }
  return 0.0f;
}
