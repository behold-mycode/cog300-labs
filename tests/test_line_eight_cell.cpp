#include <Arduino.h>
#undef ROBOT_EIGHT_CELL_PROFILE
#include "../Robot/LineWheelControl.h"
#include "firmware_fixture.h"
#include <cstdio>
#include <initializer_list>
static float segmentDistance(float x, float y, float ax, float ay, float bx,
                             float by) {
  float dx = bx - ax, dy = by - ay,
        t = wall::limit(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy),
                        0, 1);
  return hypotf(x - ax - t * dx, y - ay - t * dy);
}
int main() {
  int failures=0;
  // Uncalibrated eight-cell hypotheses: actual unscaled motor outputs, not
  // legacy nominal PWM. These models are sensitivity tests, not measurements.
  // Nominal, long coast, fast motor, opposite mismatch, higher friction,
  // and fast motor + long coast. Gains vary independently within each.
  for (int model = 0; model < 6; ++model)
    for (float gain : {.92f, 1.f, 1.08f})
      for (float angleDegrees :
           {45.f, -45.f, 90.f, -90.f, 135.f, -135.f, 150.f, -150.f, 0.f}) {
        setup();
        serviceOdometry();
        lineTapeLevel = 0;
        tape(5);
        startNavigation(LINE);
        assert(mode == LINE && !automaticSetupActive());
        line::Input input;
        float angle = angleDegrees * maze::PI_F / 180, x = 0, y = 0, h = 0,
              vl = 0, vr = 0, l = 0, r = 0;
        float lastL = 0, lastR = 0;
        int moving = 0;
        bool reached = false;
        float firstTurnX = 0, firstTurnY = 0;
        bool turned = false, advanced = false;
        for (int k = 0; k < 16000; ++k) {
          const float dt = .005f;
          auto motor = [&](int pwm, float velocity, float gain) {
            float speed =
                abs(pwm) >= (fabsf(velocity) < 1 ? (model == 4 ? 65 : 50)
                                                 : (model == 4 ? 45 : 30))
                    ? (abs(pwm) - 25) *
                          ((model == 2 || model == 5) ? 7.f : 5.f) * gain
                    : 0;
            if (pwm < 0)
              speed = -speed;
            float acceleration =
                speed ? 1800 : ((model == 1 || model == 5) ? 450 : 900);
            return velocity + wall::limit(speed - velocity, -acceleration * dt,
                                          acceleration * dt);
          };
          vl = motor(motorLeft, vl, (model == 3 ? 1.04f : .96f) * gain);
          vr = motor(motorRight, vr, (model == 3 ? .96f : 1.04f) * gain);
          float dl = vl * dt, dr = vr * dt, dh = (dr - dl) / 130,
                ds = (dl + dr) * .5f;
          x += ds * cosf(h + dh * .5f);
          y += ds * sinf(h + dh * .5f);
          h += dh;
          l += dl;
          r += dr;
          while (fabsf(l - lastL) >= odometry.leftScale) {
            lastL += (l > lastL ? 1 : -1) * odometry.leftScale;
            leftEncoderChanged();
          }
          while (fabsf(r - lastR) >= odometry.rightScale) {
            lastR += (r > lastR ? 1 : -1) * odometry.rightScale;
            rightEncoderChanged();
          }
          nowMs += 5;
          serviceOdometry();
          input.now = nowMs;
          input.pose = odometry.pose;
          input.tape = 0;
          for (int n = 0; n < 3; ++n) {
            float sx = n == 1 ? 150 : 145, sy = (1 - n) * 50;
            float wx = x + sx * cosf(h) - sy * sinf(h),
                  wy = y + sx * sinf(h) + sy * cosf(h);
            if (fminf(segmentDistance(wx, wy, -200, 0, 400, 0),
                      (angleDegrees ? segmentDistance(wx, wy, 400, 0,
                                                      400 + 600 * cosf(angle),
                                                      600 * sinf(angle))
                                    : 10000)) < 25)
              input.tape |= 4 >> n;
          }
          tape(input.tape ^ 7);
          serviceNavigation();
          serviceMotor();
          if (motorLeft || motorRight)
            ++moving;
          if (!strcmp(navigationState, "LINE_CORNER_TURN") && !turned) {
            firstTurnX = x;
            firstTurnY = y;
            turned = true;
          }
          if (!strcmp(navigationState, "LINE_SEARCH_NEW_POSITION")) {
            advanced = true;
            assert(hypotf(x - firstTurnX, y - firstTurnY) > 20);
          }
          if (!angleDegrees && mode == MANUAL) {
            assert(!strcmp(navigationState, "LINE_NOT_FOUND"));
            reached = true;
            break;
          }
          if (angleDegrees &&
              hypotf(x - (400 + 450 * cosf(angle)), y - 450 * sinf(angle)) <
                  40 &&
              maze::angleDistance(h, angle) < .4f) {
            reached = true;
            break;
          }
          if (mode == MANUAL)
            break;
        }
        fprintf(stderr,
                "model=%d angle=%g reached=%d x=%.1f y=%.1f yaw=%.2f phase=%s "
                "moving=%d\n",
                model, angleDegrees, reached, x, y, h, navigationState, moving);
        (void)advanced;
        if(!reached || moving>=4000)++failures;
        stopRobot();
        assert(motorLeft == 0 && motorRight == 0);
      }
  fprintf(stderr,"TOTAL failures=%d\n",failures);
  assert(failures==0);
}
