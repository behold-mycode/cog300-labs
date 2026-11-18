#include "firmware_fixture.h"
#include <cstdio>
#include <initializer_list>
#include <utility>
#include <vector>
using Point = std::pair<float, float>;
static float distance(float x, float y, Point a, Point b) {
  float dx = b.first - a.first, dy = b.second - a.second;
  float t = wall::limit(
      ((x - a.first) * dx + (y - a.second) * dy) / (dx * dx + dy * dy), 0, 1);
  return hypotf(x - a.first - t * dx, y - a.second - t * dy);
}
// Outcomes on courses with a gap, opposite bends and initial displacement.
// Real navigation/PWM/reversal/encoder code runs; only floor and motor are
// models.
int main() {
  for (float heading : {-.12f, -.06f, 0.f, .06f, .12f})
    for (int mirror : {-1, 1})
      for (int course = 0; course < 3; ++course)
        for (int variant = 0; variant < 4; ++variant) {
          float startY = variant == 1 ? 15 : variant == 2 ? -15 : 0;
          if (fabsf(startY + 150 * sinf(heading)) >= 25)
            continue;
          setup();
          serviceOdometry();
          lineTapeLevel = 0;
          tape(5);
          startNavigation(LINE);
          std::vector<Point> points =
              course == 0
                  ? std::vector<Point>{{-200, 0}, {1200, 0}}
                  : std::vector<Point>{{-200, 0},
                                       {400, 0},
                                       {400, course == 1 ? 350.f : 220.f},
                                       {1000, course == 1 ? 350.f : 220.f}};
          for (auto &point : points)
            point.second *= mirror;
          const Point goal = {800, points.back().second};
          float x = 0,
                y = variant == 1   ? 15
                    : variant == 2 ? -15
                                   : 0,
                h = heading;
          float vl = 0, vr = 0, l = 0, r = 0, lastL = 0, lastR = 0;
          bool reached = false;
          int steps = 0;
          float largestHeading = 0;
          for (; steps < 5000; ++steps) {
            auto motor = [&](int pwm, float v, float gain) {
              float speed = abs(pwm) >= (fabsf(v) < 1 ? 220 : 200)
                                ? (abs(pwm) - 180) * 4.f * gain
                                : 0;
              if (pwm < 0)
                speed = -speed;
              float accel = speed ? 1800 : (variant == 3 ? 450 : 900);
              return v + wall::limit(speed - v, -accel * .005f, accel * .005f);
            };
            vl = motor(motorLeft, vl, .96f);
            vr = motor(motorRight, vr, 1.04f);
            float dl = vl * .005f, dr = vr * .005f,
                  dh = (dr - dl) / (variant == 3 ? 135.f : 130.f),
                  ds = (dl + dr) * .5f;
            x += ds * cosf(h + dh * .5f);
            y += ds * sinf(h + dh * .5f);
            h += dh;
            l += dl;
            r += dr;
            largestHeading = fmaxf(largestHeading, fabsf(h));
            const float edge =
                odometry.leftScale * (variant == 3 ? 1.03f : 1.f);
            while (fabsf(l - lastL) >= edge) {
              lastL += (l > lastL ? 1 : -1) * edge;
              leftEncoderChanged();
            }
            while (fabsf(r - lastR) >= edge) {
              lastR += (r > lastR ? 1 : -1) * edge;
              rightEncoderChanged();
            }
            int mask = 0;
            for (int n = 0; n < 3; ++n) {
              float sx = n == 1 ? 150 : 145, sy = (1 - n) * 50;
              float wx = x + sx * cosf(h) - sy * sinf(h),
                    wy = y + sx * sinf(h) + sy * cosf(h);
              bool hit = false;
              for (unsigned j = 1; j < points.size(); ++j)
                hit |= distance(wx, wy, points[j - 1], points[j]) < 25;
              if (course == 0 && wx > 250 && wx < 290)
                hit = false;
              if (hit)
                mask |= 4 >> n;
            }
            tape(mask ^ 7);
            nowMs += 5;
            serviceOdometry();
            serviceNavigation();
            serviceMotor();
            if (hypotf(x - goal.first, y - goal.second) < 45 &&
                maze::angleDistance(h, 0) < .35f) {
              reached = true;
              break;
            }
            if (mode == MANUAL)
              break;
          }
          fprintf(stderr,
                  "heading=%.2f mirror=%d course=%d variant=%d reached=%d "
                  "time=%.2f pose=%.1f,%.1f,%.2f "
                  "phase=%s\n",
                  heading, mirror, course, variant, reached, steps * .005f, x,
                  y, h, navigationState);
          assert(reached);
          assert(largestHeading < 3.f);
          stopRobot();
        }
}
