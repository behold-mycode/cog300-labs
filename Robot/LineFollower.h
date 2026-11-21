#pragma once
#include "Odometry.h"

namespace line {
struct Input {
  uint32_t now = 0;
  maze::Pose pose;
  uint8_t tape =
      0; // normalized L/C/R: bit set means tape, independent of polarity
  float leftSpeed = 0, rightSpeed = 0;
  uint32_t edgeSequence=0;
  int appliedLeft = 0, appliedRight = 0;
};
// Physical wheel velocities, mm/s. A tight arc can explicitly brake its stationary inside wheel.
struct Command {
  float left = 0, right = 0;
  bool brakeInside=false;
};
struct Config {
  float cruiseSpeed = 250, turnSpeed = 250, correctionSpeed = 250,
        captureSpeed = 125;
};
class Follower {
  struct Wheels {
    float left, right;
  };
  static Wheels wheels(const Input &i) { return {i.pose.left, i.pose.right}; }
  static float motion(Wheels a, Wheels b) {
    return fmaxf(fabsf(a.left - b.left), fabsf(a.right - b.right));
  }
  static float forward(Wheels a, Wheels b) {
    return ((a.left - b.left) + (a.right - b.right)) * .5f;
  }
  static constexpr float fullTurn = 6.2831853f;
  // Absolute encoder endpoints preserve curved segments when played backwards.
  Wheels trail[32], scanOrigin{}, probeOrigin{}, reference{}, loss{};
  unsigned count = 0, retry = 0;
  float approachHeading = 0, scanHeading = 0, scanExtent = 0;
  int side = 0, scanSide = 1;
  bool opposite = false, centred = false, clearBand = false, knownTape = false;
  float bend = 0, oppositeBend = 0;
  Wheels turnStart{}, oppositeStart{};
  float turnHeading=0;
  void checkpoint(const Input &i) {
    count = 1;
    trail[0] = wheels(i);
    approachHeading = i.pose.heading;
  }
  void remember(Wheels p) {
    if (motion(p, trail[count - 1]) < 15)
      return;
    if (count ==
        32) { // Keep the anchor and progressively coarsen older samples.
      for (unsigned n = 1; n < 16; ++n)
        trail[n] = trail[2 * n];
      count = 16;
    }
    trail[count++] = p;
  }
  Command arc(float speed) const {
    float inside = speed * (1 - bend);
    return side > 0 ? Command{inside, speed, inside==0} : Command{speed, inside, inside==0};
  }
  float probeSpeed() const {
    return fmaxf(30.f, fminf(config.captureSpeed, 125.f));
  }
  Command spin() const {
    float speed = fminf(config.turnSpeed, 150.f) * scanSide;
    return {-speed, speed};
  }
  // Scale both velocities together to undo the wheel displacements in
  // proportion.
  bool returning = false;
  Wheels returnTarget{}, returnSign{};
  static float direction(float d) { return (d > 0) - (d < 0); }
  Command toward(Wheels target, Wheels p) {
    float l = target.left - p.left, r = target.right - p.right;
    if (!returning || target.left != returnTarget.left ||
        target.right != returnTarget.right) {
      returnTarget = target;
      returnSign = {direction(l), direction(r)};
      returning = true;
    }
    // Each wheel completes its segment when it reaches or crosses the endpoint.
    // Coast after crossing must not demand an immediate reversal.
    if (returnSign.left * l <= 5.2f) {
      returnSign.left = 0;
      l = 0;
    }
    if (returnSign.right * r <= 5.2f) {
      returnSign.right = 0;
      r = 0;
    }
    float d = fmaxf(fabsf(l), fabsf(r));
    if (!d) {
      returning = false;
      return {};
    }
    float scale = fminf(fminf(config.turnSpeed, 125.f), fmaxf(45.f, 2 * d)) / d;
    return {l * scale, r * scale};
  }
  void scan(const Input &i) {
    state = SEARCH;
    reason = "LINE_SCANNING";
    scanOrigin = wheels(i);
    scanHeading = i.pose.heading;
    scanExtent = 0;
    clearBand = !i.tape;
    scanSide = side ? side : (retry % 2 ? -1 : 1);
  }

public:
  enum State {
    STOPPED = 0,
    FOLLOW = 1,
    SEARCH = 3,
    VERIFY,
    UNDO_PROBE,
    UNWIND,
    RETRACE,
    ADVANCE
  } state = STOPPED;
  Config config;
  const char *reason = "LINE_STOPPED";
  void stop() {
    state = STOPPED;
    reason = "LINE_STOPPED";
  }
  bool begin(const Input &i) {
    checkpoint(i);
    returning = false;
    retry = 0;
    side = 0;
    bend = 0;
    opposite = centred = false;
    knownTape = i.tape == 2;
    reference = wheels(i);
    state = FOLLOW;
    reason = "LINE_FOLLOW";
    return true;
  }
  Command step(const Input &i) {
    Wheels p = wheels(i);
    if (state == STOPPED)
      return {};
    if (state == FOLLOW) {
      remember(p);
      if (!i.tape) {
        centred = false;
        loss = p;
        if (retry) {
          state = ADVANCE;
          reason = "LINE_SHIFT_SCAN";
        } else
          scan(i);
      } else if (i.tape == 2) {
        if (!centred) {
          centred = true;
          reference = p;
        }
        if (forward(p, reference) >= 20) {
          checkpoint(i);
          knownTape = true;
          side = 0;
          if (forward(p, reference) > 200)
            retry = 0;
        }
        bend = 0;
        opposite = false;
        reason = "LINE_FOLLOW";
        return {config.cruiseSpeed, config.cruiseSpeed};
      } else {
        centred = false;
        int observed = (i.tape == 4 || i.tape == 6)   ? 1
                       : (i.tape == 1 || i.tape == 3) ? -1
                                                      : 0;
        if (observed) {
          if (!side || bend == 0) {
            side = observed;
            turnStart = p;turnHeading=i.pose.heading;
            opposite = false;
          }
          if (observed == side) {
            if (opposite) turnStart=p;
            opposite = false;
            bend = (i.tape == 1 || i.tape == 4)
                       ? 1.f
                       : fminf(1.f, .80f + motion(p, turnStart) / 100.f);
          } else {
            if (!opposite) {
              opposite = true;
              oppositeStart = p;
              oppositeBend = bend;
            }
            bend = fmaxf(0.f, oppositeBend - motion(p, oppositeStart) /
                        (side*(i.pose.heading-turnHeading)>.20f ? 25.f : 50.f));
            if (bend == 0) {
              side = observed;
              turnStart = p;turnHeading=i.pose.heading;
              opposite = false;
              bend = .80f;
            }
          }
        }
        reason = side > 0   ? "LINE_LEFT"
                 : side < 0 ? "LINE_RIGHT"
                            : "LINE_FOLLOW";
        return arc(fminf(config.cruiseSpeed, config.correctionSpeed));
      }
    }
    if (state == ADVANCE) {
      remember(p);
      if (i.tape == 2) {
        if (!centred) {
          centred = true;
          reference = p;
        }
        if (forward(p, reference) >= 20) {
          checkpoint(i);
          knownTape = true;
          retry = 0;
          side = 0;
          bend = 0;
          state = FOLLOW;
          return step(i);
        }
        return {probeSpeed(), probeSpeed()};
      }
      centred = false;
      if (forward(p, loss) < retry * 25.f)
        return {probeSpeed(), probeSpeed()};
      scan(i);
    }
    if (state == SEARCH) {
      reason = "LINE_SCANNING";
      scanExtent = fmaxf(scanExtent, scanSide * (i.pose.heading - scanHeading));
      if (scanExtent >= fullTurn) {
        state = UNWIND;
      } else {
        if (!(i.tape & 2))
          clearBand = true;
        bool incoming =
            knownTape &&
            maze::angleDistance(i.pose.heading,
                                approachHeading + fullTurn * .5f) < .22f;
        if (!clearBand || (i.tape!=2 && i.tape!=3 && i.tape!=6) || incoming)
          return spin();
        probeOrigin = p;
        state = VERIFY;
      }
    }
    if (state == VERIFY) {
      reason = "LINE_CONFIRMING";
      if (i.tape == 2 || i.tape == 3 || i.tape == 6) {
        if (forward(p, probeOrigin) >= 20) {
          checkpoint(i);
          knownTape = true;
          retry = 0;
          centred = false;
          opposite = false;
          side = 0;
          bend = 0;
          state = FOLLOW;
          return step(i);
        }
        const float v = probeSpeed();
        if(i.tape==6)return {.2f*v,v};
        if(i.tape==3)return {v,.2f*v};
        return {v,v};
      }
      state = UNDO_PROBE;
    }
    if (state == UNDO_PROBE) {
      reason = "LINE_UNDO_PROBE";
      auto d = toward(probeOrigin, p);
      if (d.left || d.right)
        return d;
      clearBand = false;
      state = SEARCH;
      return spin();
    }
    if (state == UNWIND) {
      reason = "LINE_UNWIND";
      auto d = toward(scanOrigin, p);
      if (d.left || d.right)
        return d;
      state = RETRACE;
    }
    reason = "LINE_RETRACING";
    // A sustained centred return to the approach heading is an optical anchor.
    if (i.tape != 2 ||
        maze::angleDistance(i.pose.heading, approachHeading) > .2f)
      centred = false;
    else if (!centred) {
      centred = true;
      reference = p;
    }
    bool found =
        centred && forward(reference, p) >= 20 && motion(p, trail[0]) < 50;
    while (count && !found) {
      auto d = toward(trail[count - 1], p);
      if (d.left || d.right)
        return d;
      --count;
    }
    retry = retry % 6 + 1;
    checkpoint(i);
    side = 0;
    bend = 0;
    opposite = centred = false;
    loss = p;
    state = i.tape ? FOLLOW : ADVANCE;
    reason = "LINE_REAPPROACH";
    return {probeSpeed(), probeSpeed()};
  }
};
} // namespace line
