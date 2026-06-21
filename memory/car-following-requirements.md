---
name: car-following-requirements
description: Quadcopter must actively follow a moving car, not just hover in place
metadata:
  type: project
---

The drone's mission is **active car-following** — the target (car with IR light board) can move quickly. This imposes different requirements than a simple position-hold hover:

1. **Responsiveness matters** — position loop must react to target motion, not just slowly drift back. The car can accelerate, stop, turn.
2. **All parameters need continuous optimization** — KP, FF, Vmax, speed loop gains, everything. No parameter is "done."
3. **Feed-forward (FF) is essential** — when the car moves, `d(err)/dt` reflects target velocity. FF translates that directly into a speed command so P doesn't have to accumulate error first.
4. **Trade-off** — fast following vs. not amplifying pendulum oscillation. Slow P (0.20) provides base centering, FF provides fast response to target motion, speed loop provides damping.

**How to apply:** When tuning parameters, always consider both the pendulum disturbance rejection AND the car-following responsiveness. Parameters that are too slow will lose the car. Parameters that are too fast will amplify the tether swing. The solution is frequency separation: slow return force + fast feed-forward + mid-band damping.

Related: pendulum oscillation analysis in test.txt flight data.
