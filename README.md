# JavaScript 2D car dynamics simulation

### Physics model features:

 * auto-generated track
 * collisions with the 4 walls
 * basic static and dynamic friction
 * air drag applied to the center of mass
 * "arrow-key steering aids": velocity-based limitations to steering angle
  and its change rate

The physics model is "as simple as possible": Frictional forces are applied to
two points: the center of the front and rear axle instead of the four wheels
separately. The car is therefore actually modeled more like a "motorcycle that
does not bank". This is the same as assuming a lot of symmetry between the
forces to the left and right wheel on each axle, i.e.,

 * no tire slip due to deformations
 * no suspension
 * no roll
 * equal coefficients of friction

*TODO*:

 * brakes
 * non-constant torque from engine
 * noise

### AI drivers

 * PID control with a few tricks
 * Route planning with an ad-hoc "rubber band model" (uses a web worker)

*TODO*

 * Physics-based controller (non-PID)
