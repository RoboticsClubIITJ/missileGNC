# Checking Accuracy

*Basic test cases for validating the plant + autopilot, and what the output should look like for each one*

## 1. Thrust = 0 — pure free fall

**Setup:** T = 0, δ = 0, Vx0 = 0, Vy0 = 0

With no thrust and no initial velocity, the only force acting is gravity, so ax = 0 and ay = −g.

Plot y against t — it should trace a clean downward parabola. x, on the other hand, should stay a flat horizontal line for the entire flight. If x drifts even slightly, something is leaking into the horizontal channel that shouldn't be there — worth checking the Force Resolver logic first.

## 2. Thrust = 0, with a horizontal launch velocity — projectile motion

**Setup:** T = 0, δ = 0, Vx0 = 20, Vy0 = 0

This is the same free-fall physics as above, just with sideways motion added in. Plotting y against x this time should give you the familiar arc of a thrown ball.

## 3. Thrust exactly balances gravity — hover

**Setup:** δ = 0, T = m·g, Vx0 = 0, Vy0 = 0

Here the thrust is tuned to exactly cancel gravity, so the net vertical acceleration is zero. The missile should simply stay put — no upward or downward drift at all. Any drift here, in either direction, almost always means the sign on g is wrong somewhere, or the thrust-to-acceleration scaling isn't quite right.

## 4. Thrust greater than gravity, straight up — pure vertical acceleration

**Setup:** δ = 0, T = 2·m·g, Vx0 = 0, Vy0 = 0

With the deflection angle at zero and thrust exceeding gravity, the missile should accelerate straight upward at a constant net rate of g, with x staying perfectly flat throughout.

As with the free-fall case, any horizontal drift here — even a small one — points to a definite bug, since there's no mechanism in this setup that should ever move the missile sideways.

## 5. Thrust with a fixed deflection angle — constant-acceleration parabola

**Setup:** T = 1000, δ = 10°, m = constant, Vx0 = 0, Vy0 = 0

This is the general case: ax = (T·sin10°)/m and ay = (T·cos10°)/m − g, both constant, so the trajectory is a constant-acceleration parabola in both x and y.

Here, you compare the simulated trajectory against the closed-form analytical solution and check that the position RMSE comes out close to zero.

## 6. Symmetry check — flipping the sign of δ

**Setup:** Run δ = +10° and δ = −10°, with everything else identical

This one doesn't need any analytical derivation at all — just run the same scenario twice with opposite deflection angles and compare the two outputs directly to each other. The x(t) trajectories should be exact mirror images, and y(t) should be identical between the two runs.

It's a quick way to catch an asymmetric bug — for example sin and cos swapped somewhere, or an angle-wrapping issue.

## 7. Energy / momentum sanity check

**Setup:** Any scenario, constant thrust, no aerodynamics

This is more of a general-purpose catch-all than a specific test case, and it stays useful even later once the thrust profile gets more complex and a closed-form solution isn't easy to derive.

The idea is to check that the work-energy theorem holds: the change in kinetic energy should equal the work done by thrust plus the work done by gravity. In practice, that means numerically integrating F·v over time and comparing it against ½m(V_final² − V_initial²). Since there's no drag in this model, any mismatch here usually means a units or force-calculation error somewhere in the plant.
