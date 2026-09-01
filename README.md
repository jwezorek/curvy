# Curvy

Curvy is an experimental **fictional-physics** project built around a simple premise:

> What would mechanics look like if circular motion, rather than straight-line Newtonian motion, were primitive?

The project originated in the 1990s and has been revisited several times since. The current codebase is based on the 2021 implementation, which is the most complete version so far.

## Core idea

The basic object in Curvy is a **curvy vector**. It is not fundamentally an ordinary velocity vector. Conceptually, a curvy vector consists of:

- a circle of revolution,
- an orientation around that circle,
- a magnitude interpreted as linear speed along the circle.

Straight-line motion is treated as the degenerate infinite-radius case.

An ordinary tangent vector can be derived at a point on the circle when useful for calculation, but the directed circle itself is the primitive motion object.

## Curvy arithmetic

The 2021 implementation combines two curvy vectors applied at the same point by combining:

1. their magnitudes,
2. their instantaneous tangent directions,
3. their signed curvatures.

For vectors `A` and `B`:

```text
m_result = m_A + m_B
```

and signed curvature is magnitude-weighted:

```text
k_result = (m_A * k_A + m_B * k_B)
           / (m_A + m_B)
```

The resulting directed circle is reconstructed from the combined tangent direction and curvature.

One interesting interpretation is that, for circles tangent at a common point, signed curvature behaves naturally as an affine coordinate. In homogeneous coordinates:

```text
C <-> (m, m*k)
```

the magnitude/curvature part of Curvy addition becomes ordinary linear addition.

## Collision geometry

A particularly important construction is the Curvy direction of impact between two colliding pucks.

For a striking puck \(A\) and struck puck \(B\), the impact trajectory is the unique circle that:

- passes through the center of \(A\),
- passes through the center of \(B\),
- is tangent at \(A\) to \(A\)'s current circle of revolution,
- has orientation consistent with \(A\)'s motion.

This gives a circle-native analogue of the ordinary line of impact.

## The unresolved problem: projection

The main open problem in Curvy is **projection**.

Given an incoming curvy vector and an impact circle, the geometry determines the direction of the transferred component quite naturally. What is less obvious is how much magnitude should be transferred along that circle.

Conceptually, Curvy needs an analogue of:

```text
source = projected component + residual component
```

defined directly on directed circles with magnitudes.

The 2021 implementation uses a geometry-derived transfer factor followed by an empirically chosen response curve. It produces interesting behavior, but the long-term goal is to find a projection law that follows more intrinsically from the geometry.

Some desired behaviors are:

- **same-circle full transfer**,
- **equal opposite motion on one circle reverses direction**,
- continuity under small geometric changes,
- translation/rotation/reflection invariance,
- coherent scale behavior.

Matching Newtonian collision mechanics in the straight-line limit is interesting, but is no longer considered mandatory if abandoning it leads to a more internally consistent Curvy mechanics.

## Current development direction

The immediate goal is not to replace the 2021 physics.

Instead, the codebase is being turned into a cleaner experimental platform:

1. fix clear correctness bugs in the old implementation,
2. make projection a first-class operation,
3. preserve the 2021 addition/subtraction rules as the baseline,
4. add tests for important Curvy invariants and collision cases,
5. experiment with alternative projection and arithmetic rules independently.

The broader research question is whether Curvy can be reduced to a small set of geometrically natural rules that feel inevitable *within Curvy-space*, rather than like Newtonian mechanics with curvature bolted on.

## Building

Curvy currently targets Windows and uses GDI+ for rendering.

### Requirements

- CMake 3.25 or newer
- a C++23 compiler
- Eigen3
- Boost
- Windows / GDI+

With Visual Studio 2022:

```powershell
cmake -S . -B out/build -G "Visual Studio 17 2022" -A x64
cmake --build out/build --config Debug
```

The generated executable target is `curvy`.

## Repository

https://github.com/jwezorek/curvy
