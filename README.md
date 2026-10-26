# Open Robotics Studio

**First milestone: can a two-joint planar robot arm reach a target within its joint limits?**

Enter a target in metres → solve both inverse-kinematics branches → filter by joint limits → verify each pose with forward kinematics → inspect a JSON report and SVG scene.

This is an offline kinematics analysis tool for students and builders checking a simple arm design. It is the first working slice of a longer-term robotics studio, not a completed 3D platform or a learned physical-AI system.

## Run in under a minute

Requires Python 3.10+; no packages, GPU, API keys, cloud services, or hardware.

```sh
git clone https://github.com/Roy0x/open-robotics-studio.git
cd open-robotics-studio
python -m robotics_studio --x 0.6 --y 0.3
```

On Windows, use `py` if `python` is unavailable. Open `artifacts/reach/scene.svg` in a browser and inspect `artifacts/reach/result.json`.

Expected: `reachable: 2 valid poses`, each with a forward-kinematics position error below `1e-10` metres. Both poses are drawn over the target. A committed example is in [examples/reachable.svg](examples/reachable.svg).

Try a target outside the workspace:

```sh
python -m robotics_studio --x 1.2 --y 0 --out artifacts/unreachable
```

Expected: `unreachable: 0 valid poses`. For a constrained arm:

```sh
python -m robotics_studio --x 0.6 --y 0.3 --links 0.6 0.4 --limits-deg -180 180 0 180 --out artifacts/constrained
```

Expected: one valid pose. Link lengths must be positive and finite. Limits are inclusive, ordered, and within [-180, 180] degrees. Output paths are directories; rerunning into the same directory replaces the two output files.

## What the result means

| Status | Meaning |
| --- | --- |
| `reachable` | At least one static joint pose satisfies the limits |
| `unreachable` | Target lies outside the arm's annular workspace |
| `joint_limit_blocked` | Geometrically reachable, but neither pose satisfies the limits |
| `degenerate` | Equal links folded at the origin; infinitely many shoulder angles, deliberately not selected |

Exit code: 0 for reachable, 1 for other results, 2 for invalid CLI input. JSON contains target, lengths, limits, joint angles in radians and degrees, elbow/tip coordinates, and position error in metres.

Frame: base at (0,0), +X right, +Y up; angles counterclockwise. Joint 1 is relative to +X; joint 2 is relative to link 1. Links are rigid and zero-width. Singular full-extension poses are valid static solutions. Numerical boundary tolerance is `1e-12` metres; joint-limit tolerance is `1e-12` radians.

**Reachability does not establish a collision-free path or safe hardware motion.** There is no dynamics, collision checking, trajectory planning, actuator control, perception, URDF import, 3D editor, or AI model in this milestone.

## Structure and verification

```text
robotics_studio/arm.py       # Forward/inverse kinematics and joint limits
robotics_studio/__main__.py  # CLI, JSON and SVG export
tests/test_arm.py            # Boundaries, limits, invalid inputs, round trips
examples/                   # Reproducible sample output
```

```sh
python -m unittest discover -s tests -v
```

The milestone is accepted when the default target produces two accurate poses, the constrained case produces one, unreachable targets produce no pose, and the tests pass.

## Next decision

Use this with a real proposed arm's lengths, joint limits, and target points. Add a target-list reachability report only if that exercise needs it. Defer 3D, physics engines, and AI integration until a concrete task requires them.

MIT licensed; see [LICENSE](LICENSE).
