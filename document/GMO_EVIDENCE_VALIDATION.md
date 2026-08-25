# GMO/GRF evidence validation

This phase is deliberately read-only. GMO/GRF output is logged as contact
evidence but is not connected to `contactEstimate`, the Linear KF, CMPC,
motor commands, gait switching, or safety decisions.

## Evidence contract

- Leg order is always `[FR, FL, HR, HL]`.
- All force vectors are world-aligned and expressed in newtons.
- `gmo_valid` means the momentum observer result is numerically valid.
- `gmo_ready` additionally requires the configured warm-up sample count.
- `gmo_force_raw_*` is the direct damped Jacobian solve.
- `gmo_force_*` is raw force minus the configured per-leg bias, followed by
  the configured first-order low-pass filter.
- `grf_leg_valid_*` and `grf_jacobian_quality_*` must be checked before a
  future fusion layer consumes a leg's evidence.
- `scheduled_contact_*` is gait evidence only. It is never overwritten by GMO.

Invalid reason enums are defined in `GeneralizedMomentumObserver.h` and
`GroundReactionForceEstimator.h`. Consumers must not infer validity from a
zero force vector.

## Simulation validation

Build the simulation target and run the full matrix:

```bash
cmake -S . -B build -DBUILD_SIM=ON
cmake --build build -j2
.venv/bin/python tools/run_gmo_validation_suite.py
```

The suite covers the flat nominal model, a 2 kg model-mismatch payload, and
stair terrain for early/unscheduled touchdown. Normal trot supplies repeated
single-leg swing intervals in addition to the four-leg standing interval.
Each run produces a JSON evidence report next to its logs.

Analyze an existing telemetry file independently with:

```bash
.venv/bin/python tools/analyze_gmo_evidence.py data/cmpc_telemetry_<timestamp>.csv
```

Use `--strict` in CI or a release gate. The analyzer reports per-leg ROC/PR,
swing leakage, stance force error, event latency, invalid rate, runtime, and a
suggested median swing bias. Bias suggestions from a calibration run must be
copied into `GMOConfig::force_bias_world` only after a separate validation run
shows improvement without hiding a sign, model, or torque-feedback error.

## Robot shadow gate

Only begin after every simulation gate passes.

1. Put the robot in a harness with an operator ready to enter joint damping.
2. Record 10 seconds standing, then short straight-line trots at 0.1 m/s and
   0.3 m/s on level ground.
3. Confirm there are no worker deadline misses, unexpected GMO resets, NaN/Inf,
   force-limit rejects, or persistent low Jacobian-quality legs.
4. Compare repeat runs for stable stance/swing separation and support-force
   consistency. Hardware has no MuJoCo-equivalent force ground truth, so this
   step cannot waive a failed simulation gate.

Do not enable probabilistic contact fusion or KF covariance adaptation in this
phase. That requires a separate reviewed change after the evidence gates pass.
