# Local control laboratory

- Read README.md, MODEL_ASSUMPTIONS.md and ROADMAP.md before extending experiments.
- This directory is the desktop/offline implementation. Resolve assets relative to
  this directory; never depend on the SSH host's home directory or ROS installation.
- The user wants incremental learning. Implement the requested stage and explain
  its model, objective, constraints and evidence; do not silently implement all later stages.
- Preserve existing remote RViz/MoveIt clients and unrelated working-tree edits.
- Use output-side SI coordinates and torque. Controller output is total torque;
  gravity is added by the controller exactly once; MuJoCo motor gear is 1.
- Apply configured armature once in MuJoCo and once in the independent Pinocchio
  model. Do not fake a tool payload to represent rotor inertia. Label estimates.
- Keep physical friction, software damping and friction assistance distinct.
- For model changes, run the foundation tests and cross-engine validation when
  Pinocchio is available. Report a missing dependency as untested, not as a pass.
- Log solver status, residuals, constraint violations, timing and parameter/model
  versions. Never use a failed/infeasible optimizer result as an ordinary command.
- Default to offline experiments. A real-hardware stage needs an explicitly
  defined robot-side integration task; do not add transport as a convenience.
- Do not commit generated artifacts, environments or machine-specific paths.
