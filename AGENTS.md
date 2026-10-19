# Repository Guidelines

## Project Structure & Module Organization

This repository contains architecture and development documentation; components are implemented under `Src/`. CMake builds, GoogleTest/CTest, npm/vitest, and the `tools/task_runner.py` entry point are available.

- `Docs/structure.md`: overall planning architecture and motivations.
- `Docs/component_development_plan.md`: component dependencies, development steps, references, and acceptance milestones.
- `Docs/module/`: decision, map, and trajectory-planning designs.
- `Docs/paper/`: research notes; `Docs/reference.md`: reference index.
- `Docs/former/` and `Docs/gallery/`: historical material and demonstrations.
- `Docs/10-文章图片/`: document image assets.

Place new implementation under `Src/`, organized by independently buildable component. Keep tests alongside each component.

## Build, Test, and Development Commands

Current documentation checks:

```sh
rg --files Docs                 # List documentation and assets
git diff --check                # Check tracked changes for whitespace errors
git diff -- Docs AGENTS.md      # Review documentation changes
git status --short              # Include newly added files in review
```

Working build, test, benchmark, run, and fixture workflows go through `tools/task_runner.py` (subcommands `build`/`test`/`benchmark`/`run`/`fixtures`); `tools/README.md` documents the component registry and command templates. The core uses C++17 and CMake.

## Resource Monitoring & Concurrency

These limits apply across all agents working on this repository, not separately to each agent.

- Before starting compilation, tests, or runtime workloads, assign a dedicated resource-monitoring agent. Keep it active until workloads finish; it must not perform compilation or tests itself. If a monitoring agent cannot be started, do not launch these workloads without an equivalent active monitor.
- Sample system-wide CPU and memory utilization every 2 seconds. CPU utilization is aggregate busy time across all logical CPUs; memory utilization is `(MemTotal - MemAvailable) / MemTotal`, excluding reclaimable cache from pressure accounting.
- If either CPU or memory utilization exceeds 90%, immediately stop the most recently started active repository compilation, test, or runtime task. Track task start times and process groups; terminate that task and its children gracefully, then force termination after 5 seconds if necessary. Never terminate unrelated system or user processes.
- Suspend new workload launches while either utilization exceeds 90%. Recheck after stopping a task; if pressure remains above the threshold, stop the newest remaining repository task. Report the measurements and stopped task, and do not automatically restart it until pressure clears and its resource requirements have been reduced.
- Allow only one compilation task at a time. Limit build parallelism to four jobs: `cmake --build <build-dir> --parallel 4`, `make -j4`, or `ninja -j4`. While any repository compilation is active, queue additional builds rather than starting them. Use one shared build lock across agents and build directories.
- Run as many independent tests and runtime tasks as resources permit, with a combined maximum of four concurrent worker processes across all agents. Include test-runner workers and independently running child workers in this limit; nested parallelism must not multiply the cap. Persistent application services count toward the runtime limit; the lightweight monitoring agent does not.
- Register workloads before launch and release their slots and locks on completion, cancellation, or failure. Resource thresholds override concurrency targets; never fill available slots when doing so would violate the limits.

## Coding Style & Naming Conventions

Use descriptive `snake_case` filenames, following existing module documents. Keep Markdown headings hierarchical, tables readable, and repository links relative. Preserve the Chinese language of existing design documents. No formatter or linter is currently configured; establish component conventions before introducing code.

## Architecture & Dependency Rules

Keep core libraries ROS-free. Integration order is RS and collision detection → Hybrid A* → Dftpav with safety corridors and trajectory validation. Map development precedes browser display. Algorithms use synthetic fixtures independently of display and Lanelet2. B-spline and polynomial libraries remain independent. Keep Dftpav an isolated experimental backend and retain upstream attribution and license information.

## Testing Guidelines

E0 tools target Linux and CPython 3.8–3.12 using only the standard library, with `unittest` tests under `tools/tests/`. Shared fixtures live in root `tests/fixtures/`; component test code lives in `Src/<component>/tests/`. E0's explicit acceptance checklist, JSON contract, dependency pins, adapter contract, fuzz counts, and benchmark batch sizes are defined in the execution plan. The integration owner maintains `Src/validation/`, reusing collision interfaces; its end-to-end acceptance belongs to Dftpav.

GoogleTest and CTest are implemented and registered per component. No coverage percentage is mandated. Follow the interface, fixture, tolerance, scheduling, and benchmark rules in `Docs/component_development_plan.md`, sections 5–9. New components must provide deterministic correctness tests, standalone examples, and benchmarks. Include invalid inputs, cancellation or timeout where applicable, collision edge cases, and numerical boundaries. Use descriptive test names such as `rejects_colliding_start`.

## Reports & Experiment Evidence

### Continue After Noncritical Failures

Prioritize completing the planned components. A failed build, test, or benchmark must not end the whole development task. Record evidence, attempt targeted fixes, use a validated fallback, and continue independent work. After three unsuccessful targeted fixes for the same issue, switch approach or record a local blocker and advance other tasks; revisit unresolved issues during integration. Follow section 5.4 of the execution plan. Stop affected workloads for safety/correctness violations, data corruption, resource limits, or monitor failure, without stopping unrelated safe work. Never relax hard safety checks to claim completion. Use `completed_with_issues` only for delivered components with nonblocking issues, report acceptance separately, and track unresolved items in `Docs/reports/ISSUES.md`.

Maintain `Docs/reports/STATUS.md` as the human-readable progress entry point. Use `Docs/reports/report_template.md` for acceptance and benchmark reports. Store raw inputs, effective configuration, source provenance, logs, per-trial metrics, resource samples, and outputs under unique `artifacts/runs/<run_id>/` directories. Preserve failures and resource-stopped runs; retries receive new IDs. Never mark a component passed without evidence or present planned tools as implemented. Formal benchmarks run without competing repository workloads; ordinary development tests retain the global four-worker limit.

## Commit & Pull Request Guidelines

History uses short free-form messages, including `update`; no formal convention is established. Prefer specific subjects, such as `Document Lanelet2 display dependencies`. Keep commits focused. PRs should describe the problem, changes, validation, and affected interfaces; link relevant issues and include screenshots for display changes. Preserve unrelated working-tree changes and review moved-document links before committing.
