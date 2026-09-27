# SWRS seed files (V-001)

One JSON file per rover_ros package (`<package>.json`). `scripts/build_requirements.m`
turns each file into `requirements/SWRS_<PREFIX>.slreqx`. After that first build the
`.slreqx` files are the master (see the top-level README). These files are the V-001
authoring record.

## Schema

```json
{
  "package": "rover_twist_mux",
  "prefix": "MUX",
  "document": "ROVER-A1-ROVER_TWIST_MUX-SWRS-V-001",
  "title": "rover_twist_mux Software Requirements Specification",
  "scope": "One paragraph: what the package is and what it is responsible for in the platform.",
  "requirements": [
    {
      "id": "SWR-MUX-001",
      "category": "Functional",
      "text": "The rover_twist_mux shall ...",
      "rationale": "...",
      "verification": "Test",
      "acceptance": "Measurable pass/fail criterion.",
      "priority": "Must",
      "status": "As-Built",
      "parents": ["SYS-SR-005"],
      "derived_rationale": "",
      "evidence": ["rover_twist_mux/config/rover_twist_mux.yaml:6-9"],
      "implementation": "Implemented",
      "verified_by": ["rover_twist_mux/test/test_twist_mux_priorities.py"],
      "comments": ""
    }
  ]
}
```

| Field | Allowed values / rules |
|-------|------------------------|
| `id` | `SWR-<PREFIX>-NNN`, 3 digits, unique, sequential |
| `category` | `Functional`, `Interface`, `Performance`, `Safety`, `Parameter`, `Diagnostics`, `Lifecycle`, `Build` |
| `text` | One "shall" per requirement, starting "The <package> shall". Verifiable. Numbers must come from the evidence. |
| `verification` | `Test`, `Inspection`, `Analysis`, `Demonstration` (comma-separated when more than one) |
| `priority` | `Must`, `Should`, `Could` |
| `status` | `As-Built`: the code does it today (reverse-engineered). `Proposed`: needed by a SYS-SR but missing or different in the code. |
| `parents` | SYS-SR IDs this is derived from. It may be empty only if `derived_rationale` says why (a technical requirement with no system parent). |
| `evidence` | `path:line` relative to `src/rover_ros/`. Required for `As-Built`. For `Proposed`, cite the code that shows the gap. |
| `implementation` | `Implemented`, `Partial`, `Gap` (not implemented), `Deviation` (implemented differently from the SYS-SR or the documentation) |
| `verified_by` | Existing test files, relative to `src/rover_ros/`. Leave it empty if no test exists. Model tests are linked later by the build scripts. |
| `comments` | TBD/TBC notes, doc drift, open questions |

Value rules: never invent numbers. Every number comes from the cited evidence or from
a SYS-SR. If the SYS-SR value is `[TBD]`, write `[TBD per SYS-SR-0xx]`.
