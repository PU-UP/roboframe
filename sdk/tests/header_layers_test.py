"""Keep public SDK layers one-way; run on both source and installed header trees."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
allowed = {
    "core": {"core"},
    "datatype": {"datatype"},
    "orchestration": {"core", "orchestration"},
}
errors = []
for header in sorted(root.rglob("*.hpp")):
    relative = header.relative_to(root)
    if relative.as_posix() == "sdk_version.hpp":
        continue  # Generated release identity.
    layer = relative.parts[0]
    if layer not in allowed:
        errors.append(str(relative) + ": header must have an explicit layer")
        continue
    for dependency in re.findall(r'#include\s*[<"]([^>"]+)[>"]', header.read_text()):
        if dependency.startswith("roboframe/"):
            target = dependency[len("roboframe/"):]
            if target == "sdk_version.hpp":
                continue
            if not (root / target).is_file():
                errors.append(str(relative) + ": missing " + dependency)
            elif target.split("/")[0] not in allowed[layer]:
                errors.append(str(relative) + ": forbidden dependency " + dependency)
        elif "/" in dependency or dependency.endswith(".hpp"):
            errors.append(str(relative) + ": non-SDK dependency " + dependency)
if errors:
    raise SystemExit("\n".join(errors))
print("PASS: SDK core / datatype dependency boundaries")
