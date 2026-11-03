"""Compare the retained manual-mapping sources and/or historical r5 baseline.

This is an on-demand provenance check, not a rule that future commits must
retain the initial source hashes. No original workspace or device is needed.
"""
import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inside(root, relative):
    candidate = root / relative
    if (Path(relative).is_absolute() or ".." in Path(relative).parts
            or candidate.is_symlink()):
        raise ValueError("Unsafe manifest path: " + relative)
    resolved = candidate.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError:
        raise ValueError("Manifest path escapes repository: " + relative) from None
    return resolved


def verify_manual_import(root=ROOT):
    document = json.loads((root / "provenance/source_manifest.json").read_text(encoding="utf-8"))
    if document.get("schema") != "manual-mapping-source-manifest-v1":
        raise ValueError("Unexpected manual-mapping source manifest")
    failures = []
    for relative, entry in document["files"].items():
        path = inside(root, relative)
        if not path.is_file():
            failures.append(relative + ": missing")
        elif digest(path) != entry["sha256"] or path.stat().st_size != entry["bytes"]:
            failures.append(relative + ": differs from manual-mapping import")
    return {"kind": "manual_mapping_import", "checked": len(document["files"]), "failures": failures}


def verify_r5_baseline(root=ROOT):
    document = json.loads((root / "baselines/manual-mapping-r5.json").read_text(encoding="utf-8"))
    if document.get("schema") != "host-manual-mapping-r5-baseline-v1":
        raise ValueError("Unexpected r5 baseline")
    files = document["original_manifest"]["files"]
    failures = []
    for relative, expected in files.items():
        path = inside(root / "app", relative)
        if not path.is_file() or digest(path) != expected:
            failures.append(relative + ": differs from historical r5 source")
    return {"kind": "manual_mapping_r5_sources", "checked": len(files),
            "historical_archive_sha256": document["archive"]["sha256"], "failures": failures}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-manual-import", "--verify-initial-import",
                        dest="verify_manual_import", action="store_true",
                        help="Compare the retained hand-pushed mapping source files")
    parser.add_argument("--verify-r5-baseline", action="store_true",
                        help="Compare the nine manual-mapping sources with actual r5")
    args = parser.parse_args(argv)
    checks = []
    if args.verify_manual_import or not args.verify_r5_baseline:
        checks.append(verify_manual_import())
    if args.verify_r5_baseline:
        checks.append(verify_r5_baseline())
    okay = all(not item["failures"] for item in checks)
    print(json.dumps({"passed": okay, "checks": checks}, ensure_ascii=False, indent=2))
    return 0 if okay else 1


if __name__ == "__main__":
    raise SystemExit(main())
