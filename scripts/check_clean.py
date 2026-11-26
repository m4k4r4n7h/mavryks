from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARKERS = {
    ".py": ("#",),
    ".ts": ("//", "/*"),
    ".tsx": ("//", "/*"),
    ".css": ("/*",),
    ".html": ("<!--",),
    ".json": (),
    ".txt": (),
    ".sh": ("#",),
}
SKIP_DIRS = {"node_modules", "venv", ".venv", "dist", "__pycache__", ".git", "run"}


def docstring_line(line: str) -> bool:
    stripped = line.strip()
    return stripped.startswith(('"""', "'''"))


def scan() -> list[str]:
    bad: list[str] = []
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = [item for item in dirs if item not in SKIP_DIRS]
        for name in files:
            ext = os.path.splitext(name)[1]
            if ext not in MARKERS:
                continue
            path = os.path.join(base, name)
            rel = os.path.relpath(path, ROOT)
            with open(path, "r", encoding="utf-8", errors="ignore") as handle:
                for lineno, line in enumerate(handle, 1):
                    stripped = line.lstrip()
                    if lineno == 1 and line.startswith("#!"):
                        continue
                    if ext == ".py" and docstring_line(line):
                        bad.append(f"{rel}:{lineno}: docstring")
                        continue
                    for marker in MARKERS[ext]:
                        if marker and stripped.startswith(marker):
                            bad.append(f"{rel}:{lineno}: {marker}")
    return bad


def main() -> int:
    bad = scan()
    if bad:
        for item in bad:
            print(item)
        return 1
    print("CLEAN_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
