import json
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RECORD_PATH = REPO_ROOT / "cache" / "demo_action_record.json"


def load_record() -> int:
    if not RECORD_PATH.exists():
        return 0
    try:
        data = json.loads(RECORD_PATH.read_text())
    except (json.JSONDecodeError, OSError):
        return 0
    return int(data.get("actions", 0))


def save_record(actions: int) -> None:
    RECORD_PATH.parent.mkdir(parents=True, exist_ok=True)
    RECORD_PATH.write_text(json.dumps({"actions": actions}, indent=2) + "\n")


def try_commit_record(actions: int) -> bool:
    """
    Persist a new all-time action-completion record and commit it to git.
    Returns True when a new record was saved and committed.
    """
    previous = load_record()
    if actions <= previous:
        return False

    save_record(actions)
    try:
        subprocess.run(
            ["git", "add", str(RECORD_PATH.relative_to(REPO_ROOT))],
            cwd=REPO_ROOT,
            check=True,
        )
        subprocess.run(
            [
                "git",
                "commit",
                "-m",
                f"Demo record: {actions} actions completed successfully.",
            ],
            cwd=REPO_ROOT,
            check=True,
        )
        print(
            f"New demo record: {actions} actions completed — committed to git.",
            flush=True,
        )
        return True
    except subprocess.CalledProcessError as exc:
        print(
            f"Saved demo record ({actions} actions) but git commit failed: {exc}",
            flush=True,
        )
        return False
