import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from monkey_bot import demo_progress


class DemoProgressTest(unittest.TestCase):
    def test_try_commit_record_skips_when_not_a_new_high(self):
        with tempfile.TemporaryDirectory() as tmp:
            record_path = Path(tmp) / "demo_action_record.json"
            record_path.write_text(json.dumps({"actions": 5}) + "\n")
            with patch.object(demo_progress, "RECORD_PATH", record_path):
                self.assertFalse(demo_progress.try_commit_record(3))
                self.assertEqual(demo_progress.load_record(), 5)

    def test_try_commit_record_saves_and_commits_new_high(self):
        with tempfile.TemporaryDirectory() as tmp:
            record_path = Path(tmp) / "demo_action_record.json"
            with patch.object(demo_progress, "RECORD_PATH", record_path):
                with patch.object(demo_progress, "REPO_ROOT", Path(tmp)):
                    with patch("monkey_bot.demo_progress.subprocess.run") as run:
                        self.assertTrue(demo_progress.try_commit_record(2))
                        self.assertEqual(demo_progress.load_record(), 2)
                        self.assertEqual(run.call_count, 2)


if __name__ == "__main__":
    unittest.main()
