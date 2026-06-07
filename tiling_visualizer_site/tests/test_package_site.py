import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import package_site as ps


class TestProbe(unittest.TestCase):
    def test_parse_ffprobe_output(self):
        out = "h264,1920,1080,30000/1001,1314"
        info = ps.parse_probe_csv(out)
        self.assertEqual(info["width"], 1920)
        self.assertEqual(info["height"], 1080)
        self.assertAlmostEqual(info["fps"], 29.97002997, places=5)
        self.assertEqual(info["frames"], 1314)

    def test_parse_ffprobe_na_frames(self):
        info = ps.parse_probe_csv("h264,1920,1080,30/1,N/A")
        self.assertIsNone(info["frames"])


class TestTranscodeCmd(unittest.TestCase):
    def test_cmd_shape(self):
        cmd = ps.transcode_cmd(Path("/src/a.mp4"), Path("/out/b.mp4"))
        self.assertEqual(cmd[0], "ffmpeg")
        joined = " ".join(cmd)
        self.assertIn("libx264", joined)
        self.assertIn("+faststart", joined)
        self.assertIn("scale=1920:-2", joined)
        self.assertIn("yuv420p", joined)
        self.assertIn("-an", joined)
        self.assertIn("-g 30", joined)          # 1s GOP: cheap backward stepping

    def test_needs_transcode(self):
        with mock.patch.object(Path, "exists", return_value=False):
            self.assertTrue(ps.needs_transcode(Path("/s.mp4"), Path("/d.mp4")))


class TestManifest(unittest.TestCase):
    def make_cfg(self, tmp):
        run = tmp / "run.frames.json"
        run.write_text(json.dumps({"label": "r", "frames": []}))
        trials = tmp / "trials.json"
        trials.write_text(json.dumps({"aggregate": {}}))
        src = tmp / "src.mp4"
        src.write_bytes(b"x")
        return {
            "title": "T",
            "videos": [{"id": "v1", "title": "V1", "variants": [{
                "fov": "fov50", "source": str(src),
                "runs": [{"id": "r1", "label": "run 1", "type": "DYN", "path": run.name}],
                "trials": [{"id": "t1", "label": "trials 1", "path": trials.name}],
            }]}],
        }

    def test_build_manifest_entry(self):

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            probe = {"width": 3840, "height": 2160, "fps": 29.97, "frames": 1314,
                     "out_width": 1920, "out_height": 1080}
            man = ps.build_manifest(cfg, probe_lookup=lambda p: probe)
            v = man["videos"][0]["variants"][0]
            self.assertEqual(v["video"], "data/videos/v1_fov50.mp4")
            self.assertEqual(v["width"], 1920)
            self.assertEqual(v["height"], 1080)
            self.assertEqual(v["fps"], 29.97)
            self.assertEqual(v["runs"][0]["frames_json"], "data/runs/r1.frames.json")
            self.assertEqual(v["runs"][0]["type"], "DYN")
            self.assertEqual(v["trials"][0]["trials_json"], "data/runs/t1.json")

    def test_validate_config_ok(self):

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            ps.validate_config(cfg, repo_root=tmp)  # should not raise

    def test_missing_run_file_raises(self):

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            cfg["videos"][0]["variants"][0]["runs"][0]["path"] = "missing.json"
            with self.assertRaises(ps.ConfigError) as ctx:
                ps.validate_config(cfg, repo_root=tmp)
            self.assertIn("missing.json", str(ctx.exception))

    def test_missing_source_video_raises(self):

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            cfg["videos"][0]["variants"][0]["source"] = str(tmp / "nope.mp4")
            with self.assertRaises(ps.ConfigError):
                ps.validate_config(cfg, repo_root=tmp)


if __name__ == "__main__":
    unittest.main()


class TestCopySite(unittest.TestCase):
    def test_dev_fixtures_and_js_tests_excluded(self):
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            web = tmp / "here" / "web"
            (web / "js").mkdir(parents=True)
            (web / "data" / "videos").mkdir(parents=True)
            (web / "index.html").write_text("<html></html>")
            (web / "js" / "a.js").write_text("export const x = 1;")
            (web / "js" / "a.test.js").write_text("// test")
            (web / "data" / "videos" / "poison.mp4").write_bytes(b"x")
            out = tmp / "dist"
            ps.copy_site(tmp / "here", out)
            self.assertTrue((out / "index.html").exists())
            self.assertTrue((out / "js" / "a.js").exists())
            self.assertFalse((out / "js" / "a.test.js").exists())
            # dev fixture data/ must NOT leak into the deliverable
            self.assertFalse((out / "data" / "videos" / "poison.mp4").exists())
            # but the real data dirs are created empty, ready for packaging
            self.assertTrue((out / "data" / "runs").is_dir())
