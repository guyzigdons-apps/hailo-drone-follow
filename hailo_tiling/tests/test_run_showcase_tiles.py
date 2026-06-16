from tiling_lab.live.run_showcase import build_arg_parser


def test_select_mode_flag_defaults_to_click():
    args = build_arg_parser().parse_args(["--video", "x", "--out", "y"])
    assert args.select_mode == "click"
