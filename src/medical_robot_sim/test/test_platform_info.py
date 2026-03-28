from medical_robot_sim.platform_info import parse_nv_tegra_release


def test_parse_nv_tegra_release_empty():
    rel = parse_nv_tegra_release("")
    assert rel.raw == ""
    assert rel.r is None
    assert rel.rev is None


def test_parse_nv_tegra_release_extracts_r_and_revision():
    # Example-ish format (varies between Jetson releases)
    text = "# R35 (release), REVISION: 2.1, GCID: 12345678, BOARD: t234ref\n"
    rel = parse_nv_tegra_release(text)
    assert rel.raw.startswith("# R35")
    assert rel.r == "35"
    assert rel.rev == "2.1"


def test_parse_nv_tegra_release_missing_tokens():
    text = "# something without expected tokens\n"
    rel = parse_nv_tegra_release(text)
    assert rel.raw.startswith("# something")
    assert rel.r is None
    assert rel.rev is None
