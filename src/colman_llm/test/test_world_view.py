from colman_llm.world_view import Column, WorldView


def test_status_says_what_the_arm_holds():
    view = WorldView([Column(["tag_1"], 1)], held="tag_2")
    assert "The arm is holding the" in view.status()
    assert "not holding anything" in WorldView([Column(["tag_1"], 1)]).status()
