from brachiograph import BrachioGraph

bg = BrachioGraph(
    # the lengths of the arms
    inner_arm=10,
    outer_arm=10,
    wait=0.35,
    # the drawing area
    bounds=(-0, 5, 10, 15),
    angle_up=115,
    angle_down=44,
)
