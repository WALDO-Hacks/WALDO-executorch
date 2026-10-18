"""WALDO's 28 YOLOv8 classes (from the team's vision hand-off, host/export/classes.py).

Order = output channel 64 + i. person and the 10 animals first, then the room objects.
"""
COCO_NAMES = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign parking_meter bench "
              "bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee skis snowboard "
              "sports_ball kite baseball_bat baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon "
              "bowl banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed dining_table "
              "toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink refrigerator book clock vase scissors "
              "teddy_bear hair_drier toothbrush").split()
assert len(COCO_NAMES) == 80

LIVING = ["person", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"]
TARGETS = ["backpack", "handbag", "bottle", "cup", "chair", "couch", "potted_plant", "dining_table",
           "tv", "laptop", "mouse", "remote", "keyboard", "cell_phone", "book", "clock", "scissors"]
KEPT_NAMES = LIVING + TARGETS
KEPT_IDS = [COCO_NAMES.index(n) for n in KEPT_NAMES]
