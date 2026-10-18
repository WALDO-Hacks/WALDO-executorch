"""WALDO KWS v2 vocabulary: labels in model output order, what to say for each, and the
YOLO class each object word tracks (src/yolo/waldo_yolo_classes.h). One place for the data
scripts, training, the export (src/kws_labels.h) and the tests.
"""
# The 10 MicroNet words first, in their old order, so old label indices stay valid.
COMMANDS = ["down", "go", "left", "no", "off", "on", "right", "stop", "up", "yes"]
# object word -> YOLO class index
OBJECTS = {"person": 0, "cup": 14, "bottle": 13, "phone": 24, "laptop": 20, "book": 25,
           "backpack": 11, "keyboard": 23, "dog": 3, "cat": 2}
EXTRA = ["home", "patrol", "hey_waldo"]
LABELS = COMMANDS + list(OBJECTS) + EXTRA + ["_silence_", "_unknown_"]
WORDS = len(LABELS) - 2

# Texts sent to text-to-speech per label (punctuation varies the prosody).
SPOKEN = {w: [w, w.capitalize() + ".", w.capitalize() + "!", w + "?"] for w in LABELS[:WORDS]}
SPOKEN["hey_waldo"] = ["Hey Waldo.", "Hey, Waldo!", "hey waldo", "Hey Waldo?"]

# Speech Commands v0.02 words that are labels here (real speakers).
SC_LABELS = COMMANDS + ["dog", "cat"]

# Near misses, trained as _unknown_.
CONFUSABLE = ["cap", "cop", "pup", "cut", "hope", "foam", "bone", "comb", "look", "cook",
              "lap", "top", "bottom", "model", "bottled", "hey", "Waldo", "hello", "hey Walter",
              "hey Wally", "hello there", "back", "pack", "board", "parole", "petrol", "phony",
              "dock", "cot", "hat", "come", "pearson", "notebook", "book it", "laptop bag"]
# Ordinary talk, for _unknown_ chunks and false-trigger counts.
SENTENCES = [
    "Hi everyone, this is our hackathon project.",
    "It runs entirely on the microcontroller, with no cloud.",
    "Let me show you what happens when somebody walks in.",
    "Could you pass me the water, please?",
    "We trained the model this morning and it seems to work.",
    "The camera sends compressed frames over a serial link.",
    "I think we should grab some lunch after the demo.",
    "Thanks for coming by, do you have any questions?",
    "The battery is almost empty again.",
    "Put that back on the shelf when you are finished.",
]
