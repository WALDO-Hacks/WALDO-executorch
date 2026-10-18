"""WALDO KWS v2 vocabulary: labels in model output order, what to say for each, and the
YOLO class each object word tracks (src/yolo/waldo_yolo_classes.h). One place for the data
scripts, training, the export (src/kws_labels.h) and the tests.
"""
# The 10 MicroNet words first, in their old order, so old label indices stay valid.
COMMANDS = ["down", "go", "left", "no", "off", "on", "right", "stop", "up", "yes"]
# object word -> YOLO class index
# Things at the venue (r4): person, dog and cat dropped (no animals; "person" sounded like "no"),
# chair added. Their old clips train as _unknown_ (DROPPED).
OBJECTS = {"cup": 14, "bottle": 13, "phone": 24, "laptop": 20, "book": 25,
           "backpack": 11, "keyboard": 23, "chair": 15}
DROPPED = ["person", "dog", "cat"]
EXTRA = ["home", "patrol", "hey_waldo"]
LABELS = COMMANDS + list(OBJECTS) + EXTRA + ["_silence_", "_unknown_"]
WORDS = len(LABELS) - 2

# Texts sent to text-to-speech per label (punctuation varies the prosody).
SPOKEN = {w: [w, w.capitalize() + ".", w.capitalize() + "!", w + "?"] for w in LABELS[:WORDS]}
SPOKEN["hey_waldo"] = ["Hey Waldo.", "Hey, Waldo!", "hey waldo", "Hey Waldo?"]
# "cup" alone is one short syllable next to "up": longer forms, all one label.
SPOKEN["cup"] = ["cup", "Cup.", "mug", "Mug!", "paper cup", "Paper cup.", "plastic cup", "Plastic cup!",
                 "coffee cup", "Coffee cup."]
SPOKEN["bottle"] = ["bottle", "Bottle.", "water bottle", "Water bottle!", "Water bottle."]
SPOKEN["phone"] = ["phone", "Phone!", "cell phone", "Cell phone.", "Cell phone!"]
# Labels with multi-word forms: no word-fragment negatives (the end of "paper cup" is "cup").
MULTI = {w for w in LABELS[:WORDS] if w != "hey_waldo" and any(" " in t for t in SPOKEN[w])}

# Speech Commands v0.02 words that are labels here (real speakers).
SC_LABELS = list(COMMANDS)

# Near misses, trained as _unknown_.
CONFUSABLE = ["cap", "cop", "pup", "cut", "hope", "foam", "bone", "comb", "look", "cook",
              "lap", "top", "bottom", "model", "bottled", "hey", "Waldo", "hello", "hey Walter",
              "hey Wally", "hello there", "back", "pack", "board", "parole", "petrol", "phony",
              "dock", "cot", "hat", "come", "pearson", "notebook", "book it", "laptop bag",
              "paper", "plastic", "coffee", "water", "cell", "share", "chain", "cheer"]
# Near misses that contain a label word: neither trained as _unknown_ nor counted as false triggers.
CONTAINS_LABEL = {"book it", "laptop bag", "notebook"}
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
