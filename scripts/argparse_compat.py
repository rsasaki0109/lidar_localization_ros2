"""argparse.BooleanOptionalAction on Python 3.8 (ROS 2 Foxy).

Python 3.9 and later (Humble, Jazzy) use the standard library class unchanged.
"""

import argparse

if hasattr(argparse, "BooleanOptionalAction"):
    BooleanOptionalAction = argparse.BooleanOptionalAction
else:

    class BooleanOptionalAction(argparse.Action):
        """--flag sets True and --no-flag sets False, as in Python 3.9."""

        def __init__(self, option_strings, dest, **kwargs):
            negated = []
            for option_string in option_strings:
                negated.append(option_string)
                if option_string.startswith("--"):
                    negated.append("--no-" + option_string[2:])
            super().__init__(negated, dest, nargs=0, **kwargs)

        def __call__(self, parser, namespace, values, option_string=None):
            if option_string in self.option_strings:
                setattr(namespace, self.dest, not option_string.startswith("--no-"))

        def format_usage(self):
            return " | ".join(self.option_strings)
