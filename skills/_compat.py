"""Load a helpers module whether it is named helpers/<name>.py or helpers/helpers_<name>.py."""

import importlib


def load(name):
    tried = []
    for mod in (f"helpers.{name}", f"helpers.helpers_{name}"):
        try:
            return importlib.import_module(mod)
        except ModuleNotFoundError as exc:
            if exc.name != mod:      # a different module is missing (e.g. protobuf): surface it
                raise
            tried.append(mod)
    raise ModuleNotFoundError(
        f"Could not import any of {tried}. Check the file names in your helpers/ folder.")
