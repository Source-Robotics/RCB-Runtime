"""Recording training data on a PC: each robot's packets (every control tick) and its camera, episode by episode.

Run it as a program (`python -m robotclient.recording --help`), or use `Recorder` from Python. The format is chosen
when it starts (`--format`): ``mcap`` (the default: everything exactly as received, one MCAP file per episode; needs
``pip install mcap``). More formats are modules named in `recorder.FORMATS`. Nothing here is imported by
`import robotclient`, so a PC without the mcap package still uses the rest of the client.

The design: TODO/p4_recording_plan.md section 3.1.
"""
from .recorder import DEFAULT_FORMAT, FORMATS, Recorder, load_formats

__all__ = ["Recorder", "load_formats", "FORMATS", "DEFAULT_FORMAT"]
