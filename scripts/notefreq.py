#! /usr/bin/env python

from __future__ import print_function

notes = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

pitches = []
for n in range(128):
    f = 440.0 * (2 ** ((n - 69) / 12.0))  # A4 = MIDI key 69
    note = notes[n % 12]
    octave = (n // 12) - 1
    pitches.append(f)
    print("%3d: %3s%-2d %.2f Hz" % (n, note, octave, f))

