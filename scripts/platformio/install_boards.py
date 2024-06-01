#!/usr/bin/env python

#
# install_boards.py:
#
#   Install the ESP32-3248S035 package-provided board definition(s) to the
#   PlatformIO core "boards_dir" directory (e.g., "~/.platformio/boards").
#
#   No arguments are accpted, and no output is generated if successful.
#
#   Requires the "pio" command for resolving core configuration.
#
#   By default, each JSON board definition is installed as a symlink that
#   points back to this package's "boards" directory, relative to this
#   script being executed. It silently replaces any conflicting JSON board 
#   definitions that may already exist in the core directory.
#
#   If symlink creation fails or is unsupported, it will fallback to ordinary
#   file copy (again, silently replacing any conflicts). If an environment 
#   variable named "PLATFORMIO_DISABLE_BOARD_SYMLINKS" exists (even if empty),
#   then no symlinks will be created and only file copy will be attempted.
#

from json import ( loads as parse )
from os import ( environ as env )
from pathlib import ( Path as fs )
from shutil import ( copy as cp )
from subprocess import ( run as call, PIPE as fifo )
from sys import ( exit as halt )
from traceback import ( format_exc as unwind )

# Call pio to retrieve core dir. For some reason, we cannot access the SCons
# build environment via `Import("env")` or `DefaultEnvironment()` — probably
# because we don't necessarily have a build env if just installing a package.
cmd = [ 'pio', 'system', 'info', '--json-output' ]

symlink_disabled = "PLATFORMIO_DISABLE_BOARD_SYMLINKS" in env

try:
    # Parse JSON output from 'pio system info'
    result = parse(call(cmd, stdout=fifo, text=True).stdout)
    # Construct path to core boards_dir and create it if necessary
    boards = fs(result['core_dir']['value']) / 'boards'
    boards.mkdir(parents=True, exist_ok=True)

    # Construct paths to all board definitions included with this package
    source = fs(__file__).parent / '..' / '..' / 'boards'
    for src in source.resolve().glob('**/*.json'):
        dst = boards / src.name
        # Remove existing board definition if it already exists
        #  (esp., an orphaned symlink pointing to a previous project)
        dst.unlink(missing_ok=True)
        try:
            # Skip to file copy if forced via flag
            if symlink_disabled: raise Exception
            # Create a symlink in core boards_dir pointing to source
            dst.symlink_to(src)
        except:
            # Resort to file copy if linking fails or is not supported
            cp(src, dst)

except:
    halt(unwind())
