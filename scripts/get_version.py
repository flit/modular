#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# DAPLink Interface Firmware
# Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
# Copyright (c) 2018 Immo Software
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from __future__ import absolute_import
from __future__ import print_function

import sys
import os
import argparse
from subprocess import check_output, CalledProcessError

VERSION_GIT_FILE_TEMPLATE = """
/*
 * Copyright (c) 2018 Immo Software
 *
 * This file is auto-generated. Do not edit by hand!
 */
#ifndef VERSION_GIT_H
#define VERSION_GIT_H

#define GIT_COMMIT_VERSION  \"{full}\"
#define GIT_COMMIT_SHA      \"{sha}\"

#define GIT_VERSION_MAJOR   ({major})
#define GIT_VERSION_MINOR   ({minor})
#define GIT_VERSION_BUGFIX  ({bugfix})

#endif

"""

def parse_version(git_vers):
    git_vers = git_vers.split('-')[0]
    if not git_vers.startswith('v'):
        raise ValueError("incorrect version format")
    versionFields = git_vers[1:].split('.')[:3]
    if len(versionFields) < 3:
        versionFields += ["0"] * (3 - len(versionFields))
    versionFields = [int(f) for f in versionFields]
    return versionFields

def pre_build():
    parser = argparse.ArgumentParser(description='Git version extractor')
    parser.add_argument("-p", "--prefix", type=str, default="", help="Git tab prefix.")
    parser.add_argument("output", type=str, help="Output file name.")
    args = parser.parse_args()

    # Tag prefix argument
    tagPrefix = args.prefix
    if len(tagPrefix) and not tagPrefix.endswith("-"):
        tagPrefix += "-"

    # Output file argument.
    outputFilePath = args.output

    # Get the git SHA.
    try:
        git_sha = check_output("git rev-parse --verify HEAD", shell=True)
        git_sha = git_sha.strip()
    except CalledProcessError:
        print("#> ERROR: Failed to get git SHA")
        git_sha = "<unknown>"

    # Get version tag.
    try:
        git_vers = check_output("git describe --match '{}v*' --always".format(tagPrefix), shell=True)
        git_vers = git_vers.strip()
        # Convert to string for python3.
        if not isinstance(git_vers, str):
            git_vers = git_vers.decode('utf-8')
        if len(tagPrefix) and git_vers.startswith(tagPrefix):
            git_vers = git_vers[len(tagPrefix):]
    except (CalledProcessError, OSError):
        git_vers = "v0.0.0"

    # Parse the version tag.
    try:
        fields = parse_version(git_vers)
    except ValueError:
        fields = [0, 0, 0]

    # Create format data dictionary.
    format_dict = {
        'full' : git_vers,
        'sha' : git_sha,
        'major' : fields[0],
        'minor' : fields[1],
        'bugfix' : fields[2],
        }

    print("#> git: {full}; version={major}.{minor}.{bugfix}; sha={sha}".format(**format_dict))

    # Generate output from template.
    output = VERSION_GIT_FILE_TEMPLATE.format(**format_dict)

    # Read existing file content.
    try:
        with open(outputFilePath, 'r') as f:
            content = f.read()
    except (OSError, IOError):
        content = None

    # Only write file if the content has changed, so we don't always retrigger a build.
    if output != content:
        # Create the version file.
        with open(outputFilePath, 'w+') as version_file:
            version_file.write(output)

    return 0

if __name__ == "__main__":
    exit(pre_build())
