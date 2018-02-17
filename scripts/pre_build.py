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
 */
#ifndef VERSION_GIT_H
#define VERSION_GIT_H

#define GIT_COMMIT_VERSION  \"{}\"
#define GIT_COMMIT_SHA      \"{}\"

#define GIT_VERSION_MAJOR   ({})
#define GIT_VERSION_MINOR   ({})
#define GIT_VERSION_BUGFIX  ({})

#endif

"""

GIT_VERSION_FILE_PATH = "../../../src/app/version_git.h"

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
    # Get the git SHA.
    try:
        git_sha = check_output("git rev-parse --verify HEAD", shell=True)
        git_sha = git_sha.strip()
    except:
        print("#> ERROR: Failed to get git SHA")
        git_sha = "<unknown>"

    # Get version tag.
    if len(sys.argv) > 1:
        tagPrefix = sys.argv[1]
    else:
        tagPrefix = ""
    if len(tagPrefix):
        tagPrefix += "-"
    try:
        git_vers = check_output("git describe --match '{}v*' --always".format(tagPrefix), shell=True)
        git_vers = git_vers.strip()
        if len(tagPrefix) and git_vers.startswith(tagPrefix):
            git_vers = git_vers[len(tagPrefix):]
    except CalledProcessError, WindowsError:
        git_vers = "v0.0.0"

    # Parse the version tag.
    try:
        fields = parse_version(git_vers)
    except ValueError:
        fields = [0, 0, 0]

    print("#> git: {}; version={}.{}.{}; sha={}".format(git_vers, fields[0], fields[1], fields[2], git_sha))

    # Generate output from template.
    output = VERSION_GIT_FILE_TEMPLATE.format(git_vers, git_sha, fields[0], fields[1], fields[2])

    # Read existing file content.
    try:
        with open(GIT_VERSION_FILE_PATH, 'r') as f:
            content = f.read()
    except OSError:
        content = None

    # Only write file if the content has changed, so we don't always retrigger a build.
    if output != content:
        # Create the version file.
        with open(GIT_VERSION_FILE_PATH, 'w+') as version_file:
            version_file.write(output)

    return 0

if __name__ == "__main__":
    exit(pre_build())
