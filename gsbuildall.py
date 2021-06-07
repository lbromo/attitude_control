#!/usr/bin/env python
# Copyright (c) 2013-2020 GomSpace A/S. All rights reserved.

import gsbuildtools
from gs.buildtools import util
from gs.buildtools import verify_dist_tarball

util.clear_artifacts()
util.status_command()

util.manifest_command()

options = ['--adcs-full']

# release - matlab image
util.waf_command(options)

# Try and generate configuration files from default definition
options = ['gen_config']

# release - matlab image
util.waf_command(options)

util.doc_command()
util.copy_to_artifact(['build/doc/adcssim-getting-started/latex/*.pdf'])
util.copy_to_artifact(['build/doc/adcssim-getting-started/html'],
                      subdir='html_getting_started')

util.cppcheck_command()
util.stylecheck_command()
