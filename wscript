#!/usr/bin/env python
# encoding: utf-8
# Copyright (c) 2013-2020 GomSpace A/S. All rights reserved.

import os
import gs_util
import gs_doc
import gs_gcc
import gs.buildtools.util
import gs_dist

from waflib.Build import BuildContext

APPNAME = 'JuliaSim'

# Scan modules in local lib dir
modules = ['lib/libadcssim']


def get_build_info():
    return gs.buildtools.util.get_build_info(appName=APPNAME)


def options(ctx):
    ctx.load('gs_gcc gs_doc')
    gs_gcc.gs_recurse(ctx)
    ctx.recurse(modules)

    if os.path.exists('../tools/eclipse.py'):
        ctx.load('eclipse', tooldir='../tools/')

    gr = ctx.add_option_group("A3200 SDK options")
    gr.add_option('--emul-part', action='store',
                  default='uc3c0512c', help='MCU to emulate')
    gr.add_option('--doc-man', action='store',
                  default='True', help='Build manual')


def configure(ctx):
    ctx.load('gs_gcc gxx')

    ctx.env.append_unique('CFLAGS',
                          [
                              '-std=c99',
                              '-g',
                              '-DMATLAB_MEX_FILE',
                              '-D_GNU_SOURCE',
                              '-fexceptions',
                              '-fPIC',
                              '-fno-omit-frame-pointer',
                              '-pthread',
                              '-fprofile-arcs',
                              '-ftest-coverage'
                          ])

    ctx.env.append_unique(
        'CPPFLAGS', '-D_GNU_SOURCE -fPIC -fno-omit-frame-pointer -pthread'.split(' '))
    ctx.env.append_unique('LDFLAGS',
                          [
                              '-Wl,--whole-archive',
                              '-Wl,--allow-multiple-definition',
                              '-Wl,--no-whole-archive',
                              '-lgcov'
                          ])

    # ctx.env.append_unique('LIBS', 'mx mex mat m stdc++'.split(' '))

    # ctx.env.append_unique('FILES_CSPTERM', 'src/*.c')
    # ctx.env.append_unique('LIBS_CSPTERM', ['rt', 'pthread', 'elf', 'stdc++', 'm'])

    # Options for CSP
    ctx.options.with_os = 'posix'
    ctx.options.enable_rdp = True
    ctx.options.enable_qos = True
    ctx.options.enable_crc32 = True
    ctx.options.enable_hmac = True
    ctx.options.enable_xtea = True
    ctx.options.enable_promisc = True
    ctx.options.enable_if_kiss = True
    ctx.options.enable_if_can = False
    ctx.options.enable_if_zmqhub = False
    ctx.options.disable_stlib = True
    ctx.options.with_rtable = 'cidr'
    ctx.options.with_driver_can = 'socketcan'
    ctx.options.with_driver_usart = 'linux'

    # Options for libgosh
    ctx.options.gosh_redirect = True
    ctx.options.gosh_app = True

    # Options for liblog
    ctx.options.enable_log_node = True

    # Options for libutil
    ctx.options.clock = 'linux'
    ctx.options.with_log = 'printf'
    ctx.options.enable_vmem = True
    ctx.options.enable_lzo = True

    # Options for libadcs
    ctx.options.enable_adcs = True
    ctx.options.enable_adcs_client = True
    ctx.options.enable_adcs_server = True
    ctx.options.adcs_platform = 'matlab'
    ctx.options.enable_astro_wde = True
    ctx.options.adcs_full = True
    ctx.options.adcs_enable_htpa = True
    ctx.options.adcs_enable_julia = True

    # Options for libparam
    ctx.options.enable_param_local_client = True
    ctx.options.enable_param_csp_server = True
    ctx.options.enable_param_csp_client = True
    ctx.options.enable_param_server = True
    ctx.options.enable_param_cmd = True
    ctx.options.param_backend = 'file'
    ctx.options.param_table_size = 1024
    ctx.options.param_lock = 'none'
    ctx.options.param_max_tables = 40

    # Document manual
    ctx.options.doc_man = True

    # Clients
    # ctx.env.append_unique('USES_ADCS', ['p60-pdu-client', 'libp60-client', 'nanopower-client', 'libadcs-client'])
    # ctx.options.with_clients = 'libadcs_client'

    # Add version/revision info
    bi = gs.buildtools.util.get_build_info(appName=APPNAME)
    ctx.options.revision = bi.revision()

    # ctx.define('CSPTERM_VERSION', VERSION)

    # Recurse and write config
    ctx.write_config_header('include/conf_cspterm.h', top=True, remove=True)
    # ctx.recurse(modules, mandatory=False)
    gs_gcc.gs_recurse(ctx)


def build(ctx):
    # ctx(export_includes=['include', 'client', 'clients'], name='include')
    # ctx.recurse(modules, mandatory=False)
    gs_gcc.gs_recurse(ctx)

    # use = ['gscsp', 'gosh', 'param', 'util', 'adcs', 'log']
    use = [t.name for t in ctx.get_all_task_gen() if hasattr(t, 'typ')
           and t.typ == 'objects']

    ctx.stlib(
        source=ctx.path.ant_glob('lib/libadcssim/src/target.c'),
        target='shadcs',
        use=use,
        linkflags=ctx.env.LDFLAGS
        # lib=ctx.env.LIBS_CSPTERM + ctx.env.LIBS
    )

    ctx.shlib(
        source=ctx.path.ant_glob('lib/libadcssim/src/target.c'),
        target='shadcs',
        use=use,
        linkflags=ctx.env.LDFLAGS
        # lib=ctx.env.LIBS_CSPTERM + ctx.env.LIBS
    )



def gen_config(ctx):
    ctx.recurse(modules)


def doc(ctx):
    sphinx_tags = []

    # if ctx.options.doc_man:
    #     gs_doc.manual(ctx, keyvalues={
    #         'gs_prod_group': 'ADCSSIM',
    #         'gs_prod_name': 'GETTING STARTED',
    #         'gs_prod_desc': 'ADCS Simulation Environment Documentation',
    #         'gs_output_name': 'adcssim-getting-started',
    #         'gs_ifs_doc_number': '107558',
    #         'gs_sphinx_exclude': ['lib/libadcs/**', 'lib/libemul/**', 'lib/libgosh/**', 'tools/**'],
    #         'gs_master_doc': 'lib/libadcssim/doc/index',
    #         'gs_front_image': os.path.abspath('doc/img/adcs_front_page_high.png'),
    #         'gs_sphinx_tags': sphinx_tags},
    #         doxygen=False
    #     )

    # gs_doc.product_brief(ctx, keyvalues={
    #     'gs_prod_group': 'ADCS',
    #     'gs_prod_name': 'ADCS Simulation Report - Starling',
    #     'gs_prod_desc': 'Mission description and Simulation report',
    #     'gs_output_name': 'adcs_report',
    #     'gs_sphinx_exclude': ['lib/libadcs/**', 'lib/libemul/**', 'lib/libgosh/**', 'tools/**'],
    #     'gs_master_doc': 'doc/index',
    #     'gs_front_image': os.path.abspath('doc/img/adcs_front_page_high.png'),
    #     'gs_sphinx_tags': sphinx_tags},
    #     doxygen=False
    # )


def gs_dist(ctx):
    filename = 'gs-'
    name = 'adcssim-2019a'

    ctx.set_appname(filename + name)

    ctx.source_modules = ['lib/libadcssim']
    ctx.add_default_files(source_module=True)

    ctx.add_file_to_dir('build', '')

    ctx.add_file_to_dir('mission/mission.json', 'mission')

    ctx.add_file_to_dir('lib/libadcssim/mission_builder',
                        'lib/libadcssim/')

    ctx.add_file_to_dir('lib/libadcssim/visualization/pictures',
                        'lib/libadcssim/visualization/')

    ctx.add_file_to_dir(ctx.path.ant_glob('lib/libadcssim/visualization/animation/*.p'),
                        'lib/libadcssim/visualization/animation')

    ctx.add_file_to_dir(ctx.path.ant_glob('lib/libadcssim/utils/*.p'),
                        'lib/libadcssim/utils')

    ctx.add_file_to_dir('lib/libadcssim/sim/components_lib.slx',
                        'lib/libadcssim/sim')

    ctx.add_file_to_dir('lib/libadcssim/models/gs_demomodel_with_onb.slx',
                        'lib/libadcssim/models')

    ctx.add_file_to_dir('lib/libadcssim/bin',
                        'lib/libadcssim/')

    ctx.add_file_to_dir('lib/libadcssim/src',
                        'lib/libadcssim/')

    ctx.add_file_to_dir('lib/libadcssim/wscript',
                        'lib/libadcssim')

    ctx.add_file_to_dir('lib/libadcssim/external_tools/build_mex.py',
                        '')

    ctx.add_file_to_dir('build/lib/libgosh/gosh',
                        '')

    ctx.add_file_to_dir('build/lib/libgosh/libgsgosh.so',
                        '')

    ctx.add_file_to_dir('lib/libadcssim/startup.m',
                        '')

    # Include libemul
    ctx.add_file_to_dir('lib/libemul/wscript', 'lib/libemul')
    ctx.add_file_to_dir('lib/libemul/CHANGELOG.rst', 'lib/libemul')
    ctx.add_file_to_dir('lib/libemul/README.md', 'lib/libemul')
    ctx.add_file_to_dir('lib/libemul/avr32', 'lib/libemul')
    ctx.add_file_to_dir('lib/libemul/avr8', 'lib/libemul')

    ctx.recurse('tools/buildtools')

    gs_gcc.gs_recurse(ctx)
