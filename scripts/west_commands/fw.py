from west.commands import WestCommand
from west import log
from west import util
import os
import subprocess

def run_cmd(args, env={}):
    newenv = os.environ.copy()
    newenv.update(env)

    p = subprocess.Popen(args, env=newenv)
    p.communicate()

    if p.returncode:
        raise subprocess.CalledProcessError(p.returncode, args)

class IDFBuild(WestCommand):
    def __init__(self, cmdname, cmddesc, projectdir, builddir):
        super().__init__(
            cmdname,
            cmddesc,
            None,
            accepts_unknown_args=True)

        self.top_dir = util.west_topdir()
        self.build_dir = os.path.join(self.top_dir, os.path.join('build', builddir))
        self.project_dir = os.path.join(self.top_dir, projectdir)
        self.idf_path = os.path.join(self.top_dir, 'external/esp-idf')
        self.extra_component_dirs = [
            os.path.join(self.top_dir, 'components'),
            os.path.join(self.top_dir, 'components/usfs/lib'),
        ]
        self.sdkconfig_final = os.path.join(self.build_dir, 'sdkconfig')
        self.sdkconfig_defaults = os.path.join(self.build_dir, 'sdkconfig.defaults')
        self.sdkconfigs = [
            os.path.join(self.project_dir, 'sdkconfig.defaults'),
            os.path.join(self.top_dir, 'sdkconfig.defaults'),
        ]

    def do_add_parser(self, parser_adder):
        return parser_adder.add_parser(self.name,
                                       add_help=False,
                                       description=self.description)

    def sdkconfig_needs_update(self):
        try:
            gentime = os.path.getmtime(self.sdkconfig_defaults)
        except FileNotFoundError:
            return True

        for cfg in self.sdkconfigs:
            try:
                if os.path.getmtime(cfg) > gentime:
                    return True
            except FileNotFoundError:
                pass

        return False

    def gen_sdkconfig(self):
        log.inf('=== regenerate sdkconfig.defaults', colorize=True)

        try:
            with open(self.sdkconfig_defaults, 'w+') as fout:
                for cfg in self.sdkconfigs:
                    try:
                        with open(cfg, 'r') as fin:
                            fout.write(fin.read())
                    except FileNotFoundError:
                        pass
        except:
            if os.path.exists(self.sdkconfig_defaults):
                os.remove(self.sdkconfig_defaults)

            raise

    def run_idfpy(self, addargs):
        os.makedirs(self.build_dir, exist_ok=True)

        # WORKAROUND: make project.cmake happy when we move the config to the builddir
        if not os.path.exists(self.sdkconfig_final):
            open(self.sdkconfig_final, 'a').close()

        if self.sdkconfig_needs_update():
            self.gen_sdkconfig()

        env = {
            'IDF_PATH': self.idf_path,
        }
        cacheentries = {
            'SDKCONFIG': self.sdkconfig_final,
            'SDKCONFIG_DEFAULTS': self.sdkconfig_defaults,
            'EXTRA_COMPONENT_DIRS': ';'.join(self.extra_component_dirs),
            'MQTT_DIR': os.path.join(self.top_dir, 'external/mqtt-c'),
            'BMP280_DIR': os.path.join(self.top_dir, 'external/bmp280'),
        }
        args = [
            os.path.join(self.project_dir, 'scripts/idfpy_wrapper'),
            '-C', self.project_dir,
            '-B', self.build_dir,
            '-G', 'Ninja',
        ]

        for e in cacheentries:
            args.append('--define-cache-entry')
            args.append(e + '=' + cacheentries[e])

        run_cmd(args + addargs, env)

    def do_run(self, args, unknown_args):
        self.run_idfpy(unknown_args)

class Fw(IDFBuild):
    def __init__(self):
        super().__init__('fw', 'build the firmware', 'imulogger', 'imulogger')

class IPerf(IDFBuild):
    def __init__(self):
        super().__init__('iperf', 'build iperf', 'external/esp-idf/examples/wifi/iperf', 'iperf')
