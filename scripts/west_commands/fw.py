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

class Fw(WestCommand):
    def __init__(self):
        super().__init__(
            'fw',
            'build the firmware',
            None,
            accepts_unknown_args=True)

        self.top_dir = util.west_topdir()
        self.build_dir = os.path.join(self.top_dir, 'build')
        self.project_dir = os.path.join(self.top_dir, 'imulogger')
        self.idf_path = os.path.join(self.top_dir, 'external/esp-idf')
        self.sdkconfig_final = os.path.join(self.build_dir, 'sdkconfig')

    def do_add_parser(self, parser_adder):
        return parser_adder.add_parser(self.name,
                                       add_help=False,
                                       description=self.description)

    def run_idfpy(self, addargs):
        os.makedirs(self.build_dir, exist_ok=True)

        # WORKAROUND: make project.cmake happy when we move the config to the builddir
        if not os.path.exists(self.sdkconfig_final):
            open(self.sdkconfig_final, 'a').close()

        env = {
            'IDF_PATH': self.idf_path,
        }
        args = [
            os.path.join(self.idf_path, 'tools/idf.py'),
            '-C', self.project_dir,
            '-B', self.build_dir,
            '-G', 'Ninja',
            '-DSDKCONFIG=' + self.sdkconfig_final
        ]
        run_cmd(args + addargs, env)

    def do_run(self, args, unknown_args):
        self.run_idfpy(unknown_args)
