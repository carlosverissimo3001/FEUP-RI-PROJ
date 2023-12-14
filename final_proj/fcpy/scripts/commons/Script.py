import argparse,json,sys
from os import path, listdir, getcwd, cpu_count
from os.path import join, realpath, dirname, isfile, isdir, getmtime
from scripts.commons.UI import UI
import subprocess
import __main__
import pickle

class Script():
    ROOT_DIR = path.dirname(path.dirname(realpath( join(getcwd(), dirname(__file__))) )) # fcpy's root directory

    def __init__(self, cpp_builder_unum=0, hide_UI=False) -> None:

        '''
        Arguments specification
        -----------------------
        - To add new arguments, edit the information below
        - After changing information below, the config.json file must be manually deleted
        - In other modules, these arguments can be accessed by their 1-letter ID
        '''
        # list of arguments: 1-letter ID, Description, Hardcoded default
        self.options = {'i': ('Server Hostname/IP', 'localhost'),
                        'p': ('agent_port',         '3100'),
                        'm': ('monitor_port',       '3200'),
                        't': ('team_name',          'FCPortugal'),
                        'u': ('uniform_number',     '1'),
                        'r': ('robot_type',         '4')}

        # list of arguments: 1-letter ID, data type, choices      
        self.op_types = {'i': (str, None),
                         'p': (int, None),
                         'm': (int, None),
                         't': (str, None),
                         'u': (int, range(1,12)),
                         'r': (int, [0,1,2,3,4])}

        '''
        End of arguments specification
        '''

        self.read_or_create_config()

        #advance help text position
        formatter = lambda prog: argparse.HelpFormatter(prog,max_help_position=52)
        parser = argparse.ArgumentParser(formatter_class=formatter)

        o = self.options
        t = self.op_types

        for id in self.options: # shorter metavar for aesthetic reasons
            parser.add_argument(f"-{id}", help=f"{o[id][0]:30}[{o[id][1]:20}]", type=t[id][0], nargs='?', default=o[id][1], metavar='X', choices=t[id][1])
        
        self.args = parser.parse_args()

        self.players = [] # list of created players

        Script.build_cpp_modules(exit_on_build = (cpp_builder_unum != 0 and cpp_builder_unum != self.args.u))

        if not hide_UI:
            try:
                print(f"\nNOTE: for help run \"python {__main__.__file__} -h\"")
            except:
                pass

            columns = [[],[],[]]
            for key, value in vars(self.args).items():
                columns[0].append(o[key][0])
                columns[1].append(o[key][1])
                columns[2].append(value)

            UI.print_table(columns, ["Argument","Default at /config.json","Active"], alignment=["<","^","^"])


    def read_or_create_config(self) -> None:

        if not path.isfile('config.json'):      # Save hardcoded default values if file does not exist
            with open("config.json", "w") as f:
                json.dump(self.options, f)
        else:                                   # Load user-defined values (that can be overwritten in the terminal)
            with open("config.json", "r") as f:
                self.options = json.loads(f.read())


    @staticmethod
    def build_cpp_modules(special_environment_prefix=[], exit_on_build=False):
        '''
        Build C++ modules in folder /cpp using Pybind11
        
        Parameters
        ----------
        special_environment_prefix : `list`
            command prefix to run a given command in the desired environment
            useful to compile C++ modules for different python interpreter versions (other than default version)
            Conda Env. example: ['conda', 'run', '-n', 'myEnv']
            If [] the default python interpreter is used as compilation target
        exit_on_build : bool
            exit if there is something to build (so that only 1 player per team builds c++ modules)
        '''
        cpp_path = Script.ROOT_DIR + "/cpp/"
        exclusions = ["__pycache__"]

        cpp_modules = [d for d in listdir(cpp_path) if isdir(join(cpp_path, d)) and d not in exclusions]

        if not cpp_modules: return #no modules to build

        python_cmd = f"python{sys.version_info.major}.{sys.version_info.minor}" # "python3" can select the wrong version, this prevents that

        def init():
            print("--------------------------\nC++ modules:",cpp_modules)

            try:
                process = subprocess.Popen(special_environment_prefix+[python_cmd, "-m", "pybind11", "--includes"], stdout=subprocess.PIPE)
                (includes, err) = process.communicate()
                process.wait()
            except:
                print(f"Error while executing child program: '{python_cmd} -m pybind11 --includes'")
                exit()

            includes = includes.decode().rstrip() # strip trailing newlines (and other whitespace chars)
            print("Using Pybind11 includes: '",includes,"'",sep="")
            return includes

        nproc = str(cpu_count())
        zero_modules = True

        for module in cpp_modules:
            module_path = join(cpp_path, module)

            # skip module if there is no Makefile (typical distribution case)
            if not isfile(join(module_path, "Makefile")):
                continue

            # skip module in certain conditions
            if isfile(join(module_path, module+".so")) and isfile(join(module_path, module+".c_info")):
                with open(join(module_path, module+".c_info"), 'rb') as f:
                    info = pickle.load(f)
                if info == python_cmd:
                    code_mod_time = max(getmtime(join(module_path, f)) for f in listdir(module_path) if f.endswith(".cpp") or f.endswith(".h"))
                    bin_mod_time = getmtime(join(module_path, module+".so"))
                    if bin_mod_time + 30 > code_mod_time: # favor not building with a margin of 30s (scenario: we unzip the fcpy project, including the binaries, the modification times are all similar)
                        continue

            # init: print stuff & get Pybind11 includes
            if zero_modules:
                if exit_on_build:
                    print("There are C++ modules to build. This player is not allowed to build. Aborting.")
                    exit()
                zero_modules = False
                includes = init()

            # build module
            print(f'{f"Building: {module}... ":40}',end='',flush=True)
            process = subprocess.Popen(['make', '-j'+nproc, 'PYBIND_INCLUDES='+includes], stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=module_path)
            (output, err) = process.communicate()
            exit_code = process.wait()
            if exit_code == 0:
                print("success!")
                with open(join(module_path, module+".c_info"),"wb") as f: # save python version
                    pickle.dump(python_cmd, f, protocol=4) # protocol 4 is backward compatible with Python 3.4
            else:
                print("Aborting! Building errors:")
                print(output.decode(), err.decode())
                exit()     

        if not zero_modules:
            print("All modules were built successfully!\n--------------------------")


    def batch_create(self, agent_cls, args_per_player):    
        ''' Creates batch of agents '''

        for a in args_per_player:
            self.players.append( agent_cls(*a) )

    def batch_execute_agent(self):  
        ''' Executes agent normally (including commit & send) '''   
        
        for p in self.players:
            p.think_and_send()

    def batch_execute_behavior(self, behavior):
        ''' Executes behavior '''

        for p in self.players:
            p.world.robot.behavior.execute(behavior)

    def batch_commit_and_send(self):
        ''' Commits & sends data to server '''
        for p in self.players:
            p.scom.commit_and_send( p.world.robot.get_command() ) 

    def batch_receive(self):
        ''' Waits for server messages '''
        for p in self.players:
            p.scom.receive()

    def batch_commit_beam(self, pos2d_rotation):
        '''
        Beam all player to 2D position with a given rotation

        Parameters
        ----------
        pos3d_rotation : `list`
            iterable of 2D positions and rotations e.g. [(0,0,45),(-5,0,90)]
        '''        
        for p, pos_rot in zip(self.players,pos2d_rotation): 
            p.scom.commit_beam(pos_rot[0:2],pos_rot[2])

    def batch_unofficial_beam(self, pos3d_rotation):
        '''
        Beam all player to 3D position with a given rotation

        Parameters
        ----------
        pos3d_rotation : `list`
            iterable of 3D positions and rotations e.g. [(0,0,0.5,45),(-5,0,0.5,90)]
        '''        
        for p, pos_rot in zip(self.players,pos3d_rotation): 
            p.scom.unofficial_beam(pos_rot[0:3],pos_rot[3])

    def batch_close(self):
        '''
        Close all sockets connected to the agent port
        For scripts where the agent lives until the appliction ends, this is not needed
        '''
        for p in self.players:
            p.scom.close()
        self.players = [] #reset list of players

            
            












