def main():

    from scripts.commons.Script import Script
    script = Script() #Initialize: load config file, parse arguments, build cpp modules (warns the user about inconsistencies before choosing a test script)

    from scripts.commons.UI import UI
    from scripts.commons.Train_Base import Train_Base
    from os.path import isfile, join, realpath, dirname
    from os import listdir, getcwd
    from importlib import import_module

    _cwd = realpath( join(getcwd(), dirname(__file__)))
    demos_path = _cwd + "/scripts/demos/"
    tests_path = _cwd + "/scripts/tests/"
    gyms_path  = _cwd + "/scripts/gyms/"
    exclusions = ["__init__.py"]

    demos = sorted([f[:-3] for f in listdir(demos_path) if isfile(join(demos_path, f)) and f.endswith(".py") and f not in exclusions])
    tests = sorted([f[:-3] for f in listdir(tests_path) if isfile(join(tests_path, f)) and f.endswith(".py") and f not in exclusions])
    gyms  = sorted([f[:-3] for f in listdir(gyms_path ) if isfile(join(gyms_path , f)) and f.endswith(".py") and f not in exclusions])

    while True:
        # _, col_idx, col = UI.print_table( [demos, tests, gyms], ["DEMOS","TESTS","GYMS"], numbering=[True]*3, prompt='Choose script (ctrl+c to abort): '

        chosen = ("scripts.gyms." , gyms[3])

        mod = import_module(chosen[0]+chosen[1])

        while True:
            mod.Train(script).train(dict())


# allow child processes to bypass this file
if __name__ == "__main__":
    main()