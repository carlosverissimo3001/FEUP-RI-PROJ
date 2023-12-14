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
        _, col_idx, col = UI.print_table( [demos, tests, gyms], ["DEMOS","TESTS","GYMS"], numbering=[True]*3, prompt='Choose script (ctrl+c to abort): ' )

        is_gym = False
        if col == 0:                                               # it's a demo
            chosen = ("scripts.demos." , demos[col_idx])
        elif col == 1:                                             # it's a test
            chosen = ("scripts.tests." , tests[col_idx])
        else:                                                      # it's a gym
            chosen = ("scripts.gyms." , gyms[col_idx])
            is_gym = True


        cls_name = chosen[1]
        mod = import_module(chosen[0]+chosen[1])

        '''
        An imported script should not automatically execute the main code because:
            - Multiprocessing methods, such as 'forkserver' and 'spawn', will execute the main code in every child
            - The script can only be called once unless it is reloaded
        '''
        if not is_gym:
            ''' 
            Demos and tests receive a script support object with user-defined arguments and useful methods
            Each demo or test must implement an execute() method, which may or may not return
            '''
            obj = getattr(mod,cls_name)(script)
            obj.execute()
            del obj
        else:
            ''' 
            Gyms must implement a class Train() which is initialized with user-defined arguments and implements:
                train() - method to run the optimization and save a new model
                test(folder_dir, folder_name, model_file) - method to load an existing model and test it
            '''
            while True:
                idx = UI.print_table([["Abort  ","Train  ","Test   ","Retrain"]], alignment=["^"], numbering=[True], prompt='Choose option: ')[0]
                if idx == 0: break

                if idx == 1:
                    mod.Train(script).train(dict())
                else:
                    model_info = Train_Base.prompt_user_for_model()
                    if model_info is not None and idx == 2:
                        mod.Train(script).test(model_info)
                    elif model_info is not None:
                        mod.Train(script).train(model_info)


# allow child processes to bypass this file
if __name__ == "__main__":
    main()