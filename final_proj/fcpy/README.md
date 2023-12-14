# FCPy

---

Scripts

Scripts are the entry points to execute the official player, tests/demos, optimizations (machine learning)

    Scripts that run the official player or the entire team, or a part of the team are placed in the root folder.
    The idea is to have a clean and automated workflow, that works without user input, and that can be compiled for official matches.

    There are other types of scripts that are not for official matches, that are placed in the "/scripts" folder:
    - "/scripts/demos" are demonstration scripts to showcase a key feature of the team/agents
    - "/scripts/tests" are test scripts to check if a certain feature is still working
    - "/scripts/gyms" are gym environments for machine learning

    To launch a demo, test or gym, run 'Script_Tests.py'

    To create a demo or test, add a new python file to "/scripts/demos" or "/scripts/tests" with 
    a class that has the same name as the file and a method called "execute()".
    To create a gym environment add a python file to "/scripts/gyms" that defines:
        A class Train() which is initialized with user-defined arguments and implements:
            train() - method to run the optimization and save a new model
            test(folder_dir, folder_name, model_file) - method to load an existing model and test it
        Example: /scripts/gyms/Fall.py

---

Joint Control

    Joint names:

        0       Head horiz:  right  (-120)/(+120)   left   
        1       Head vert :  down    (-45)/(+45)      up
        ------------------------------------------------
        2/3     Leg twist :  fwd/out (-90)/(+1)  reverse   
        4/5     Leg side  :  in      (-25)/(+45)     out   
        6/7     Leg front :  back    (-25)/(+100)  front   
        8/9     Knee      :  back   (-130)/(+1)    front
        ------------------------------------------------
        10/11   Foot vert :  down    (-45)/(+75)      up   
        12/13   Foot roll :  in      (-45)/(+25)     out
        ------------------------------------------------   
        14/15   Arm front :  down   (-120)/(+120)     up   
        16/17   Arm side  :  in       (-1)/(+95)     out   
        18/19   Arm twist :  in     (-120)/(+120)    out   
        20/21   Elbow     :  down     (-1)/(+90)      up 
        ------------------------------------------------  
        22/23   toe       :  down     (-1)/(+70)      up   

        notes: 
        - the index order is always left/rigth
        - all joint pairs are corrected by the World_Parser to be exactly symmetrical
        - the last 2 joints exist only on NAO 4
        - these are the perceptor names used to sense the position of each joint to control these joints, 
          the server wants the names of the effectors, which are slightly different however, they are handled 
          internally, the developer can always use the perceptor names


    How to control the joints:

    - Position control (direct) 
        function:  set_joints_target_position_direct(name, value, max_speed=7)
        objective: the joint will move as fast as possible (or at max_speed)
        notes:     this function compensates for the server's 1 cycle delay by predicting 
                   the current position using the last sent command
        output:    robot.joints_target_speed
    - Position control (PID)
        Not Implemented
    - Speed control in rads/s
        change robot.joints_target_speed directly

    Notes:
    - After sending robot.joints_target_speed to the server, the target speed is zeroed by default for every joint.
      This is performed locally, since the server will maintain the last speed if nothing is sent. 
    - The preferred control method must be called every step. If two control methods are called in one step, and 
      some joints were defined twice, those joints will be overwritten by the second method.

---

Reference frame of visual/force/inertial sensors

    Robot's reference frame:
        X:back(-)/front(+)      Y:right(-)/left(+)      Z:down(-)/up(+)

    Sensors that use the Robot's reference frame:
        - Foot/Toe contact position/force 
            - the force vector is applied to the foot, so it usually points up
            - the contact point is given in cartesian coordinates, relative to the foot's center
        - Accelerometer (torso)
            - The accelerometer measures the acceleration relative to freefall 
            - It will read zero during any type of free fall
            - When at rest relative to the ground, it indicates an upwards acceleration of 9.81m/s^2 (in SimSpark)
        - Gyroscope (torso)
            - The gyroscope measures the robot's torso angular velocity (rotation rate vector)
            - The angular velocity's orientation is given by the right-hand rule
            - Examples: 
                - if the robot leans to the right: it'll have a positive angular speed in x
                - if the robot leans back:         it'll have a negative angular speed in y
                - if the robot rotates clockwise:  it'll have a negative angular speed in z
    
    Relative measurements that use the Robot's reference frame:
        - Cartesian coordinates
        - Spherical coordinates (radial distance, azimuthal angle, polar angle)
            - in other words (distance, horizontal angle, vertical angle)
            - Examples:
                - (1,10,0) Object is at 1 meter, slightly to the left (10 deg), at eye level (0 deg)
                - (2,0,90) Object is at 2 meters, right above the robot's head, in cartesian coordinates: (0,0,2)

    Orientation 2D (Yaw)
        - The Yaw angle is used for: localizer orientation, cheat orientation, beam, unofficial beam
        - The rotation is measured around the field's z-axis, following the right-hand rule
        - The rotation is zero if the robot is aligned with the field's x-axis

---

Behaviors

    Behavior is a low-level class that controls physical movements to achieve a certain goal.

    Notes: 
        - behaviors commit joint targets without actually advancing a simulation step
            (commit = changing values in the array player.world.robot.joints_target_speed )
        - multiple behaviors can be called before advancing a simulation step
            (useful if each behavior controls different parts of the robot, otherwise, the overlaped targets
            will be overwritten by the last behavior in player.world.robot.joints_target_speed)

    There are 4 types of behaviors:
        poses    - behaviors made of a single pose
        slot     - behaviors composed of multiple poses (one per time slot)
        custom   - custom behaviors implemented as a python script
        head     - behavior that controls the head orientation

        Each behavior is characterized by the following dictionary entry:
            key: ( description, auto_head, lambda reset[,a,b,c,..]: self.execute(...), lambda: self.is_ready(...) )
            key - unique string ID
            description - describe what behavior does (this is useful when listing available behaviors)
            auto_head - boolean indicating whether the automatic head orientation can be enabled during this behavior
            lambda reset[,a,b,c,..]: self.execute(...) - lambda function that executes one step of the behavior
                fixed arguments:
                    - reset: boolean indicating whether the behavior should be reset in the current time step
                the user may provide additional arguments
            lambda: self.is_ready(...) - lambda function indicating if the behavior is ready to be executed


    Poses

        A list of poses is available at: /behaviors/Poses.py
        To add a new pose just add a new entry to the dictionary in that file, using a unique behavior name

    Slot

        Slot behaviors are defines as XML files in /behaviors/slot/[common OR rX]
            "common" contains slot behaviors that are common to all robot types
            rX (r0,r1,r2,r3,r4) contains behaviors for specific robot types

    Head

        The behavior that controls the automatic head orientation if defined at behaviors/Head.py

    Custom Behaviors
    
        Each folder inside /behaviors/custom is a behavior
        If a folder is called X, it must contain a file named X.py, which implements class X with the following methods/variables:
            - __init__(world)
            - execute(reset,(...)) -> bool  # reset flag is set to true to intialize/reset behavior, returns True if finished/aborted
            - is_ready() -> bool      # returns True if behavior is ready to start under current game/robot conditions
            - self.description        # class variable - one-liner description of behavior
            - self.auto_head          # class variable - set to true to enable automatic head movements
        Usage:
            player.world.robot.behavior.execute("My_Behavior", [arg1, ...])       # internally calls My_Behavior.execute(True)
            # one step later...
            player.world.robot.behavior.execute("My_Behavior", [arg1, ...])       # internally calls My_Behavior.execute(False)
            # one step later...
            player.world.robot.behavior.execute("Another_Behavior", [arg1, ...])  # internally calls Another_Behavior.execute(True)

---

Localization

    The localization method computes a 6D pose every VISION cycle.

    Generated data (accessed through Robot Class):

        # Localization variables relative to head
        cheat_abs_pos                # Absolute head position provided by the server as cheat
        cheat_ori                    # Head orientation in degrees provided by the server as cheat
        loc_head_to_field_transform  # Transformation matrix from head to field
        loc_field_to_head_transform  # Transformation matrix from field to head
        loc_rotation_head_to_field   # Rotation matrix from head to field
        loc_rotation_field_to_head   # Rotation matrix from field to head
        loc_head_position            # Absolute head position (m)
        loc_head_velocity            # Absolute head velocity (m/s)
        loc_head_orientation         # Head orientation in degrees
        loc_is_up_to_date            # False if this is not a visual step, or not enough elements are visible
        loc_last_computed            # World.time_local_ms when the localization was last computed

        # Localization variables relative to torso
        loc_torso_orientation   # Torso orientation in degrees
        loc_torso_pitch         # Torso pitch in degrees (inclination of x-axis in relation to field xy plane)
        loc_torso_roll          # Torso roll  in degrees (inclination of y-axis in relation to field xy plane)
        loc_torso_inclination   # Torso inclination in degrees (inclination of z-axis in relation to field z-axis)
        loc_torso_position      # Absolute torso position (m)
        loc_torso_velocity      # Absolute torso velocity (m/s)
        loc_torso_acceleration  # Absolute Coordinate acceleration (m/s2)

    Generated data (accessed through World Class):

        ball_abs_pos  # Ball absolute position (up to date if ball is visible and localization is up to date)
        teammates     # Contains information about teammates, based on visual data, refer to Other_Robot Class
        opponents     # Contains information about opponents, based on visual data, refer to Other_Robot Class

        Other_Robot Class useful data:
            unum        # convenient variable to indicate uniform number (same as other robot's index + 1)
            is_selfe    # convenient flag to indicate if this robot is self
            is_visible  # True if this robot was seen in the last message from the server

                Note: Even if the robot is visible, we only know its absolute localition if we can self-locate

            body_parts_cart_rel_pos  # cartesian relative position of the robot's visible body parts
            body_parts_sph_rel_pos   # spherical relative position of the robot's visible body parts

            State variables (below): are computed when (is_visible) and the original robot is able to self-locate

            state                # STATE_NORMAL, STATE_FALLEN, STATE_UNKNOWN (when insufficient visible body parts)
            state_last_update    # World.time_local_ms when the state was last updated
            state_distance       # head distance if head is visible, otherwise, avg. distance of visible body parts
            state_abs_pos        # 3D head pos. if head is visible, otherwise, 2D avg. pos. of visible body parts
            state_orientation    # orientation based on pair of lower arms or feet, or average of both
            state_velocity       # 3D filtered velocity (m/s) is updated if head is visible now and in last update
            state_ground_area    # projection of player area on ground (circle)

                Note: state_ground_area is useful for obstacle avoidance when the visible robot falls,
                      state_ground_area is not precise if farther than 3m (for performance), 
                      
            state_body_parts_abs_pos   # absolute position of each body part
            

    There are situations in which the rotation and translation cannot be computed, but the z-coordinate can still
    be found through vision. If this is the case, the variables above will not be updated. But the new z-coordinate
    can be accessed through "loc_head_z", and "loc_head_z_is_up_to_date" will be "True". It should be used in 
    applications which rely on z as an independent coordinate, such as detecting if the robot has fallen, or as an
    observation for machine learning. It should NEVER be used for 3D transformations.

    Get linear transformations or subsets (e.g. translation) (accessed through Robot Class). These methods combine
    localization data with forward kinematics, so, use only when self.loc_is_up_to_date is True. Otherwise, the 
    forward kinematics will not be synced with the localization data and strange results may occur.

        get_body_part_to_field_transform(body_part_name)   # transformation matrix from body part to field
        get_body_part_abs_position(body_part_name)         # absolute position of body part
        get_joint_to_field_transform(joint_index)          # transformation matrix from joint to field
        get_joint_abs_position(joint_index)                # absolute position of joint

    Convert relative <-> absolute coordinates or other types of conversion using transformation matrices:

        Matrix_3x3 or Matrix_4x4 objects can be called to apply the multiplication operation.
        Arguments: (matrix or vector to be multiplied, true if vector uses spherical coordinates)
        Examples:
            ball_absolute_position = loc_head_to_field_transform( ball_rel_head_sph_pos, True )
            ball_rel_head_cart_pos = loc_field_to_head_transform( ball_absolute_position, False )


---

Monitoring

    print(world) -> monitor world status


---

Drawings

    Several shapes can be drawn in RoboViz natively. 
    In this section, there are 2 main concepts: shape (e.g. circle) and drawing, which is composed of >=1 shapes
    Each drawing has a specific string ID. Drawings can be created, replaced, erased, or hidden.

    Note: the drawing methods are independent for each agent
    Note: do not use IDs that are composed of existing IDs, such as "shape" and "shape1" (to avoid a RoboViz bug)

Shapes:

    circle(     pos2d,   radius,    thickness, color:bytes,              id:str, flush=True)
    line(       p1,p2,              thickness, color:bytes,              id:str, flush=True)
    point(      pos3d,   size,                 color:bytes,              id:str, flush=True)
    sphere(     pos3d,   radius,               color:bytes,              id:str, flush=True)
    polygon(    vertices,                      color:bytes, alpha:bytes, id:str, flush=True)
    annotation( pos3d,   text,                 color:bytes,              id:str, flush=True)
    arrow(      p1,p2,   arrowhead, thickness, color:bytes,              id:str, flush=True)

Examples:

    Note: for these examples we assume that 'world' is a variable in the class where we're creating the drawing

    Enable/disable drawings:

    #this is not recommeded, as the starting script will define if the drawing are enabled for a given agent
    self.world.draw.enable = True/False  

    Create a drawing made of one shape:

    #Every time this instruction is called, it will replace the drawing called "my_drawing"
    self.world.draw.circle( (0,0),2,2,Draw.Color.green_light,"my_drawing")   # center=(0,0), radius=thickness=2

    #If we draw another shape with the same ID, it will also replace the previous shape
    self.world.draw.sphere( (0,0,0.2),0.2, Draw.Color.red, "my_drawing" )  # center=(0,0,0.2), radius=0.2

    Create a complex drawing:

    #We can draw a pyramid by creating a polygon base and some lines and flush only the last shape
    self.world.draw.polygon( ((0,0,0),(1,0,0),(1,1,0),(0,1,0)), Draw.Color.blue, b'\xFF', "solid", False)
    self.world.draw.line( (0,0,0), (0.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
    self.world.draw.line( (1,0,0), (0.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
    self.world.draw.line( (0,1,0), (0.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
    self.world.draw.line( (1,1,0), (0.5,0.5,1), 2, Draw.Color.cyan, "solid", True)

    #A complex drawing can also be flushed manually
    for pts in random_pairs_of_points:
        self.world.draw.line( pts[0], pts[1], 2, Draw.Color.yellow, "complex", False)
    self.world.draw.flush("complex")
    #Calling flush again would clear the drawing, since no shapes have been committed in the meantime

    Erase a specific drawing:
    self.world.draw.clear("my_drawing")

    Erase all drawings created by a specific agent:
    self.world.draw.clear_player()

    Erase all drawings:
    Draw.clear_all()


---

C++ Modules

    C++ modules can be integrated in python using Pybind11.
    The source code is compiled automatically when one of the initial python scripts is executed.
    New modules must be placed in a folder named MODULENAME inside cpp/ with a makefile that creates MODULENAME.so

    As an example, please refer to the localization module.
    
---

Bundle App

    The script /bundle/bundle.sh will bundle the app, dependencies and data files into a single executable/folder.

    Bundle options:
    - One file:
        - Everything goes into a single executable
        - Compressed and easier to share
        - A tiny bit harder to access data files by other people
        - Loading the app takes a long time to extract data* to a tmp folder
    - One folder:
        - Much faster bundling and loading app
        - Easier to access data files
        - Large folder size
        
    *Non-Python support files and data files in one-file mode:
    - These files are compressed and distributed inside the executable but they are extracted to a temporary folder 
      /tmp/_MEI*** during execution. So, they are easily accessible by unauthorised parties. Do not store sensitive
      information in data files. Current policy: if needed, encrypt file.

---

VS Code Tips

Hide certain files or folders

    In Settings/Text Editor/Files/Exclude add pattern:
    **/__pycache__

launch.json configuration examples

    //Example: launch current file with player number 2, robot type 3
    //for more arguments use the help argument with any start script
    //e.g. "python Script_One_Player.py --help"
    //note: Default arguments are defined in "config.json"
    {
        "name": "Python: Current File",
        "type": "python",
        "request": "launch",
        "program": "${file}",
        "args": ["-u","2","-r","3"],
        "console": "integratedTerminal"
    },

    //easily launch a specific script from any location
    {
        "name": "Python: 1Player",
        "type": "python",
        "request": "launch",
        "program": "Script_One_Player.py",
        "console": "integratedTerminal"
    },

    {
        "name": "Python: Team",
        "type": "python",
        "request": "launch",
        "program": "Script_Full_Team.py",
        "console": "integratedTerminal"
    },
    {
        "name": "Python: Tests",
        "type": "python",
        "request": "launch",
        "program": "Script_Tests.py",
        "console": "integratedTerminal"
    },

    //profile code (to enable this, add the print_profile task to tasks.json, see below)
    {
        "name": "Python: Profile Current File",
        "type": "python",
        "request": "launch",
        "module": "cProfile",
        "args": ["-o", "profile", "${file}",],
        "console": "integratedTerminal",
        "postDebugTask":"print_profile"
    }

tasks.json task examples

    //task to print profile data from "profile" file, created by the configuration "Python: Profile Current File"
    {
        "label": "print_profile",
        "type": "shell",
        "command": "python -c \"import pstats; p = pstats.Stats('profile'); p.sort_stats('cumulative').print_stats(30)\""
    }

--- 

Server configuration

Sync Mode is recommended, unless it is an official match

    ~/.simspark/spark.rb
        $agentSyncMode = true

Cheat data is useful, just remember not to use it for official routines

    /usr/local/share/rcssserver3d/rsg/agent/nao/naoneckhead.rsg
        (setSenseMyPos true)
        (setSenseMyOrien true) 
        (setSenseBallPos true)

For machine learning, these settings are safe

    /usr/local/share/rcssserver3d/rcssserver3d.rb
        $enableRealTimeMode = false

    /usr/local/share/rcssserver3d/naosoccersim.rb
        addSoccerVar('BeamNoiseXY',0.0)
        addSoccerVar('BeamNoiseAngle',0.0)
        #gameControlServer.initControlAspect('SoccerRuleAspect')

These settings are NOT recommended (for the sake of generalization)

    /usr/local/share/rcssserver3d/rsg/agent/nao/naoneckhead.rsg
        (setViewCones 360 360)
        (addNoise false)

    

