import threading
import rover.commands as commands

MAX_TURN = 36  # max L / R corner wheel rotation
DESCRIPTIONS = []
with open('DESCRIPTIONS.txt', 'r') as f:
    for line in f:
        DESCRIPTIONS.append(line)

follow_thread: threading.Thread

""" 'manual' for avail commands / command cookbook """
""" 
to add new func / command: 
    - map its verbage to its corresponding function in commands.py in parseCommand
"""

# dumps this info when user enters help cuz I'm dumb and can't remember every command and its field
def print_message():
    with open("DESCRIPTIONS.txt", "r") as f:
        for line in f:
            print(line, end="")


def command_help(tokn: str):
    out_str = ""
    found = False
    for line in DESCRIPTIONS:
        line_lst = line.split()
        if len(line_lst) > 0 and line_lst[0] == tokn:
            found = True
        if found:
            if line == '\n':
                break
            out_str += line
    out_str = out_str.strip()
    if len(out_str) == 0:
        print('command not found: %s' % tokn, end="")
    else:
        print(out_str.strip(), end="")


def command_calibrate(tokn: list):
    """Handles all versions of a calibrate command

    Returns
    ----
    -1
        when input command does not conform to any template
    0
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 0:  # assume a calibrate all
        return commands.calibrate_all()
    for m in tokn:
        commands.calibrate_one(m)
    return 0




def command_backward(tokn: list):
    """Handles all versions of backward commands
    Types: {'backward speed', 'backward speed distance'}

    Returns
    ----
    -1
        when input command does not conform to any template
    0
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 1:
        # backward speed
        return commands.backward_with_stop(tokn[0])
    elif len(tokn) == 2:
        # backward speed distance
        return commands.move_distance_meters(-1 * tokn[0], tokn[1])
    else:
        return -1


def command_forward(tokn: list):
    """Handles all versions of forward commands.
    Types: {'forward speed', DEPRECATED: 'forward speed distance'}

    Returns
    ----
    -1
        when input command does not conform to any template
    0
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 1:
        # forward speed
        return commands.forward_with_stop(tokn[0])
    elif len(tokn) == 2:
        # forward speed distance
        # DEPRECATED! Uses time based command!
        return commands.move_distance_meters(tokn[0], tokn[1])
    else:
        return -1


def parseCommand(command):  # try-except commented out for debugging purposes
    global follow_thread
    try:
        if command[0] == "help" and len(command) > 1:
            return command_help(command[1])

        elif command[0] == 'help' and len(command) == 1:
            return print_message()

        elif command[0] == "calibrate":
            return command_calibrate(command[1:])

        elif command[0] == "arc" and len(command) == 5:
            return commands.turn(command[1], command[2], command[3], command[4])

        elif command[0] == "recenter":
            return commands.recenter()

        elif command[0] == "forward":
            return command_forward(command[1:])

        elif command[0] == "backward":
            return command_backward(command[1:])

        elif command[0] == "tankturn":
            if len(command) != 3:
                return -1
            return commands.tank_with_turn(command[1], command[2])

        elif command[0] == "stop":
            return commands.kill_all()

        elif command[0] == "follow_me":
            follow_thread = threading.Thread(target=commands.follow_me)
            follow_thread.start()

            input("Enter to cancel... ")
            commands.follow_stop()
            follow_thread.join()
            return 0

        elif command[0] == "follow_me_stop":
            commands.follow_stop()
            follow_thread.join()
            return 0

        elif command[0] == "mtp" or command[0] == "move_to_point":
            if len(command) == 2:
                return commands.move_to_point(command[1])
            elif len(command) == 3:
                return commands.move_to_point(command[1], command[2])
            else:
                command_help(command[0])
                return -1
        else:
            print("Unrecognized Command")
            return -1
    except ValueError:
        return -1