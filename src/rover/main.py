from multiprocessing.dummy import Process

from Communications.communications.parent import parent_proc

function_set = {
        "GPS": lambda args : parse_location({int(args[0])}),
       # "": lambda args : 
    }
communications = Process(target=parent_proc, args=("192.168.4.10",7676, "192.168.4.3", 7777, function_set))

def parse_location(gps_location):
    target_lat = gps_location[0:8]
    target_long = gps_location[8:]
    #call brets calc functions
    
    

def main():
    pass


if __name__ == "__main__":
    main()
