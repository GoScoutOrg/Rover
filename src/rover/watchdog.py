import threading, _thread, time, signal

class Watchdog:
    '''
    Watchdog class monitors the status of target program using a boolean value.

    Target program includes this class and updates it's status by calling the
    provided "setflag()" function.

    In turn, a separate thread is created specifically for the watchdog that 
    checks the status of the flag every minute. The watchdog then shuts down
    the target program if the flag has not been set within the last minute.
    '''
    
    def __init__(self):
        self.flag = True

    def set_flag(self):
        # Update status by marking flag
        self.flag = True

    def check_flag(self):
        # Run in separate thread. Every minute, checks status of flag
        while True:
            # If flag set, unset the flag
            if self.flag:
                self.flag = False

            # If flag is not set, kill the target program
            else:
                print('Watchdog killing program')
                signal.raise_signal(signal.SIGKILL)

            # Wait 60 seconds before checking again
            time.sleep(5)

    def start(self):
        # Make a new thread that only runs "check_flag()"
        self.thread = threading.Thread(target=self.check_flag, daemon=True)

        # Start the thread
        self.thread.start()
