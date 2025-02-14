import time, os, subprocess, sys

# Constants

UPDATE_FREQUENCY = 1 # second

# ---

if len(sys.argv) < 2:
    print(f"Invalid usage of {sys.argv[0]}, a single argument denoting the script's path is needed.")
    exit()

script = sys.argv[1]
watched_files = sys.argv[1:]

def run():
    global instance
    if instance:
        instance.kill()

    print(f"Running '{script}'")
    instance = subprocess.Popen(["python", script])

# ---

instance: subprocess.Popen = None

run()
print(f"Watching {watched_files} for changes. Press Ctrl+C to stop.")

# ---

# Continuously check for modifications
modification_times = {file: os.path.getmtime(file) for file in watched_files}
try:
    while True:
        new_modification_times = {file: os.path.getmtime(file) for file in watched_files}
        
        if new_modification_times != modification_times:
            print("Changed detected, reloading...")
            modification_times = new_modification_times
            run()
        
        time.sleep(UPDATE_FREQUENCY)
except KeyboardInterrupt:
    print("Stopped watching.")