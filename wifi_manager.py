import subprocess
import re
import os
import time
from colour_printing import print_coloured, bcolors

def get_windows_ssid():
    try:
        # Run command and parse for SSID
        results = subprocess.check_output(["netsh", "wlan", "show", "interface"], text=True)
        ssid = re.search(r"SSID\s*:\s*(.*)\n", results)
        return ssid.group(1).strip() if ssid else "Not Connected"
    except Exception:
        return "Not Connected"
    
def connect_to_wifi(ssid_name):
    # Command to connect to a known network profile by name
    print_coloured(f"You are not connected to the {ssid_name} network. Trying to connect automatically...", bcolors.OKCYAN)
    command = f'cmd /c "netsh wlan connect name={ssid_name}"'
    os.system(command)
    
    start = time.time()
    timeout = 10 # Seconds
    while(time.time()-start < timeout):
        if(get_windows_ssid() == ssid_name):
                print_coloured(f"Successfully connected to {ssid_name} network.", bcolors.OKBLUE)
                return
    
    print_coloured(f"Failed to connect to the {ssid_name} network after {timeout} seconds.", bcolors.FAIL)
    return

def assert_connection_to_network(ssid):
    if(get_windows_ssid() != ssid):
        
        connect_to_wifi(ssid)
    


