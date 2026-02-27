import subprocess
import re
import os
import time

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
    command = f'cmd /c "netsh wlan connect name={ssid_name}"'
    os.system(command)
    
    start = time.time()
    timeout = 10 # Seconds
    while(time.time()-start < timeout):
        if(get_windows_ssid() == ssid_name):
                return f"Successfully connected to {ssid_name} network."
    
    return f"Failed to connect to the {ssid_name} network after {timeout} seconds."

def assert_connection_to_network(ssid):
    if(get_windows_ssid() != ssid):
        print(f"You are not connected to the {ssid} network. Trying to connect automatically...")
        print(connect_to_wifi(ssid))
    


