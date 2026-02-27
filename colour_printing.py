class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_coloured(text:str,colour_code:str,*args,**kwargs):
    try:
        print(colour_code + text + bcolors.ENDC,*args,**kwargs)
    except KeyboardInterrupt as e:
        raise KeyboardInterrupt
    except Exception as e:
        # For OS's that don't support the control characters.
        print(text,*args,**kwargs)