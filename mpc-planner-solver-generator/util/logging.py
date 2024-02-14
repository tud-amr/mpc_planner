import sys, os
import numpy as np

class bcolors:
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    HEADER = BOLD


def print_value(name, value, tab=False, **kwargs):

    if tab:
        string = " "
    else:
        string = ""
    
    print(string + bcolors.BOLD + bcolors.UNDERLINE + f"{name}" + bcolors.ENDC + f": {value}", **kwargs)

def print_path(name, value, tab=False, **kwargs):
    print_value(name, os.path.abspath(value), tab, **kwargs)

def print_success(msg):
    print(bcolors.BOLD + bcolors.OKGREEN + f"{msg}" + bcolors.ENDC)

def print_warning(msg):
    print("\t" + bcolors.BOLD + bcolors.WARNING + f"{msg}" + bcolors.ENDC)

def print_header(msg):
    print("==============================================")
    print("\t" + bcolors.HEADER + f"{msg}" + bcolors.ENDC)
    print("==============================================")

class TimeTracker:

    def __init__(self, name):
        self._name = name
        self._times = []

    def add(self, timing):
        self._times.append(timing)

    def get_stats(self):
        return np.mean(self._times), np.max(self._times), len(self._times)

    def print_stats(self):
        print(f"--- Computation Times {self._name} ---")
        print_value("Mean", f"{np.mean(self._times):.1f} ms", tab=True)
        print_value("Max", f"{np.max(self._times):.1f} ms", tab=True)
        print_value("Number of calls", len(self._times), tab=True)
