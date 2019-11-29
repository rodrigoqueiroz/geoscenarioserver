__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

class Report(object):

    def __init__(self):
        self.errors = []
        self.warnings = ""
        self.tips = ""
        self.autoprint = False
        self.file = ""

    def log_error(self,msg):
        msg = str("Error: "+msg)
        if (self.autoprint): 
            print (bcolors.FAIL + msg + bcolors.ENDC)
        self.errors.append(msg)

    def log_warning(self,msg):
        msg = str("Warning: "+msg+'\n')
        self.warnings += msg
        if (self.autoprint): 
            print (bcolors.WARNING + msg + bcolors.ENDC)

    def log_tip(self,msg):
        msg = str("Warning: "+msg+'\n')
        self.tips += msg
        if (self.autoprint): 
            print (bcolors.OKBLUE + msg + bcolors.ENDC)

    def print(self):
        print(bcolors.HEADER+"#### GSChecker Report #### "+bcolors.ENDC)
        print("GeoScenario: " + str(self.file))
        if len(self.errors) > 0: 
            print ("Contains the following errors:")
            for msg in self.errors:
                print (bcolors.FAIL + msg + bcolors.ENDC)
        else:
           print (bcolors.OKGREEN + "No errors found :)" + bcolors.ENDC)
        if len(self.warnings) > 0: 
            print ("Warnings:")
            print (self.warnings)
    
    

        

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'