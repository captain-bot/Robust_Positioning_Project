import os
import shutil
import platform
import site

def CopyToPackage(filename, srcPath, destPath):
    if (not os.path.exists(srcPath + filename)):
        print(srcPath + filename, "is missing.")
        raise Exception("Missing File")
    if (os.path.exists(destPath + filename)):
        os.remove(destPath + filename)
    shutil.copy(srcPath + filename, destPath)
    print("\tFile installed:", filename)

if platform.system() == "Linux":
    HOME = os.path.expanduser("~")
    IKFAST_PY_PATH = HOME + "/dev_research/Baxter_IKFast_Python/"
    IKFAST_BINARY_FILE = "ikModule.so"
    IKFAST_INSTALL_PATH = site.getsitepackages()[0]
    CopyToPackage(IKFAST_BINARY_FILE, IKFAST_PY_PATH, IKFAST_INSTALL_PATH)    
else:
    print("Not supported for currently used OS\n")

