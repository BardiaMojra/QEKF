"""
@author: Kory Herber  Uploaded: 12/31/19

This script extracts the start/stop time from the trial xcp file and displays them in the Nexus log output

Modified version of JGay's script to extract time stamp info from xcp file and print to Nexus log.  This version should run without the need
to install any addtional modules.
"""

import ViconNexus
import re

log_separator = "=============================================================================================================================="
vicon = ViconNexus.ViconNexus()

def StartStopStamps():
    SessionLoc = vicon.GetTrialName()[0]
    XCPTrialName = SessionLoc + vicon.GetTrialName()[1] + ".xcp"
    startTime = r"(?<=Capture).*?(?<=START_TIME=\")(.*?)(?=\")"
    endTime = r"(?<=Capture).*?(?<=END_TIME=\")(.*?)(?=\")"
    xcpfile = open(XCPTrialName)
    xcpfilestring = xcpfile.read()
    startTimeValue = re.findall(startTime, xcpfilestring, re.MULTILINE)[0]
    endTimeValue = re.findall(endTime, xcpfilestring, re.MULTILINE)[0]

    print(log_separator)
    print('Start Time: ' + startTimeValue)
    print('End Time: ' + endTimeValue)
    print(log_separator)
            

if __name__ == "__main__":
	StartStopStamps()