## Music and lyrics by John Porter and Nev Pires
##
## This script will calculate teh position and orientation of instrumented treadmill force plates at any given incline.
## The position and orientation of the plates in the level configuration are required to be entered into the Nexus system
## configuration ahead of time. It is preferred that only the treadmill's instrumented plates are defined within the system
## configuration. It is required that all devices be given unique names.
##
## The script also requires at least three non-collinear reference
## markers to be placed on the treadmill frame and a VSK created for them.
## There are no other restrictions as to the number of markers (greater than 3), their names or
## arrangement as long as they are not collinear. It is recommended to distribute the markers around
## the perimeter of the treadmill frame.
##
## A trial must be captured in the level position along with any number of
## trials in the desired inclined positions. All trials must be
## reconstructed, labeled and saved prior to running this script. There can
## be gaps in the marker data (don't fill the gaps) but all markers in the
## VSK must have some associated data.
##
## Trial names cannot contain spaces and must begin with a letter.
##
## The script file should be copied to the session folder and run from there.
## Run it through Matlab, not from the Nexus Matlab tab or pipeline
## operation. The script will prompt you to select the level trial and then
## to multi-select all incline trials for processing. The script expects at
## least two incline trials.
##
## The script computes the optimal translation and rotation between the level
## marker configuration and the inclined marker configuration. Then it
## applies the same transformation to the force plate position and
## orientation values.
##
## The script outputs a spreadsheet file to the session folder with a tab for each trial. Each tab
## presents the position and orientation values for each force plate. These
## values are to be entered into a new Nexus system configuration corresponding
## to an inclined position.
##
## This script will require you to have the following python modules installed:
##
## os
## math
## numpy
## sys
## Tkinter
## tkMessageBox
## pandas
##
## This script was written in Python 2.7.14

########################################################################################################################
########################################################################################################################
###################################                                                   ##################################
###################################     Import the relevant modules needed            ##################################
###################################                                                   ##################################
########################################################################################################################
########################################################################################################################

import os
import math
import numpy as np
import sys
from Tkinter import *
import tkMessageBox, Tkconstants, tkFileDialog
import numpy.matlib
import xlsxwriter
import pandas as pd

sys.path.append('C:/Program Files (x86)/Vicon/Nexus2.7/SDK/Win32')
sys.path.append('C:/Program Files (x86)/Vicon/Nexus2.7/SDK/Python')
import ViconNexus

## Connect to Nexus

vicon = ViconNexus.ViconNexus()

########################################################################################################################
########################################################################################################################
###################################                                                   ##################################
###################################                  Function Section                 ##################################
###################################                                                   ##################################
########################################################################################################################
###################################                                                   ##################################
########################################################################################################################

###################################       Angle Axis from matrix function             ##################################

##
## This function requires a 3x3 matrix as an input
##

def AngleAxisFromMatrix(A):
    angle = np.degrees(np.arccos(( A[0,0] + A[1,1] + A[2,2] - 1)/2))
    r = np.sqrt((A[2,1] - A[1,2])**2+(A[0,2] - A[2,0])**2+(A[1,0] - A[0,1])**2)
    if r!= 0:
        x = (A[2, 1] - A[1, 2]) / r
        y = (A[0, 2] - A[2, 0]) / r
        z = (A[1, 0] - A[0, 1]) / r
        orientation = np.matrix([angle * x, angle * y, angle * z]).T
    else:
        orientation = np.matrix([0,0,0]).T

    return orientation

###################################           Trajectory Check function               ##################################
##
## This function will check to see whether the markers exist in the trial.
## This function requires the subject name and the marker list to be entered in as a string
## It will return an error message if a marker is missing

def TrajectoryCheck(vicon, subject, markerList):

    dataExists = [vicon.HasTrajectory(subject, markerList[markers]) for markers in range(len(markerList))]

    return(dataExists)

###################################           Get Marker Data function                ##################################
##
## This function will import the marker data
## This function requires the subject name and the marker list to be entered as a string
##

def GetMarkerData(vicon, subject, markerList):

    [selectedStart, selectedEnd] = vicon.GetTrialRegionOfInterest()
    markerData = {markers: np.transpose(vicon.GetTrajectory(subject, markers)) for markers in markerList}
    markerData = {markers: markerData[markers][selectedStart-1:selectedEnd,:] for markers in markerData.keys()}

    return(markerData)

###################################              Set NaN's function                   ##################################
##
## This function will look through each marker to see whether it exists or not. If it does not exist, it will set
## the value to nan.
## This function requires the marker array to be entered

def SetNaN(markerArray):

    for markers in markerArray.keys():
        markerArray[markers][markerArray[markers][:, -1] == 0, 0:3] = np.nan

    return(markerArray)

###################################        Create Average Matrix function             ##################################
##
## This funtion will calculate the average for the x,y,z values for each marker and will write that average to a 3 x n
## matrix where n is the number of markers in the trial.
## This function requires the marker array to be entered as an input


def CreateAverageMatrix(markerArray):

    markerArrayAve = {markers: np.nanmean(markerArray[markers][:,0:3], axis = 0) for markers in markerArray.keys()}
    markerMatrix = np.array([items for _, items in sorted(markerArrayAve.items())]).T

    return(markerMatrix)

###################################      Change Marker Clound Pose function           ##################################

def ChangeMarkerCloudPose(initialMatrix, finalMatrix):
    initialOrigin = np.zeros((3,1))
    finalOrigin = np.zeros((3,1))
    centeredInitialMatrix = np.zeros(np.shape(initialMatrix))
    centeredFinalMatrix = np.zeros(np.shape(finalMatrix))

    finalInitialMatrix = {}
    for planes in range(len(initialMatrix)):
        initialOrigin[planes] = np.mean(initialMatrix[planes])
        finalOrigin[planes] = np.mean(finalMatrix[planes])
        for markers in range(len(initialMatrix[planes])):
            centeredInitialMatrix[planes, markers] = initialMatrix[planes, markers] - initialOrigin[planes]
            centeredFinalMatrix[planes, markers] = finalMatrix[planes, markers] - finalOrigin[planes]
    C = np.matmul(centeredFinalMatrix, np.transpose(centeredInitialMatrix))
    [U, _, V] = np.linalg.svd(C, full_matrices=False)

    detUVT = np.linalg.det(np.matmul(U, V))
    L = np.identity(3)

    if detUVT < 0:
        L[2,2] = -1

    R = reduce(np.dot, [U,L, V])
    T = finalOrigin - reduce(np.dot, [R, initialOrigin])

    return (R, T)

########################################################################################################################
########################################################################################################################
###################################                                                   ##################################
###################################               Level Trial Section                 ##################################
###################################                                                   ##################################
########################################################################################################################
########################################################################################################################

## Load the level trial

tkMessageBox.showinfo("Load Level Trial", "Load Level Trial") ## Message dialog box

fullLevelTrial = Tk()

fullLevelTrial.filename = tkFileDialog.askopenfilename(title = "Select level trial",filetypes = (("c3d Trials","*.c3d"),("all files","*.*")))
fullLevelTrial = str(fullLevelTrial.filename[:-4])
fullLevelTrial = fullLevelTrial.replace("/", "\\")

levelTrialName = os.path.basename(fullLevelTrial)

vicon.OpenTrial(fullLevelTrial, 60)

tkMessageBox.showinfo("Level Trial has been loaded",
                      "Level Trial has been loaded")

## Read the level treadmill force plate positions etc.

deviceIDs = np.array(vicon.GetDeviceIDs())
deviceNames = vicon.GetDeviceNames()

if not deviceNames:
    tkMessageBox.showinfo("WARNING!!!",
                          "Devices not found, or they are unnamed")

deviceDetails = {devices: vicon.GetDeviceDetails(devices) for devices in deviceIDs} # Gets all the device details for all the devices in the system file
plateNames = tuple([deviceDetails[devices][0] for devices in deviceIDs if deviceDetails[devices][1] == 'ForcePlate']) # extracts the force plate names by looping through the device details and checking to see if a device type = 'Force Plate'
plateIDs = {names: vicon.GetDeviceIDFromName(names) for names in plateNames}
levelR = {names: np.reshape(vicon.GetDeviceDetails(values)[4].WorldR, (3,3)) for names, values in plateIDs.items()}
levelOrient = {names: AngleAxisFromMatrix(values) for names, values in levelR.items()}
worldT = {names: vicon.GetDeviceDetails(values)[4].WorldT for names, values in plateIDs.items()}

if not plateNames:
    tkMessageBox.showinfo("WARNING!!!", "No force plates found")

subjects = vicon.GetSubjectNames()
if not subjects:
        tkMessageBox.showinfo("WARNING!!!", "No subject found")

## Get Subject Info

subject = subjects[0]
segment = vicon.GetRootSegment(subject)
[_, _, markerList] = vicon.GetSegmentDetails(subject, segment)

if not markerList:
    tkMessageBox.showinfo("WARNING!!!",
                              "No marker set associated with the subject")
elif len(markerList) < 3:
    tkMessageBox.showinfo("WARNING!!!",
                            "The marker set requires at least three markers")

## Read the level marker data and set up the marker matrix

hasData = TrajectoryCheck(vicon, subject, markerList)

if sum(hasData) < len(markerList):
    tkMessageBox.showinfo("WARNING!!!",
                          "The marker set requires at least three markers")

levelMarkers = GetMarkerData(vicon, subject, markerList)
levelMarkers = SetNaN(levelMarkers)
avgLevelMatrix = CreateAverageMatrix(levelMarkers)

########################################################################################################################
########################################################################################################################
###################################                                                   ##################################
###################################            Incline Trials Section                 ##################################
###################################                                                   ##################################
########################################################################################################################
########################################################################################################################

## Load and process each of the incline trials

tkMessageBox.showinfo("Load Incline Treadmill Trials", "Load Incline Trials") ## Message dialog box

fullInclineTrials = Tk()

fullInclineTrials.filename = tkFileDialog.askopenfilenames(title = "Select incline trials",filetypes = (("c3d Trials","*.c3d"),("all files","*.*")))

trialNames = [os.path.basename(os.path.normpath(fullInclineTrials.filename[names])[:-4]) for names in range(len(fullInclineTrials.filename))]
fullInclineFileNames = {name: fullInclineTrials.filename[index][:-4] for index, name in enumerate(trialNames)}

inclineMarkers = {}
avgInclineMatrix = {}
translate = {}
rotate = {}

fullInclineFileNames = {name: fullInclineTrials.filename[index][:-4].replace("/","\\") for index, name in enumerate(trialNames)}

for names in fullInclineFileNames.keys():
    vicon.OpenTrial(fullInclineFileNames[names], 600)
    hasData = TrajectoryCheck(vicon, subject, markerList)

    if sum(hasData) < len(markerList):
        tkMessageBox.showinfo("WARNING!!!",
                              "The marker set requires at least three markers")

    inclineMarkers[names] = GetMarkerData(vicon, subject, markerList)
    inclineMarkers[names] = SetNaN(inclineMarkers[names])
    avgInclineMatrix[names] = CreateAverageMatrix(inclineMarkers[names])

    [rotate[names], translate[names]] = ChangeMarkerCloudPose(avgLevelMatrix, avgInclineMatrix[names])

inclinePos = {names: {plates: translate[names] + reduce(np.dot, [rotate[names], np.matrix(worldT[plates]).T]) for plates in plateNames} for names in trialNames}
inclineR = {names: {plates: reduce(np.dot, [rotate[names], levelR[plates]]) for plates in plateNames} for names in trialNames}
inclineOrient = {names: {plates: np.matrix(AngleAxisFromMatrix(inclineR[names][plates])) for plates in plateNames} for names in trialNames}

## Compile Level Data

levelOrientOutput = np.array([items for _, items in sorted(levelOrient.items())]).T
levelPositionOutput = np.array([worldT[plates] for plates in sorted(plateNames)]).T

## Compile Incline Data

inclinePositionOutput = {names: np.array([items for _, items in sorted(inclinePos[names].items())]).T for names in trialNames}
inclineOrientOutput = {names: np.array([items for _, items in sorted(inclineOrient[names].items())]).T for names in trialNames}

## Write Level Output Data to Excel

plateNameOutputHeaderDF = pd.DataFrame(list(plateNames)).T
levelOrientOutputDF = pd.DataFrame(levelOrientOutput[0])
levelPositionOutputDF = pd.DataFrame(levelPositionOutput)
column1DF = pd.DataFrame(['Position', 'x', 'y', 'z', 'Orientation', 'x','y', 'z'])

levelWriter = pd.ExcelWriter('InclineOutputPython.xlsx', engine='xlsxwriter')

column1DF.to_excel(levelWriter, header=False, index=False, sheet_name=levelTrialName)
plateNameOutputHeaderDF.to_excel(levelWriter, header=False, index=False, sheet_name=levelTrialName, startcol=1)
levelPositionOutputDF.to_excel(levelWriter, header=False, index=False, sheet_name=levelTrialName, startcol=1, startrow=1)
plateNameOutputHeaderDF.to_excel(levelWriter, header=False, index=False, sheet_name=levelTrialName, startcol=1, startrow=4)
levelOrientOutputDF.to_excel(levelWriter, header=False, index=False, sheet_name=levelTrialName, startcol=1, startrow=5)

## Write Incline Output Data to Excel

inclineOrientOutputDF = {names: pd.DataFrame(inclineOrientOutput[names][0]) for names in trialNames}
inclinePositionOutputDF = {names: pd.DataFrame(inclinePositionOutput[names][0]) for names in trialNames}

[column1DF.to_excel(levelWriter, header=False, index=False, sheet_name=trialNames[plateNum]) for plateNum, names in enumerate(trialNames)]
[plateNameOutputHeaderDF.to_excel(levelWriter, header=False, index=False, sheet_name=trialNames[plateNum], startcol=1) for plateNum, names in enumerate(trialNames)]
[inclinePositionOutputDF[names].to_excel(levelWriter, header=False, index=False, sheet_name=trialNames[plateNum], startcol=1, startrow=1) for plateNum, names in enumerate(trialNames)]
[plateNameOutputHeaderDF.to_excel(levelWriter, header=False, index=False, sheet_name=trialNames[plateNum], startcol=1, startrow=4) for plateNum, names in enumerate(trialNames)]
[inclineOrientOutputDF[names].to_excel(levelWriter, header=False, index=False, sheet_name=trialNames[plateNum], startcol=1, startrow=5) for plateNum, names in enumerate(trialNames)]

levelWriter.save()
