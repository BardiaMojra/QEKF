Script Name: Inclined_Treadmill_FP_Properties
Language: Matlab, Python (2.7)

Summary:

This script will calculate the position and orientation of instrumented treadmill force plates
at any given incline. The position andorientation of the plates in the level configuration are
required to be entered into the system configuration ahead of time. It is preferred that only
the treadmill's instrumented plates are defined within the system configuration. It is required
that all devices be given unique names.

The script also requires at least three non-collinear reference markers to be placed on the
treadmill frame and a VSK created for them. There are no other restrictions as to the number of
markers (greater than 3), their names or arrangement as long as they are not collinear. It is
recommended to distribute the markers around the perimeter of the treadmill frame. Although it
may be common practice for treadmills, the incline reference marker set (subject) does not need
to be the L-Frame used to set the volume origin.

A trial must be captured in the level position along with any number of trials in the desired
inclined positions. All trials must bereconstructed, labeled and saved prior to running this
script. There can be gaps in the marker data (don't fill the gaps) but all markers in the VSK
must have some associated data.

Trial names cannot contain spaces and must begin with a letter.

The script file should be copied to the session folder and run from there. Run it through Matlab,
not from the Nexus Matlab tab or pipeline operation. The script will prompt you to select the level
trial and then to multi-select all incline trials for processing. The script expects at least two
incline trials.If only one incline trial exists, simply select the level trial along with the
incline trial.

The script computes the optimal translation and rotation between the level marker configuration
and the inclined marker configuration. Then it applies the same transformation to the force plate
position and orientation values.

The script outputs a spreadsheet file to the session folder with a tab for each trial. Each tab
presents the position and orientation values for each force plate. These values are to be entered
into a new Nexus system configuration corresponding to an inclined position.

The script applies a number of functions defined within the script. Therefore, the script will need
to be run within a version of Matlab that supports built-in functions.

Dependencies: 
Matlab: None (originally created in Matlab 2018a)
Python modules: OS, Math, numpy, sys, Tkinter, tkMessageBox, Pandas 

Run in Vicon Nexus: No. Must be run within Matlab with Nexus running alongside.
Example Provided: Yes, (Processed in Nexus 2.10)
Author: John Porter, Nev Pires - Vicon Motion Systems, Inc.



