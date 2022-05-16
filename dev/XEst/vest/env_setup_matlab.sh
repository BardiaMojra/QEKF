#!/bin/bash
# pause function
function pause(){
 read -s -n 1 -p "Press any key to continue..."
 echo ""
}
sudo apt clean && sudo apt update

printf '\n\n\n --->> download rvc01 (robotic vision and control toolbox v01) and copy vision directory to path... \n'
printf ' --->> then, press any key to continue...\n' 
pause
printf '\n\n\n --->> downloading rtb smtb and common toolboxes... \n'
git clone https://github.com/petercorke/robotics-toolbox-matlab rtb
git clone https://github.com/petercorke/spatial-math smtb
git clone https://github.com/petercorke/toolbox-common-matlab common
