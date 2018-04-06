#! /bin/sh
# return EETF to ARTagTF
roslaunch handeye_calib_camodocal handeye_tf.launch \
  data_folder:=~/macaca/data/handeye/eyetracker/scene/left/ \
  ARTagTF:=scene/left/camera_link  \  
  cameraTF:=scene/left/marker_ar_0  \ 
  EETF:=eyetracker_link  \
  baseTF:=lighthouse_link
