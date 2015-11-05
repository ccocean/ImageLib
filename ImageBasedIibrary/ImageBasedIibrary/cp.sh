#!/bin/bash
if [ "$1" = "" ]
then
echo "please enter mcfw dir"
else
echo "cp track file"
echo "$1/interfaces"
cp tch_params.h $1/interfaces/link_api
cp tch_track.c $1/src_bios6/alg/teachtrack/src/
cp tch_track.h $1/src_bios6/alg/teachtrack

cp stuTrack_settings_parameter.h $1/interfaces/link_api
cp stuTrack_track_img.h $1/src_bios6/alg/stutrack/
cp stuTrack_track_img.c $1/src_bios6/alg/stutrack/src

cp itc* $1/src_bios6/alg/teachtrack/src/
fi
