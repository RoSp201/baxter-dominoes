#!/bin/bash
START_TAG=0
END_TAG=9
i=$START_TAG
while [ $i -lt $END_TAG ];do
    rosrun follow ik_from_ar_tag_pos.py $i
    i=$(expr $i + 1)
done
rosrun follow ik_from_ar_tag_pos.py $START_TAG
