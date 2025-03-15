#!/bin/sh
cd /home/rob/workspace/Final_Code/PatrolRob_bag/CH4_decode/build
sudo ./CH4_decode &
sleep 1

cd /home/rob/workspace/Final_Code/PatrolRob_bag/DOVE_E4/build
sudo ./Dove_E4 &
sleep 1

cd /home/rob/workspace/Final_Code/PatrolRob_bag/H2S_decode/build
sudo ./H2S_decode &
sleep 1

cd /home/rob/workspace/Final_Code/PatrolRob_bag/VOC_decode/build
sudo ./VOC_decode &
sleep 1

cd /home/rob/workspace/Final_Code/PatrolRob_bag/ZSHRobot_ws/build
sudo ./ZSHRobot &

