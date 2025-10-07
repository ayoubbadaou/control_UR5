#!/bin/bash
gnome-terminal --tab --title="Data Runner" --command "bash -c 'python3 /home/robot/Bureau/Data_run/data_run.py; exec bash'" \
               --tab --title="DB Publisher" --command "bash -c 'python3 /home/robot/Bureau/Node-red_storage/Db_pub_read/db_pub.py; exec bash'" \
               --tab --title="DB Reader" --command "bash -c 'sleep 2; python3 /home/robot/Bureau/Node-red_storage/Db_pub_read/db_read.py; exec bash'"
