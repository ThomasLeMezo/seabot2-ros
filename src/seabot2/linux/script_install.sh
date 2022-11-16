#!/usr/bin/zsh
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.driver.service /etc/systemd/system/
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.service /etc/systemd/system/
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.record.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable seabot2.driver.service
sudo systemctl enable seabot2.record.service
sudo systemctl enable seabot2.service

cp ~/seabot2-ros/src/seabot2/config/* ~/config/
