#!/usr/bin/zsh
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.driver.service /etc/systemd/system/
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable seabot2.driver.service
sudo systemctl enable seabot2.service
sudo systemctl enable fake-hwclock.service

sudo mkdir -p /opt/bin
cd /opt/bin
sudo ln -s /home/pi/seabot2-ros/install/seabot2/share/seabot2/linux/wtf.sh wtf

ln -s /home/pi/seabot2-ros/install/seabot2/share/seabot2/config config

# autologin
# sudo nano /etc/gdm3/custom.conf
#AutomaticLogginEnable=True
#AutomaticLoggin=pi