#!/usr/bin/zsh
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.driver.service /etc/systemd/system/
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.service /etc/systemd/system/
sudo cp ~/seabot2-ros/src/seabot2/linux/seabot2.record.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable seabot2.driver.service
sudo systemctl enable seabot2.record.service
sudo systemctl enable seabot2.service
sudo systemctl enable fake-hwclock.service

sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

sudo mkdir -p /opt/bin
cd /opt/bin
sudo ln -s /home/pi/seabot2-ros/src/seabot2/linux/wtf.sh wtf
gsettings set org.gnome.desktop.lockdown disable-lock-screen true
gsettings set org.gnome.desktop.lockdown disable-log-out true

# @reboot screen -dmS wtf_daemon zsh -c "wtf"

# sudo useradd -M rosuser
# sudo usermod -L rosuser
# sudo usermod -a -G pi,adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,netdev,lxd rosuser
# sudo chown rosuser /home/rosuser