# This will remove this file here if it exists: /etc/udev/rules.d/99-serial.rules 
#   and recreate it with the following cat command

rm -rf /etc/udev/rules.d/99-serial.rules 
touch /etc/udev/rules.d/99-serial.rules 

cat <<EOF >/etc/udev/rules.d/99-serial.rules 
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
EOF