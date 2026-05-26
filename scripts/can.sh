sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100


sudo ip link set can1 type can bitrate 1000000 
sudo ip link set can1 up
sudo ifconfig can1 txqueuelen 100


sudo ip link set can2 type can bitrate 1000000 
sudo ip link set can2 up
sudo ifconfig can2 txqueuelen 100


sudo ip link set can3 type can bitrate 1000000 
sudo ip link set can3 up
sudo ifconfig can3 txqueuelen 100