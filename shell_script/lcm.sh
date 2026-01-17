sudo ufw disable
sudo ip link set eth0 multicast on
sudo ip route add 224.0.0.0/4 dev eth0
ip route | grep 224