robot_wifi_device="enp4s0"
# Visionパケットをロボットに送信しない
sudo iptables -A FORWARD -o $robot_wifi_device -d 224.5.23.2 -j DROP
# Refereeパケットをロボットに送信しない
sudo iptables -A FORWARD -o $robot_wifi_device -d 224.5.23.1 -j DROP
