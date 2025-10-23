#!/bin/bash

# Define the udev rule file path
UDEV_RULE_FILE="/etc/udev/rules.d/99-reddog-serial.rules"

# Create or overwrite the udev rule file with the following content
sudo bash -c "cat > $UDEV_RULE_FILE" <<EOL
# Udev rules for RedDog USB Serial Devices
# Reddog
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", KERNEL=="ttyACM0", SYMLINK+="ttywheel", MODE="0666"

# Imu
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", KERNEL=="ttyACM1", SYMLINK+="ttysteering", MODE="0666"

EOL

# Reload udev rules to apply the new configurations
sudo udevadm control --reload-rules
sudo udevadm trigger

# Inform the user
echo "Udev rules have been successfully updated."
echo "You can now access the devices using /dev/ttywheel and /dev/ttysteering"
