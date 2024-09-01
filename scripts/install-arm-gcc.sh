# Install ARM GCC toolchain (arm-none-eabi-gcc/g++) on linux
# can be used for CI testing or CodeQL, for final build use PROS
# MUST BE RUN WITH SUDO

set +x

# Variables
install_version = 13.3.rel1
install_path = /usr/share/

# DO NOT MODIFY BELOW THIS LINE

# Remove outdated toolchain (if it exists)

echo "Removing outdated Launchpad toolchain (if it exists)"
sudo apt remove gcc-arm-none-eabi

# Check if ARM GCC is already installed
if command -v arm-none-eabi-gcc &>/dev/null; then
    echo "arm-none-eabi-gcc is already installed. Halting script execution."
    exit 1
fi

# Install deps
echo "Installing deps: wget, libncurses5"
sudo apt update
sudo apt install wget
sudo apt install libncurses5

# Download and extract the toolchain
sudo wget -P $install_path https://developer.arm.com/-/media/Files/downloads/gnu/$install_version/binrel/arm-gnu-toolchain-$install_version-x86_64-arm-none-eabi.tar.xz
sudo tar -xJf /usr/share/arm-gnu-toolchain-$install_version-x86_64-arm-none-eabi.tar.xz -C $install_path

install_bin_dir = $install_path/arm-gnu-toolchain-$install_version-x86_64-arm-none-eabi/bin
echo "Extracted toolchain binaries to $install_bin_dir"

# Setup symlinks
sudo ln -s $install_bin_dir/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -s $install_bin_dir/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc
sudo ln -s $install_bin_dir/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy
sudo ln -s $install_bin_dir/arm-none-eabi-objsize /usr/bin/arm-none-eabi-objsize
echo "Created symlinks in /usr/bin for access to toolchain binaries"

# Final version test
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version

echo "ARM GCC toolchain installation complete"
