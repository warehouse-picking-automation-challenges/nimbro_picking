#!/bin/bash
# UNINSTALL
rm -rf ~/torch

# INSTALL
cd ~
git clone https://github.com/torch/distro.git ~/torch --recursive
cd ~/torch; 
# bash install-deps;
./install.sh

source ~/.bashrc

# install dependencies
luarocks install nn
luarocks install optim
luarocks install image
luarocks install cunn
luarocks install cudnn
luarocks install cutorch
luarocks install luafilesystem
luarocks install matio