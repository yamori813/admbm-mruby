This projct use these libraries.

newlib-3.0.0.20180831  
lwip-2.1.2  
bearssl-0.6  
mruby (git submodule)  

Build tools.  

gcc 4.9.2 

Also use mkimage command.  

I build on FreeBSD/amd64 11.2 used by linux emuration.  

Work on ADM5120 and ADM5120P

Build u-boot image.  

% make image  

Defalte script is samples/hello.rb  

Custom script build is this.  

% make image RBSCRIPT=myscript.rb  

Todo  

2038 problem  
real memory size  
UART support  
Switch support  
I2C support  
USB Support  
IPv6 support  
etc.  
