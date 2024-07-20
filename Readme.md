This projct use these libraries.

newlib-3.0.0.20180831  
lwip-2.1.2  
bearssl-0.6  
mruby (git submodule)  

Build tools.  

gcc 4.9.2 

Also use mkimage command.  

I build on FreeBSD/amd64 13.1 used by ports gcc and binutil. 

Work on ADM5120 and ADM5120P

Build VM. 

```
% make
```

Build u-boot image.  

```
% make image
```

Defalte script is samples/hello.rb.  

Custom script build is this.  

```
% make image RBSCRIPT=myscript.rb
```

Multi script file comple is this. 

```
% make image RBSCRIPT="sub.rb main.rb"
```


Todo  

2038 problem  
UART support  
Switch support  
etc.  
