#
# rtlbm-mruby mruby script
#

begin

rtl = YABM.new

ledout = [0x33, 0x32, 0x23, 0x22]

i = 0
j = 0
while 1 do
   start = rtl.count() 
   while rtl.count() < start + 1000 do
   end
   rtl.gpiosetled(j, ledout[i])
   rtl.print j.to_s + ":" + ledout[i].to_s + "\n"
   i = i + 1
   if i == 4
     j = j + 1
     if j == 5
       j = 0
     end
     i = 0
   end
end

rescue => e
  rtl.print e.to_s
end
