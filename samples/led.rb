#
# BBR-4MG LED test script
#

begin

yabm = YABM.new

ledout = [0x33, 0x32, 0x23, 0x22]

i = 0
j = 0
while 1 do
   start = yabm.count() 
   while yabm.count() < start + 1000 do
   end
   yabm.gpiosetled(j, ledout[i])
   yabm.print j.to_s + ":" + ledout[i].to_s + "\n"
   i += 1
   if i == 4
     j += 1
     if j == 5
       j = 0
     end
     i = 0
   end
end

rescue => e
  yabm.print e.to_s
end
