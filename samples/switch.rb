#
# BBR-4MG switch port check script
#

begin

addr = "10.10.10.2"
mask = "255.255.255.0"
gw = "10.10.10.1"
dns = "10.10.10.1"

usegmii = 0

yabm = YABM.new

yabm.netstart(addr, mask, gw, dns)

loop do
  yabm.print "===\r\n"

  dat = yabm.getphyst
  for port in 0..4 do
    if dat & (1 << port) != 0
      yabm.print port.to_s + " is "
      if dat & (1 << (port + 8)) != 0
        yabm.print "100M "
      else
        yabm.print "10M "
      end
      if dat & (1 << (port + 16)) != 0
        yabm.print "Full duplex"
      else
        yabm.print "Half duplex"
      end
      yabm.print "\r\n"
    else
      yabm.print port.to_s + " is off\r\n"
    end
  end

  if usegmii == 1
    for reg in 0..4 do
      for port in 0..31 do
        dat = yabm.readmdio(port, reg);
        yabm.print dat.to_s + " "
      end
      yabm.print "\r\n"
    end
  end
  yabm.msleep 10_000
end

rescue => e
  yabm.print e.to_s
end
