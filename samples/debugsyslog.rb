#
# syslog debug script
#

begin

yabm = YABM.new

addr = "10.10.10.123"
mask = "255.255.255.0"
gw = "10.10.10.3"
dns = "10.10.10.3"

yabm.netstart(addr, mask, gw, dns)

dist = "10.10.10.3"

yabm.udpinit

i = i + 1

rescue => e
  yabm.print e.to_s
  yabm.udpsend(dist, 514, e.to_s, e.to_s.length)
end
