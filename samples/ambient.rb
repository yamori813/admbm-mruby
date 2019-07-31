#
# mruby on YABM script
#
# ambient update script
#

WRITEKEY = "naisyo"
CHANNELID = "naisyo"

INTERVAL = 10

def delay(yabm, val) 
  start = yabm.count() 
  while yabm.count() < start + val do
  end
end

begin

yabm = YABM.new

yabm.gpiosetdir(1 << 3)

yabm.gpiosetdat(1 << 3)

# net setup

yabm.netstartdhcp

yabm.print yabm.getaddress + "\n"

count = 0

while 1 do
  yabm.print "."
  count = count + 1
  yabm.print " " + count.to_s

  body='{ "writeKey":"' + WRITEKEY + '", "d1":' + count.to_s + '}'

  header = Hash.new
  header.store('User-Agent', "test-agent")
  header.store('Content-Type', "application/json")
  header.store('Body', body)
  path = "/api/v2/channels/" + CHANNELID + "/data"
  yabm.gpiosetdat(0)
  res = SimpleHttp.new("http", "ambidata.io", 80).post(path, header)
  if res 
    yabm.print " " + res.status.to_s
  end
  yabm.gpiosetdat(1 << 3)
  yabm.print "\n"
  delay(yabm, INTERVAL * 1000)
end

rescue => e
  yabm.print e.to_s
end
