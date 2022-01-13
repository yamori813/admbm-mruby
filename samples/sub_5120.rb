#
# yabm mruby script
# ADM5120 subroutine script
#

def gpioinit yabm
  yabm.gpiosetdir(1 << 3)

  yabm.gpiosetdat(1 << 3)
end

def ledon yabm
    yabm.gpiosetdat(0)
end

def ledoff yabm
  yabm.gpiosetdat(1 << 3)
end
