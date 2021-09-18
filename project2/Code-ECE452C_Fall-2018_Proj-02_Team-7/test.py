import lightmove

angle = 180

try:
    lightmove.followlightmove(angle)
except KeyboardInterrupt:
    lightmove.Ab.stop()
