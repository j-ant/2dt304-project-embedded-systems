import network
import time
import gc

gc.collect()

ssid = 'TP Link 2.4'
password = 'mjkobedurant'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

max_wait_time = 15

while max_wait_time > 0:
  if wlan.status() < 0 or wlan.status() >= 3:
    break
  max_wait_time -= 1
  print('Waiting for connection...')
  time.sleep(1)

if wlan.status() != 3:
  raise RuntimeError('Wifi Connection Failed!')

else:
  print('Connection Successful!')
  status = wlan.ifconfig()
  print('ip: ' + status[0])


