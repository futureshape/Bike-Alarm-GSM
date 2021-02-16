# Bike-Alarm-GSM

This is a simple attempt at implementing an alarm that sends a notification via GSM when an object has been moved without permission. 

It was designed as an alarm for bicycle, but can really be used for any other moveable object.

## Ingredients

* [TTGO T-Call](https://github.com/Xinyuan-LilyGO/LilyGo-T-Call-SIM800) ESP32 board with built in GSM module 
* [ADXL362](https://coolcomponents.co.uk/products/triple-axis-accelerometer-breakout-adxl362) accelerometer board - this is a very low-power accelerometer SPI interface and a configurable way of producing motion interrupts
* A buzzer (optional, if you want your alarm to make noise)
* A GSM (2G) SIM card from [Hologram](https://www.hologram.io/) - you can get a free SIM on their "Pilot SIM" plan with 1MB data per month. Since the Hologram platform is used for notifications, you'll need to modify the code if you want to use another SIM provider.
* A big enough battery - because the ESP32 sleeps most of the time, and the accelerometer is a low-power part, a 3000mAH 18650 cell can last for at least 2 months (recharging works via the ESP32 board)
* An iPhone with a custom app (TBC) for the BLE lock/unlock functionality

## Connections 

TBC

## How it works

1. By default, go to sleep in "armed" state and wait to be woken up by an accelerometer interrupt
2. If woken up, start up a BLE server and advertise
3. If an iPhone with the unlock app is around, it will pick up the advertisement, wake up the app in the background, and write a password to the BLE characteristic
4. If the password is correct, go back to sleep in "disarmed" state and periodically wake up to check that the iPhone is still there. If not, go back to sleep in "armed" state
5. If the password is incorrect or no iPhone tried to connect, go into "prealarm" state and keep watching for movement
6. If movement continues for a period of time, use the GSM module to send an alarm
